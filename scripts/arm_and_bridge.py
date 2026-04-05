from pymavlink import mavutil
import time, math, os, json, threading

SITL_UDP='udpin:0.0.0.0:14557'
LOG_FILE=os.path.join(os.path.dirname(__file__),'../logs/flight_data.json')
FLIGHT_DURATION=30; DT=0.01
K_PO=0.8; K_PI=0.035; K_II=0.030; K_DI=0.0018; ALPHA=0.015
INT_MIN=-0.15; INT_MAX=0.15; U_CLAMP=0.3
HOVER_THROTTLE=0.50; TARGET_ALTITUDE=1.0
ALT_KP=0.3; ALT_KI=0.05; ALT_KD=0.1
GYRO_EMA_ALPHA=0.35; K_YAW_P=0.08

class AttitudeAxis:
    def __init__(self): self.integrator=0.0; self.prev_rate_meas=0.0; self.d_filt=0.0
    def reset(self): self.integrator=0.0; self.prev_rate_meas=0.0; self.d_filt=0.0
    def update(self,des,ang,rate,dt):
        dt=max(0.0005,min(dt,0.05))
        rate_des=max(-150.0,min(K_PO*max(-30.0,min(des-ang,30.0)),150.0))
        rate_err=rate_des-rate
        u_p=K_PI*rate_err
        self.integrator=max(INT_MIN,min(self.integrator+K_II*rate_err*dt,INT_MAX))
        dm=max(-200.0,min(rate-self.prev_rate_meas,200.0)); self.prev_rate_meas=rate
        lpf=dt/(ALPHA+dt); self.d_filt+=lpf*(-K_DI*(dm/dt)-self.d_filt)
        return max(-U_CLAMP,min(u_p+self.integrator+self.d_filt,U_CLAMP))

class AltPID:
    def __init__(self): self.integrator=0.0; self.prev_err=0.0
    def update(self,des,h,vz,dt):
        err=max(-1.0,min(des-h,1.0))-vz
        self.integrator+=err*dt
        d=(err-self.prev_err)/dt; self.prev_err=err
        return max(-0.3,min(ALT_KP*err+ALT_KI*self.integrator+ALT_KD*d,0.3))

def clamp(x): return max(0.0,min(x,1.0))

def run():
    print("Connecting...")
    mav=mavutil.mavlink_connection(SITL_UDP)
    mav.wait_heartbeat()
    print(f"Connected sysid={mav.target_system}")

    # Wait for EKF to give sane altitude
    print("Waiting for EKF...")
    h_baseline=None
    for _ in range(30):
        pos=mav.recv_match(type='LOCAL_POSITION_NED',blocking=True,timeout=1)
        if pos and abs(-pos.z) < 100:
            h_baseline=-pos.z
            print(f"EKF stable at h={h_baseline:.2f}m")
            break
        time.sleep(0.3)
    if h_baseline is None:
        h_baseline=0.0
        print("EKF not stable — using h=0 baseline")

    # Pre-arm offboard setpoints
    print("Pre-arming...")
    for i in range(40):
        mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        mav.mav.set_attitude_target_send(int(time.time()*1e3)&0xFFFFFFFF,
            mav.target_system,mav.target_component,0b00000111,[1,0,0,0],0,0,0,0.3)
        time.sleep(0.05)

    # OFFBOARD mode
    print("Setting OFFBOARD...")
    mav.mav.command_long_send(mav.target_system,mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,209,6,0,0,0,0,0)
    time.sleep(0.5)

    # ARM
    print("Arming...")
    mav.mav.command_long_send(mav.target_system,mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
    time.sleep(1)
    ack=mav.recv_match(type='COMMAND_ACK',blocking=True,timeout=3)
    print(f"ARM ACK: result={ack.result if ack else 'none'}")

    # Heartbeat thread — 5Hz to keep offboard alive
    stop_flag=threading.Event()
    def hb(): 
        while not stop_flag.is_set():
            mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            time.sleep(0.2)
    threading.Thread(target=hb,daemon=True).start()

    roll_c=AttitudeAxis(); pitch_c=AttitudeAxis(); alt_c=AltPID()
    gyro_r=gyro_p=gyro_y=roll_f=pitch_f=0.0
    angle_init=False; flight_data=[]; start=time.time(); last=start

    print("Flying...")
    while True:
        now=time.time(); elapsed=now-start
        if elapsed>=FLIGHT_DURATION: break
        att=mav.recv_match(type='ATTITUDE',blocking=True,timeout=0.05)
        if att is None: continue
        dt=max(0.001,min(now-last,0.05)); last=now
        rd=math.degrees(att.roll); pd=math.degrees(att.pitch)
        gyro_r+=GYRO_EMA_ALPHA*(math.degrees(att.rollspeed)-gyro_r)
        gyro_p+=GYRO_EMA_ALPHA*(math.degrees(att.pitchspeed)-gyro_p)
        gyro_y+=GYRO_EMA_ALPHA*(math.degrees(att.yawspeed)-gyro_y)
        if not angle_init: roll_f=rd; pitch_f=pd; angle_init=True
        roll_f+=0.35*(rd-roll_f); pitch_f+=0.35*(pd-pitch_f)
        pos=mav.recv_match(type='LOCAL_POSITION_NED',blocking=False)
        h=vz=0.0
        if pos:
            raw_h=-pos.z
            h=raw_h-h_baseline  # relative altitude
            vz=-pos.vz
        thr=0.3+(HOVER_THROTTLE-0.3)*(elapsed/3.0) if elapsed<3.0 else clamp(HOVER_THROTTLE+alt_c.update(TARGET_ALTITUDE,h,vz,dt))
        ur=roll_c.update(0,roll_f,gyro_r,dt)
        up=pitch_c.update(0,pitch_f,gyro_p,dt)
        uy=max(-0.15,min(-gyro_y*K_YAW_P,0.15))
        mav.mav.set_attitude_target_send(int(elapsed*1e3)&0xFFFFFFFF,
            mav.target_system,mav.target_component,0b00000000,
            [1,0,0,0],ur,up,uy,thr)
        flight_data.append({"t":round(elapsed,3),"roll":round(rd,3),
            "pitch":round(pd,3),"h_est":round(h,3),"vz":round(vz,3),"throttle":round(thr,3)})
        if len(flight_data)%100==0:
            print(f"  t={elapsed:.1f}s roll={rd:.1f}° pitch={pd:.1f}° h={h:.2f}m thr={thr:.2f}")
        time.sleep(max(0,DT-(time.time()-now)))

    stop_flag.set()
    os.makedirs(os.path.dirname(LOG_FILE),exist_ok=True)
    with open(LOG_FILE,'w') as f: json.dump(flight_data,f,indent=2)
    print(f"Done. {len(flight_data)} points saved")

if __name__=='__main__': run()
