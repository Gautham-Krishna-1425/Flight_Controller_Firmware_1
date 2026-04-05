from pymavlink import mavutil
import time, json, math, os

SITL_UDP        = 'udpin:0.0.0.0:14557'
LOG_FILE        = os.path.join(os.path.dirname(__file__), '../logs/flight_data.json')
FLIGHT_DURATION = 30
HOVER_THROTTLE  = 0.50
TARGET_ALTITUDE = 1.0
LOOP_HZ         = 100
DT              = 1.0 / LOOP_HZ

K_PO   = 0.8
K_PI   = 0.035
K_II   = 0.030
K_DI   = 0.0018
ALPHA  = 0.015
INT_MIN = -0.15
INT_MAX =  0.15
U_CLAMP =  0.3

ALT_KP      = 1.2
ALT_KI      = 0.3
ALT_KD      = 0.6
ALT_OUT_MIN = -0.3
ALT_OUT_MAX =  0.3

K_YAW_P         = 0.08
GYRO_EMA_ALPHA  = 0.35

class AttitudeAxis:
    def __init__(self, k_po, k_pi, k_ii, k_di, alpha=0.015):
        self.K_po=k_po; self.K_pi=k_pi; self.K_ii=k_ii
        self.K_di=k_di; self.alpha=alpha
        self.integrator=0.0; self.prev_rate_meas=0.0; self.d_filt=0.0
        self.integrator_min=INT_MIN; self.integrator_max=INT_MAX

    def reset(self):
        self.integrator=0.0; self.prev_rate_meas=0.0; self.d_filt=0.0

    def update(self, desired_angle, angle_est, rate_est, dt):
        dt = max(0.0005, min(dt, 0.05))
        angle_err = max(-30.0, min(desired_angle - angle_est, 30.0))
        rate_des  = max(-150.0, min(self.K_po * angle_err, 150.0))
        rate_err  = rate_des - rate_est
        u_p = self.K_pi * rate_err
        self.integrator += self.K_ii * rate_err * dt
        self.integrator = max(self.integrator_min, min(self.integrator, self.integrator_max))
        u_i = self.integrator
        rate_meas_delta = max(-200.0, min(rate_est - self.prev_rate_meas, 200.0))
        self.prev_rate_meas = rate_est
        u_d_raw = -self.K_di * (rate_meas_delta / dt)
        lpf_coeff = dt / (self.alpha + dt)
        self.d_filt += lpf_coeff * (u_d_raw - self.d_filt)
        return max(-U_CLAMP, min(u_p + u_i + self.d_filt, U_CLAMP))

class AltitudePID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.out_min=out_min; self.out_max=out_max
        self.integrator=0.0; self.prev_err=0.0

    def update(self, desired_alt, h_est, vz, dt):
        desired_vz = 1.0 * (desired_alt - h_est)
        err = desired_vz - vz
        self.integrator += err * dt
        deriv = (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp*err + self.ki*self.integrator + self.kd*deriv
        return max(self.out_min, min(out, self.out_max))

def motor_mixer_x(throttle, u_roll, u_pitch, u_yaw):
    throttle = max(0.0, min(throttle, 1.0)) * 0.85
    clamp = lambda x: max(0.0, min(x, 1.0))
    return (clamp(throttle + u_roll - u_pitch - u_yaw),
            clamp(throttle - u_roll - u_pitch + u_yaw),
            clamp(throttle - u_roll + u_pitch - u_yaw),
            clamp(throttle + u_roll + u_pitch + u_yaw))

def run_bridge():
    import threading
    print("Connecting to PX4 SITL...")
    mav = mavutil.mavlink_connection(SITL_UDP)
    mav.wait_heartbeat()
    print(f"Connected — sysid={mav.target_system}")

    roll_ctrl  = AttitudeAxis(K_PO, K_PI, K_II, K_DI, ALPHA)
    pitch_ctrl = AttitudeAxis(K_PO, K_PI, K_II, K_DI, ALPHA)
    alt_pid    = AltitudePID(ALT_KP, ALT_KI, ALT_KD, ALT_OUT_MIN, ALT_OUT_MAX)

    gyro_roll_filt=gyro_pitch_filt=gyro_yaw_filt=0.0
    roll_f=pitch_f=0.0; angle_f_init=False
    is_airborne=was_airborne=False; takeoff_warmup=0
    flight_data=[]; start_time=time.time(); last_time=start_time

    while True:
        now=time.time(); elapsed=now-start_time
        if elapsed >= FLIGHT_DURATION:
            print(f"Flight complete ({FLIGHT_DURATION}s)")
            break
        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=0.05)
        if att is None: continue
        dt = max(0.001, min(now-last_time, 0.05)); last_time=now
        roll_deg=math.degrees(att.roll); pitch_deg=math.degrees(att.pitch)
        gyro_r_raw=math.degrees(att.rollspeed)
        gyro_p_raw=math.degrees(att.pitchspeed)
        gyro_y_raw=math.degrees(att.yawspeed)
        gyro_roll_filt  += GYRO_EMA_ALPHA*(gyro_r_raw-gyro_roll_filt)
        gyro_pitch_filt += GYRO_EMA_ALPHA*(gyro_p_raw-gyro_pitch_filt)
        gyro_yaw_filt   += GYRO_EMA_ALPHA*(gyro_y_raw-gyro_yaw_filt)
        if not angle_f_init: roll_f=roll_deg; pitch_f=pitch_deg; angle_f_init=True
        roll_f  += 0.35*(roll_deg-roll_f)
        pitch_f += 0.35*(pitch_deg-pitch_f)
        pos = mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        h_est=vz=0.0
        if pos: h_est=-pos.z; vz=-pos.vz
        if elapsed < 3.0:
            throttle_cmd = 0.3+(HOVER_THROTTLE-0.3)*(elapsed/3.0)
        else:
            throttle_cmd = HOVER_THROTTLE+alt_pid.update(TARGET_ALTITUDE,h_est,vz,dt)
        throttle_cmd=max(0.0,min(throttle_cmd,1.0))
        if throttle_cmd > 0.45: is_airborne=True
        if is_airborne and not was_airborne:
            roll_ctrl.reset(); pitch_ctrl.reset(); takeoff_warmup=60
        was_airborne=is_airborne
        ctrl_r=roll_f; ctrl_p=pitch_f; gr=gyro_roll_filt; gp=gyro_pitch_filt; gy=gyro_yaw_filt
        if not is_airborne or takeoff_warmup>0:
            gr=gp=ctrl_r=ctrl_p=0.0
            if takeoff_warmup>0: takeoff_warmup-=1
        u_roll  = roll_ctrl.update(0.0, ctrl_r, gr, dt)
        u_pitch = pitch_ctrl.update(0.0, ctrl_p, gp, dt)
        u_yaw   = max(-0.15, min((0.0-gy)*K_YAW_P, 0.15))
        m1,m2,m3,m4 = motor_mixer_x(throttle_cmd, u_roll, -u_pitch, u_yaw)
        if throttle_cmd > 0.10:
            m1=max(m1,0.05); m2=max(m2,0.05); m3=max(m3,0.05); m4=max(m4,0.05)
        mav.mav.set_actuator_control_target_send(
            int(elapsed*1e6), 0,
            mav.target_system, mav.target_component,
            [u_roll, -u_pitch, u_yaw, throttle_cmd, 0, 0, 0, 0])
        flight_data.append({"t":round(elapsed,3),"roll":round(roll_deg,3),
            "pitch":round(pitch_deg,3),"u_roll":round(u_roll,4),
            "u_pitch":round(u_pitch,4),"throttle":round(throttle_cmd,3),
            "h_est":round(h_est,3),"vz":round(vz,3),
            "m1":round(m1,3),"m2":round(m2,3),"m3":round(m3,3),"m4":round(m4,3)})
        if len(flight_data) % LOOP_HZ == 0:
            print(f"  t={elapsed:.1f}s roll={roll_deg:.1f}° pitch={pitch_deg:.1f}° h={h_est:.2f}m thr={throttle_cmd:.2f}")
        time.sleep(max(0, DT-(time.time()-now)))

    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    with open(LOG_FILE,'w') as f: json.dump(flight_data,f,indent=2)
    print(f"{len(flight_data)} data points saved → {LOG_FILE}")

if __name__ == '__main__':
    run_bridge()
