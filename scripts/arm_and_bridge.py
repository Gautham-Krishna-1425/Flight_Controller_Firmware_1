from pymavlink import mavutil
import time, math, os, json, threading

SITL_UDP        = 'udpin:0.0.0.0:14557'
LOG_FILE        = os.path.join(os.path.dirname(__file__), '../logs/flight_data.json')
FLIGHT_DURATION = 30
HOVER_ALT       = -1.5  # NED frame — negative = up

def clamp(x, lo, hi): return max(lo, min(x, hi))

def run():
    print("Connecting...")
    mav = mavutil.mavlink_connection(SITL_UDP)
    mav.wait_heartbeat()
    print(f"Connected sysid={mav.target_system}")

    # Wait for clean EKF
    print("Waiting for EKF...")
    h_baseline = None
    for _ in range(30):
        pos = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if pos and abs(pos.z) < 100:
            h_baseline = pos.z
            print(f"EKF stable at z={h_baseline:.2f}m")
            break
        time.sleep(0.3)
    if h_baseline is None:
        h_baseline = 0.0
        print("EKF not stable — using z=0 baseline")

    target_z = h_baseline + HOVER_ALT  # target altitude in NED

    # Pre-arm: send position setpoints for 2s so PX4 accepts offboard
    print("Pre-arming with position setpoints...")
    for i in range(40):
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        mav.mav.set_position_target_local_ned_send(
            int(time.time()*1e3) & 0xFFFFFFFF,
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # position only
            0, 0, h_baseline,   # stay on ground during pre-arm
            0, 0, 0,
            0, 0, 0,
            0, 0)
        time.sleep(0.05)

    # Set OFFBOARD mode
    print("Setting OFFBOARD...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        209, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    # ARM
    print("Arming...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"ARM ACK: result={ack.result if ack else 'none'}")

    # Heartbeat thread — keep offboard alive
    stop_flag = threading.Event()
    def hb():
        while not stop_flag.is_set():
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.2)
    threading.Thread(target=hb, daemon=True).start()

    flight_data = []
    start = time.time()

    print("Flying...")
    while True:
        now = time.time()
        elapsed = now - start
        if elapsed >= FLIGHT_DURATION:
            break

        # Send position target — let PX4 handle altitude control
        mav.mav.set_position_target_local_ned_send(
            int(elapsed*1e3) & 0xFFFFFFFF,
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # use position only
            0, 0, target_z,     # x=0, y=0, z=target (NED)
            0, 0, 0,
            0, 0, 0,
            0, 0)

        # Read telemetry
        att = mav.recv_match(type='ATTITUDE', blocking=False)
        pos = mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)

        rd = pd = h = vz = 0.0
        if att:
            rd = math.degrees(att.roll)
            pd = math.degrees(att.pitch)
        if pos:
            h = -(pos.z - h_baseline)  # relative altitude, positive up
            vz = -pos.vz

        flight_data.append({
            "t":        round(elapsed, 3),
            "roll":     round(rd, 3),
            "pitch":    round(pd, 3),
            "h_est":    round(h, 3),
            "vz":       round(vz, 3),
            "throttle": 0.5
        })

        if len(flight_data) % 50 == 0:
            print(f"  t={elapsed:.1f}s roll={rd:.1f}° pitch={pd:.1f}° h={h:.2f}m")

        time.sleep(0.1)

    stop_flag.set()
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    with open(LOG_FILE, 'w') as f:
        json.dump(flight_data, f, indent=2)
    print(f"Done. {len(flight_data)} points saved")

if __name__ == '__main__':
    run()
