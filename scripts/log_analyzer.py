import json, os

LOG_FILE = os.path.join(os.path.dirname(__file__), '../logs/flight_data.json')

def analyze():
    if not os.path.exists(LOG_FILE):
        print("RESULT: NO_DATA | Log file not found"); return
    with open(LOG_FILE) as f: data = json.load(f)
    if len(data) < 10:
        print("RESULT: NO_DATA | Too few data points"); return

    rolls   = [d["roll"]     for d in data]
    pitches = [d["pitch"]    for d in data]
    thrs    = [d["throttle"] for d in data]
    alts    = [d["h_est"]    for d in data]

    max_roll  = max(abs(r) for r in rolls)
    max_pitch = max(abs(p) for p in pitches)
    avg_thr   = sum(thrs) / len(thrs)

    # Only count altitudes below 50m as valid — anything above is EKF drift
    valid_alts = [h for h in alts if abs(h) < 5]
    max_valid_alt = max(valid_alts) if valid_alts else 0
    ekf_drifted = len(valid_alts) < len(alts) * 0.7

    crossings = sum(1 for i in range(1, len(rolls)) if rolls[i]*rolls[i-1] < 0)
    osc_rate  = crossings / (data[-1]["t"] + 0.001)

    if max_roll > 60.0 or max_pitch > 60.0:
        print(f"RESULT: CRASH | max_roll={max_roll:.1f}° max_pitch={max_pitch:.1f}°")
        return
    if ekf_drifted:
        print(f"RESULT: UNSTABLE | EKF drifted above 50m — valid={len(valid_alts)}/{len(alts)}")
        return
    if osc_rate > 3.0 or max_roll > 25.0 or max_pitch > 25.0:
        print(f"RESULT: OSCILLATING | osc_rate={osc_rate:.1f}/s max_roll={max_roll:.1f}°")
        return
    if max_roll < 8.0 and max_pitch < 8.0 and max_valid_alt > 0.3:
        print(f"RESULT: STABLE | max_roll={max_roll:.1f}° max_pitch={max_pitch:.1f}° max_alt={max_valid_alt:.2f}m")
        return
    print(f"RESULT: UNSTABLE | max_roll={max_roll:.1f}° max_pitch={max_pitch:.1f}° max_alt={max_valid_alt:.2f}m osc={osc_rate:.1f}/s")

if __name__ == '__main__': analyze()
