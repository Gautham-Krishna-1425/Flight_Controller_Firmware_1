import json, os

LOG_FILE = os.path.join(os.path.dirname(__file__), '../logs/flight_data.json')

def analyze():
    if not os.path.exists(LOG_FILE):
        print("RESULT: NO_DATA | Log file not found"); return
    with open(LOG_FILE) as f: data = json.load(f)
    if len(data) < 10:
        print("RESULT: NO_DATA | Too few data points"); return
    rolls   = [d["roll"]   for d in data]
    pitches = [d["pitch"]  for d in data]
    alts    = [d["h_est"]  for d in data]
    u_rolls = [d["u_roll"] for d in data]
    max_roll  = max(abs(r) for r in rolls)
    max_pitch = max(abs(p) for p in pitches)
    final_alt = alts[-1]
    crossings = sum(1 for i in range(1,len(u_rolls)) if u_rolls[i]*u_rolls[i-1]<0)
    osc_rate  = crossings / (data[-1]["t"]+0.001)
    if max_roll > 60.0 or max_pitch > 60.0:
        print(f"RESULT: CRASH | max_roll={max_roll:.1f}° max_pitch={max_pitch:.1f}°")
    elif osc_rate > 3.0 or max_roll > 25.0 or max_pitch > 25.0:
        print(f"RESULT: OSCILLATING | osc_rate={osc_rate:.1f}/s max_roll={max_roll:.1f}°")
    elif max_roll < 8.0 and max_pitch < 8.0 and final_alt > 0.5:
        print(f"RESULT: STABLE | max_roll={max_roll:.1f}° max_pitch={max_pitch:.1f}° final_alt={final_alt:.2f}m")
    else:
        print(f"RESULT: UNSTABLE | max_roll={max_roll:.1f}° max_pitch={max_pitch:.1f}° final_alt={final_alt:.2f}m osc_rate={osc_rate:.1f}/s")

if __name__ == '__main__': analyze()
