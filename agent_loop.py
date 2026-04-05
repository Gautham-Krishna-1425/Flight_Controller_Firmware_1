import subprocess, time, os, sys, socket

MAX_ITERATIONS=20
SCRIPTS_DIR=os.path.expanduser('~/drone-firmware-agent/scripts')
FIRMWARE_DIR=os.path.expanduser('~/drone-firmware-agent/firmware/Core/Src')
MAIN_C=os.path.join(FIRMWARE_DIR, 'main.c')
AIDER_MODEL='gemini/gemini-2.5-flash'
FLIGHT_DURATION=35

PROMPTS={
"CRASH": f"The drone CRASHED (roll/pitch exceeded 60 degrees). Open {MAIN_C} and reduce K_po by 20% (change 0.8 to 0.64) and K_pi by 20% (change 0.035 to 0.028). Make the changes and explain what you did.",
"OSCILLATING": f"The drone OSCILLATED rapidly. Open {MAIN_C} and reduce K_pi by 15% and increase K_di by 20%. Make the changes and explain what you did.",
"UNSTABLE": f"The drone is UNSTABLE. Open {MAIN_C} and increase K_po by 10% (change 0.8 to 0.88). Make the changes and explain what you did.",
}

def free_port(port):
    """Kill any process using the given UDP port"""
    try:
        subprocess.run(f"fuser -k {port}/udp", shell=True,
                      capture_output=True)
        time.sleep(0.5)
    except:
        pass

def run(cmd, label, timeout=60):
    print(f"\n[{label}]")
    try:
        r=subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if r.stdout: print(r.stdout.strip())
        if r.stderr and 'error' in r.stderr.lower():
            print(f"ERR: {r.stderr.strip()[:300]}")
        return r
    except subprocess.TimeoutExpired:
        print(f"Timed out after {timeout}s"); return None

def parse_result(output):
    if not output: return "UNKNOWN", ""
    for line in output.strip().splitlines():
        line=line.strip()
        if line.startswith("RESULT:"):
            parts=line.split("|")
            tag=parts[0].replace("RESULT:","").strip()
            detail=parts[1].strip() if len(parts)>1 else ""
            return tag, detail
    return "UNKNOWN", ""

if not os.environ.get('GEMINI_API_KEY'):
    print("Set GEMINI_API_KEY first!"); sys.exit(1)

iteration=0; last_result=None
print("\n"+"="*55+"\n  DRONE FIRMWARE AGENTIC LOOP\n"+"="*55)

while iteration < MAX_ITERATIONS:
    iteration += 1
    print(f"\n{'='*55}\n  ITERATION {iteration}/{MAX_ITERATIONS}\n{'='*55}")

    # Free port 14557 before each flight
    free_port(14557)
    time.sleep(1)

    # Run flight
    run(['python3', f'{SCRIPTS_DIR}/arm_and_bridge.py'],
        "1/4 Flight Simulation", FLIGHT_DURATION+15)

    # Free port again after flight
    free_port(14557)

    # Analyze
    a=run(['python3', f'{SCRIPTS_DIR}/log_analyzer.py'], "2/4 Analysis", 15)
    result, detail = parse_result(a.stdout if a else "")
    last_result=result
    print(f"\n  → Result: {result} | {detail}")

    # Git commit
    run(['bash', f'{SCRIPTS_DIR}/git_commit.sh',
         str(iteration), result, detail], "3/4 Git Commit", 30)

    if result == "STABLE":
        print("\n✅ FIRMWARE STABLE — Ready for real flight!")
        break

    # Aider fix — pass specific file not directory
    if result in PROMPTS:
        run(['aider', '--model', AIDER_MODEL,
             '--message', PROMPTS[result],
             '--yes',
             MAIN_C],           # ← specific file, not directory
            f"4/4 Aider Fix ({result})", 120)
    else:
        print(f"[4/4] Result '{result}' — no fix defined")

    print(f"\n  Iteration {iteration} done. Next in 5s...")
    time.sleep(5)

print(f"\nFinished after {iteration} iterations. Final: {last_result}")
