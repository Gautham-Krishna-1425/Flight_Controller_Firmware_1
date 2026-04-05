import subprocess, time, os, sys

MAX_ITERATIONS=20
SCRIPTS_DIR=os.path.expanduser('~/drone-firmware-agent/scripts')
FIRMWARE_DIR=os.path.expanduser('~/drone-firmware-agent/firmware/Core/Src')
AIDER_MODEL='gemini/gemini-2.5-flash'
FLIGHT_DURATION=35

PROMPTS={
"CRASH": f"The drone CRASHED. In {FIRMWARE_DIR}/main.c reduce K_po by 20% and K_pi by 20%. Explain changes.",
"OSCILLATING": f"The drone OSCILLATED. In {FIRMWARE_DIR}/main.c reduce K_pi by 15% and increase K_di by 20%. Explain changes.",
"UNSTABLE": f"The drone is UNSTABLE. In {FIRMWARE_DIR}/main.c increase K_po by 10%. Explain changes.",
}

def run(cmd, label, timeout=60):
    print(f"\n[{label}]")
    try:
        r=subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if r.stdout: print(r.stdout.strip())
        return r
    except subprocess.TimeoutExpired:
        print(f"Timed out after {timeout}s"); return None

def parse_result(output):
    """Extract result — must start with RESULT: on its own line"""
    if not output:
        return "UNKNOWN", ""
    for line in output.strip().splitlines():
        line = line.strip()
        if line.startswith("RESULT:"):
            parts = line.split("|")
            tag = parts[0].replace("RESULT:","").strip()
            detail = parts[1].strip() if len(parts)>1 else ""
            return tag, detail
    return "UNKNOWN", ""

if not os.environ.get('GEMINI_API_KEY'):
    print("Set GEMINI_API_KEY first!"); sys.exit(1)

iteration=0; last_result=None
print("\n"+"="*55+"\n  DRONE FIRMWARE AGENTIC LOOP\n"+"="*55)

while iteration < MAX_ITERATIONS:
    iteration += 1
    print(f"\n{'='*55}\n  ITERATION {iteration}/{MAX_ITERATIONS}\n{'='*55}")

    run(['python3', f'{SCRIPTS_DIR}/arm_and_bridge.py'],
        "1/4 Flight Simulation", FLIGHT_DURATION+15)

    a = run(['python3', f'{SCRIPTS_DIR}/log_analyzer.py'], "2/4 Analysis", 15)

    result, detail = parse_result(a.stdout if a else "")
    last_result = result
    print(f"\n  → Result: {result} | {detail}")

    run(['bash', f'{SCRIPTS_DIR}/git_commit.sh', str(iteration), result, detail],
        "3/4 Git Commit", 30)

    if result == "STABLE":
        print("\n✅ FIRMWARE STABLE — Ready for real flight!")
        break

    if result in PROMPTS:
        run(['aider','--model', AIDER_MODEL,
             '--message', PROMPTS[result],
             '--yes','--no-git', FIRMWARE_DIR],
            f"4/4 Aider Fix ({result})", 120)
    else:
        print(f"[4/4] Result '{result}' — skipping Aider")

    print(f"\n  Iteration {iteration} done. Next in 5s...")
    time.sleep(5)

print(f"\nFinished after {iteration} iterations. Final: {last_result}")
