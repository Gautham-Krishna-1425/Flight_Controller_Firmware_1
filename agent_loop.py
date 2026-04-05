import subprocess, time, os, sys

MAX_ITERATIONS=20
SCRIPTS_DIR=os.path.expanduser('~/drone-firmware-agent/scripts')
FIRMWARE_DIR=os.path.expanduser('~/drone-firmware-agent/firmware/Core/Src')
AIDER_MODEL='gemini/gemini-2.5-flash'
FLIGHT_DURATION=35

PROMPTS={
"CRASH": f"The drone CRASHED (roll/pitch exceeded 60 degrees). In {FIRMWARE_DIR}/main.c reduce K_po by 20% (0.8 to 0.64) and K_pi by 20% (0.035 to 0.028). Explain all changes.",
"OSCILLATING": f"The drone OSCILLATED. In {FIRMWARE_DIR}/main.c reduce K_pi by 15% and increase K_di by 20%. Explain all changes.",
"UNSTABLE": f"The drone is UNSTABLE. In {FIRMWARE_DIR}/main.c increase K_po by 10% (0.8 to 0.88). Explain all changes.",
}

def run(cmd,label,timeout=60):
    print(f"\n[{label}]")
    try:
        r=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout)
        if r.stdout: print(r.stdout.strip())
        return r
    except subprocess.TimeoutExpired:
        print(f"Timed out after {timeout}s"); return None

if not os.environ.get('GEMINI_API_KEY'):
    print("Set GEMINI_API_KEY first!"); sys.exit(1)

iteration=0; last_result=None
print("\n"+"="*55+"\n  DRONE FIRMWARE AGENTIC LOOP\n"+"="*55)

while iteration<MAX_ITERATIONS:
    iteration+=1
    print(f"\n{'='*55}\n  ITERATION {iteration}/{MAX_ITERATIONS}\n{'='*55}")
    run(['python3',f'{SCRIPTS_DIR}/arm_and_bridge.py'],"1/4 Flight Simulation",FLIGHT_DURATION+15)
    a=run(['python3',f'{SCRIPTS_DIR}/log_analyzer.py'],"2/4 Analysis",15)
    result="UNKNOWN"; detail=""
    if a and a.stdout:
        line=a.stdout.strip()
        for r in ["STABLE","CRASH","OSCILLATING","UNSTABLE","NO_DATA"]:
            if r in line: result=r; detail=line.split("|")[-1].strip() if "|" in line else line; break
    last_result=result
    print(f"\n  → Result: {result} | {detail}")
    run(['bash',f'{SCRIPTS_DIR}/git_commit.sh',str(iteration),result,detail],"3/4 Git Commit",30)
    if result=="STABLE":
        print("\n✅ FIRMWARE STABLE — Ready for real flight!"); break
    if result in PROMPTS:
        run(['aider','--model',AIDER_MODEL,'--message',PROMPTS[result],
             '--yes','--no-git',FIRMWARE_DIR],f"4/4 Aider Fix ({result})",120)
    print(f"\n  Iteration {iteration} done. Next in 5s..."); time.sleep(5)

print(f"\nFinished after {iteration} iterations. Final: {last_result}")
