#!/bin/bash
ITERATION=$1; RESULT=$2; DETAIL=$3
cd ~/drone-firmware-agent || exit 1
git add -A
git commit -m "Iteration $ITERATION: $RESULT

Result : $RESULT
Detail : $DETAIL
Time   : $(date '+%Y-%m-%d %H:%M:%S')
"
git push origin master 2>/dev/null || git push origin main 2>/dev/null
echo "Committed iteration $ITERATION"
