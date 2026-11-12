#!/bin/bash
# Status of long runs: E3 stage-B workers, elapsed time, load
P=$(pgrep -f 'evaluate_e3.py B' | head -1)
echo "evaluate_e3 processes: $(pgrep -fc evaluate_e3)"
[ -n "$P" ] && echo "stage B elapsed: $(ps -o etime= -p $P)"
echo "gen_demos processes: $(pgrep -fc gen_demos)"
uptime
