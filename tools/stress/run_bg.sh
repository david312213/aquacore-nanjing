#!/bin/bash
# Run the stress test headless over SSH. Every process started here carries STRESS_RUN=<dir>
# in its environment, so `stop` signals only those processes (SIGINT, as Ctrl-C would).
#   bash tools/stress/run_bg.sh start <dir> <name> <command...>   log -> <dir>/<name>.log
#   bash tools/stress/run_bg.sh phase <dir> <label>               monitor reads <dir>/phases.txt
#   bash tools/stress/run_bg.sh stop  <dir> [name]                all, or one name
#   bash tools/stress/run_bg.sh list  <dir>
set -e
action=$1 dir=$2
shift 2
mkdir -p "$dir"
tagged() {  # pids whose environment has STRESS_RUN=$dir (and STRESS_NAME=$1 if given)
    for env in /proc/[0-9]*/environ; do
        pid=${env#/proc/}; pid=${pid%/environ}
        [ -r "$env" ] || continue  # other users' processes
        content=$( (tr '\0' '\n' < "$env") 2>/dev/null) || continue
        printf '%s\n' "$content" | grep -qx "STRESS_RUN=$dir" || continue
        [ -z "$1" ] || printf '%s\n' "$content" | grep -qx "STRESS_NAME=$1" || continue
        echo "$pid"
    done
}
case "$action" in
    start)
        name=$1; shift
        STRESS_RUN=$dir STRESS_NAME=$name setsid nohup "$@" > "$dir/$name.log" 2>&1 < /dev/null &
        echo "started $name"
        ;;
    phase)
        echo "$1" >> "$dir/phases.txt"
        ;;
    stop)
        pids=$(tagged "$1")
        [ -n "$pids" ] && kill -INT $pids 2>/dev/null || true
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            [ -z "$(tagged "$1")" ] && break
            sleep 1
        done
        left=$(tagged "$1")
        [ -n "$left" ] && kill -KILL $left 2>/dev/null || true
        echo "stopped ${1:-all}${left:+ (forced: $left)}"
        ;;
    list)
        for pid in $(tagged); do
            printf '%s %s\n' "$pid" "$(tr '\0' ' ' < /proc/$pid/cmdline | cut -c1-120)"
        done
        ;;
    *)
        echo "usage: run_bg.sh start|phase|stop|list <dir> ..." >&2
        exit 2
        ;;
esac
