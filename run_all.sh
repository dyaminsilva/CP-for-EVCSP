#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

objective=${1:-makespan}
if [[ $# -gt 1 || ( "$objective" != makespan && "$objective" != costs ) ]]; then
    echo "Usage: bash run_all.sh [makespan|costs]" >&2
    exit 1
fi

# Keep the lock while this batch or its current solver is running.
exec 9>.batch.lock
flock -n 9 || { echo "A batch is already running in $PWD." >&2; exit 1; }

if [[ ! -x ./CP-for-EVCSP || ! -d E-VCSP-CGG-instances ]]; then
    echo "Build the program and supply E-VCSP-CGG-instances first." >&2
    exit 1
fi
if [[ -e results-MILP.txt || -e results-CP.txt || -e logs ]]; then
    echo "Existing results or logs found. Archive them before starting a new batch." >&2
    exit 1
fi

# Use the standalone inputs in the same group order as the original AP list.
instances=()
for size in 25 50; do
    for group in R1 C1 RC1 R2 C2 RC2; do
        for file in E-VCSP-CGG-instances/"$group"??-"$size".txt; do
            [[ -f "$file" ]] || continue
            instance=${file##*/}
            instances+=("${instance%.txt}")
        done
    done
done
if [[ ${#instances[@]} -eq 0 ]]; then
    echo "No instance TXT files found." >&2
    exit 1
fi
total=$((${#instances[@]} * 2))
mkdir logs
completed=0
failed=0
run=0

for instance in "${instances[@]}"; do
    for solver in MILP CP; do
        run=$((run + 1))
        echo "[$(date '+%F %T')] $run/$total: $instance $solver $objective"
        if ./CP-for-EVCSP "$instance" "$solver" "$objective" >"logs/$instance-$solver.log" 2>&1; then
            completed=$((completed + 1))
        else
            code=$?
            failed=$((failed + 1))
            echo "ERROR: $instance $solver exited with code $code; see logs/$instance-$solver.log" >&2
        fi
    done
done

echo "[$(date '+%F %T')] Finished: $completed/$total completed; $failed errors."
exit "$((failed > 0))"
