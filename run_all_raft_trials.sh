#!/bin/bash
# Complete Raft rerun - 5 trials per condition
# Paper timing: t=10s fault injection

echo "=== Raft Clean (5 trials) ==="
for i in {1..5}; do
    echo "Trial $i/5..."
    python run_trial.py --controller raft --fault none --trial $i
    sleep 5
done

echo "=== Raft Freeze Leader (5 trials) ==="
for i in {1..5}; do
    echo "Trial $i/5..."
    python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial $i
    sleep 5
done

echo "=== Raft Self-Injure (5 trials) ==="
for i in {1..5}; do
    echo "Trial $i/5..."
    python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial $i
    sleep 5
done

echo "=== Raft Bad Leader (5 trials) ==="
for i in {1..5}; do
    echo "Trial $i/5..."
    python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial $i
    sleep 5
done

echo "=== DONE ==="
