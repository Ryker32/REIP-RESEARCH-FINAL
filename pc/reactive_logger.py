#!/usr/bin/env python3
"""
Passive Reactive Trust Model Logger

Runs alongside the real REIP system (does NOT affect robot behavior).
Listens to robot broadcasts and computes what a REACTIVE trust model
(based on coverage prediction error, like the original simulation)
would have detected -- and when.

This gives the proactive-vs-reactive detection timing comparison
for the ISEF poster.

Usage: python3 reactive_logger.py [experiment_name]
"""

import socket
import json
import time
import math
import sys
import os
from datetime import datetime
from collections import defaultdict

UDP_PEER_PORT = 5200
ARENA_WIDTH = 2000
ARENA_HEIGHT = 1500
CELL_SIZE = 125

# Reactive model parameters (from original simulation)
REACTIVE_WINDOW = 5.0      # seconds to measure coverage gain
REACTIVE_THRESHOLD = 0.5   # If actual gain < threshold * expected, flag
REACTIVE_DECAY_RATE = 0.15 # Trust decay per bad window
REACTIVE_RECOVERY = 0.05   # Trust recovery per good window
REACTIVE_IMPEACH = 0.3     # Impeachment threshold

total_cells = (ARENA_WIDTH // CELL_SIZE) * (ARENA_HEIGHT // CELL_SIZE)


class ReactiveModel:
    """Simulates a reactive trust model that monitors coverage error."""
    
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.trust = 1.0
        self.leader_id = None
        
        # Coverage tracking
        self.known_visited = set()
        self.coverage_history = []  # (timestamp, coverage_count)
        
        # Detection timing
        self.first_flag_time = None
        self.first_flag_commands = 0
        self.impeachment_time = None
        self.windows_checked = 0
        self.bad_windows = 0
    
    def update(self, msg):
        """Process a robot broadcast message."""
        rid = msg.get('robot_id')
        if rid is None:
            return
        
        # Track leader
        leader = msg.get('leader_id')
        if leader is not None:
            self.leader_id = leader
        
        # Track coverage (from known_visited_count in log)
        known_count = msg.get('known_visited_count', 0)
        self.coverage_history.append((time.time(), known_count))
        
        # Clean old history
        cutoff = time.time() - REACTIVE_WINDOW * 2
        self.coverage_history = [(t, c) for t, c in self.coverage_history if t > cutoff]
    
    def check_reactive_trust(self):
        """
        Reactive check: Is coverage progressing as expected?
        
        This is what the simulation's trust model does:
        - Predict: "With N robots exploring, we should gain ~X cells per window"
        - Observe: "We actually gained Y cells"
        - If Y << X, trust decays
        
        The key weakness: you have to WAIT for coverage to stall before detecting.
        """
        if len(self.coverage_history) < 2:
            return
        
        now = time.time()
        recent = [(t, c) for t, c in self.coverage_history if t > now - REACTIVE_WINDOW]
        old = [(t, c) for t, c in self.coverage_history 
               if now - REACTIVE_WINDOW * 2 < t < now - REACTIVE_WINDOW]
        
        if not recent or not old:
            return
        
        self.windows_checked += 1
        
        # Observed gain in this window
        current_cov = max(c for _, c in recent)
        prev_cov = max(c for _, c in old) if old else 0
        observed_gain = current_cov - prev_cov
        
        # Expected gain: with 5 robots at ~2 cells/sec, expect ~10 cells per 5s window
        # Scale down as we approach full coverage
        remaining = total_cells - current_cov
        expected_gain = min(10, remaining * 0.1)  # Conservative estimate
        
        if expected_gain > 0 and observed_gain < REACTIVE_THRESHOLD * expected_gain:
            # Coverage stalled - reactive model flags this
            self.trust = max(0, self.trust - REACTIVE_DECAY_RATE)
            self.bad_windows += 1
            
            if self.first_flag_time is None:
                self.first_flag_time = now
            
            if self.trust < REACTIVE_IMPEACH and self.impeachment_time is None:
                self.impeachment_time = now
        else:
            # Coverage progressing - recover
            self.trust = min(1.0, self.trust + REACTIVE_RECOVERY)


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else datetime.now().strftime("%Y%m%d_%H%M%S")
    
    os.makedirs("logs", exist_ok=True)
    outfile = f"logs/reactive_{name}.jsonl"
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('', UDP_PEER_PORT))
    except OSError:
        # Port already in use (logger.py running), use different port
        sock.bind(('', UDP_PEER_PORT + 50))
        print(f"[WARN] Port {UDP_PEER_PORT} in use, using {UDP_PEER_PORT + 50}")
    sock.setblocking(False)
    
    print("=== Reactive Trust Model Logger (PASSIVE) ===")
    print(f"Logging to: {outfile}")
    print("This does NOT affect robots. Compares detection timing.\n")
    
    # One reactive model per robot
    models = defaultdict(lambda: ReactiveModel(0))
    last_check = time.time()
    start_time = time.time()
    msg_count = 0
    
    # Track proactive detection from robot logs
    proactive_first_detect = {}  # robot_id -> timestamp
    proactive_impeach = {}       # robot_id -> timestamp
    
    with open(outfile, 'w') as f:
        try:
            while True:
                # Read messages
                try:
                    data, addr = sock.recvfrom(4096)
                    msg = json.loads(data.decode())
                    msg_count += 1
                    
                    rid = msg.get('robot_id', 0)
                    models[rid].update(msg)
                    
                    # Track proactive detection from robot's own logs
                    first_decay = msg.get('first_decay_time')
                    if first_decay and rid not in proactive_first_detect:
                        proactive_first_detect[rid] = first_decay
                    
                    imp_time = msg.get('impeachment_time')
                    if imp_time and rid not in proactive_impeach:
                        proactive_impeach[rid] = imp_time
                    
                except BlockingIOError:
                    pass
                
                # Periodic reactive check
                if time.time() - last_check > REACTIVE_WINDOW:
                    for rid, model in models.items():
                        model.check_reactive_trust()
                        
                        record = {
                            't': time.time(),
                            'experiment_time': time.time() - start_time,
                            'robot_id': rid,
                            'reactive_trust': model.trust,
                            'reactive_first_flag': model.first_flag_time,
                            'reactive_impeach': model.impeachment_time,
                            'reactive_bad_windows': model.bad_windows,
                            'proactive_first_detect': proactive_first_detect.get(rid),
                            'proactive_impeach': proactive_impeach.get(rid),
                        }
                        f.write(json.dumps(record) + '\n')
                        f.flush()
                    
                    last_check = time.time()
                    
                    # Print comparison
                    elapsed = time.time() - start_time
                    print(f"\n[{elapsed:.1f}s] Reactive Trust Status:")
                    for rid in sorted(models.keys()):
                        m = models[rid]
                        pro_det = "N/A"
                        if rid in proactive_first_detect:
                            dt = proactive_first_detect[rid] - start_time
                            pro_det = f"{dt:.1f}s"
                        
                        react_det = "N/A"
                        if m.first_flag_time:
                            dt = m.first_flag_time - start_time
                            react_det = f"{dt:.1f}s"
                        
                        print(f"  R{rid}: reactive_trust={m.trust:.2f} "
                              f"reactive_detect={react_det} proactive_detect={pro_det}")
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            pass
    
    # Final summary
    print(f"\n{'='*60}")
    print("DETECTION COMPARISON SUMMARY")
    print(f"{'='*60}")
    for rid in sorted(models.keys()):
        m = models[rid]
        print(f"\nRobot {rid}:")
        print(f"  Proactive first detect: {proactive_first_detect.get(rid, 'none')}")
        print(f"  Reactive first flag:    {m.first_flag_time or 'none'}")
        if rid in proactive_first_detect and m.first_flag_time:
            delta = m.first_flag_time - proactive_first_detect[rid]
            print(f"  --> Reactive was {delta:.2f}s SLOWER than proactive")
        print(f"  Proactive impeachment:  {proactive_impeach.get(rid, 'none')}")
        print(f"  Reactive impeachment:   {m.impeachment_time or 'none'}")
    
    print(f"\nTotal messages processed: {msg_count}")
    print(f"Saved to: {outfile}")


if __name__ == "__main__":
    main()
