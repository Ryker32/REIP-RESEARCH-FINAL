#!/usr/bin/env python3
"""
REIP Fault Injector
Send fault commands to robots for testing resilience.

Usage: python3 fault_inject.py [--sim]

  --sim  Use unique ports for localhost testing (robot i on 5300+i)
         Without --sim, broadcasts to port 5300 (real hardware mode)

MOTOR FAULTS (caught by anomaly detection):
  spin <id>      - Make robot spin in circles
  stop <id>      - Make robot stop moving
  erratic <id>   - Make robot move erratically  

LEADERSHIP FAULT (caught by three-tier trust model):
  bad_leader <id> - Leader sends robots to already-explored cells
                    THIS IS THE KEY DEMO for WSSEF!

CONTROL:
  clear <id>     - Clear all faults, return to normal
  status         - Show current faults
  quit           - Exit
"""

import socket
import json
import time
import sys

# MUST MATCH robot/reip_node.py
UDP_FAULT_PORT = 5300
BROADCAST_IP = "255.255.255.255"
NUM_ROBOTS = 5

class FaultInjector:
    def __init__(self, sim_mode: bool = False):
        self.sim_mode = sim_mode
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.active_faults = {}
        mode_str = "SIM (unique ports)" if sim_mode else "HARDWARE (broadcast)"
        print(f"[{mode_str}]")
    
    def inject(self, robot_id: int, fault_type: str):
        """Send fault injection command"""
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        data = json.dumps(msg).encode()
        
        if self.sim_mode:
            # Sim: send to each robot's unique port on localhost
            if robot_id == 'all' or robot_id == 0:
                for rid in range(1, NUM_ROBOTS + 1):
                    self.socket.sendto(data, ('127.0.0.1', UDP_FAULT_PORT + rid))
            else:
                self.socket.sendto(data, ('127.0.0.1', UDP_FAULT_PORT + int(robot_id)))
        else:
            # Hardware: broadcast to shared port, robots filter by ID
            self.socket.sendto(data, (BROADCAST_IP, UDP_FAULT_PORT))
        
        if fault_type == 'none':
            if robot_id in self.active_faults:
                del self.active_faults[robot_id]
            print(f"Cleared fault on Robot {robot_id}")
        else:
            self.active_faults[robot_id] = fault_type
            print(f"Injected '{fault_type}' fault on Robot {robot_id}")
    
    def run(self):
        print("=== REIP Fault Injector ===")
        print("Motor faults:     spin <id>, stop <id>, erratic <id>")
        print("Leadership fault: bad_leader <id>  <-- KEY DEMO")
        print("Control:          clear <id>, status, quit\n")
        
        while True:
            try:
                cmd = input("> ").strip().lower()
                parts = cmd.split()
                
                if not parts:
                    continue
                
                if parts[0] == 'quit':
                    break
                    
                elif parts[0] == 'status':
                    if self.active_faults:
                        print("Active faults:")
                        for rid, fault in self.active_faults.items():
                            print(f"  Robot {rid}: {fault}")
                    else:
                        print("No active faults")
                
                elif parts[0] in ['spin', 'stop', 'erratic', 'bad_leader'] and len(parts) >= 2:
                    rid = int(parts[1])
                    self.inject(rid, parts[0])
                    if parts[0] == 'bad_leader':
                        print("  → Leader will now send robots to explored cells")
                        print("  → Watch for trust decay in followers!")
                
                elif parts[0] == 'clear' and len(parts) >= 2:
                    rid = int(parts[1])
                    self.inject(rid, 'none')
                
                else:
                    print("Unknown command. Try: bad_leader 1, spin 2, clear 1, status, quit")
                    
            except ValueError:
                print("Invalid robot ID")
            except KeyboardInterrupt:
                break
        
        # Clear all faults on exit
        for rid in list(self.active_faults.keys()):
            self.inject(rid, 'none')
        
        print("\nExiting.")

if __name__ == "__main__":
    sim_mode = "--sim" in sys.argv
    fi = FaultInjector(sim_mode=sim_mode)
    fi.run()
