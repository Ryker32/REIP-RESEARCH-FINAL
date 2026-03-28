#!/usr/bin/env python3
"""
REIP Fault Injector
Send fault commands to robots for testing resilience.

Usage: python3 fault_inject.py

Commands:
  spin <id>      - Make robot spin in circles
  stop <id>      - Make robot stop moving
  erratic <id>   - Make robot move erratically  
  clear <id>     - Clear fault, return to normal
  status         - Show current faults
  quit           - Exit
"""

import socket
import json
import time

UDP_FAULT_PORT = 5005
BROADCAST_IP = "255.255.255.255"

class FaultInjector:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.active_faults = {}
    
    def inject(self, robot_id: int, fault_type: str):
        """Send fault injection command"""
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        data = json.dumps(msg).encode()
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
        print("Commands: spin <id>, stop <id>, erratic <id>, clear <id>, status, quit\n")
        
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
                
                elif parts[0] in ['spin', 'stop', 'erratic'] and len(parts) >= 2:
                    rid = int(parts[1])
                    self.inject(rid, parts[0])
                
                elif parts[0] == 'clear' and len(parts) >= 2:
                    rid = int(parts[1])
                    self.inject(rid, 'none')
                
                else:
                    print("Unknown command. Try: spin 1, stop 2, clear 1, status, quit")
                    
            except ValueError:
                print("Invalid robot ID")
            except KeyboardInterrupt:
                break
        
        # Clear all faults on exit
        for rid in list(self.active_faults.keys()):
            self.inject(rid, 'none')
        
        print("\nExiting.")

if __name__ == "__main__":
    fi = FaultInjector()
    fi.run()
