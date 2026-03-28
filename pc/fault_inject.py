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
  self_injure_leader <id> - Leader commands followers into an occupied peer zone
                            Critical coordination / collision-inducing attack

CONTROL:
  clear <id>     - Clear all faults, return to normal
  status         - Show current faults
  quit           - Exit
"""

import socket
import json
import time
import sys
import threading

# MUST MATCH robot/reip_node.py
UDP_FAULT_PORT = 5300
UDP_PEER_PORT = 5200
BROADCAST_IP = "192.168.20.255"
def _detect_wifi_ip() -> str:
    """Auto-detect the LAN IP on the robot subnet, fall back to 0.0.0.0."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((BROADCAST_IP, 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"

WIFI_BIND_IP = _detect_wifi_ip()
NUM_ROBOTS = 5

class FaultInjector:
    def __init__(self, sim_mode: bool = False):
        self.sim_mode = sim_mode
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if not sim_mode:
            self.socket.bind((WIFI_BIND_IP, 0))
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
    
    def send_start(self):
        """Broadcast 'start' to all robots so they begin moving."""
        msg = {'type': 'fault_inject', 'robot_id': 0, 'fault': 'start',
               'timestamp': time.time()}
        data = json.dumps(msg).encode()
        if self.sim_mode:
            for rid in range(1, NUM_ROBOTS + 1):
                self.socket.sendto(data, ('127.0.0.1', UDP_FAULT_PORT + rid))
        else:
            for _ in range(3):
                self.socket.sendto(data, (BROADCAST_IP, UDP_FAULT_PORT))
                time.sleep(0.05)
        print("START sent to all robots -- motors engaged")

    def run(self):
        print("=== REIP Fault Injector ===")
        print("Trial control:    start              <-- START THE TRIAL")
        print("Motor faults:     spin <id>, stop <id>, erratic <id>")
        print("Leadership faults: bad_leader <id>, self_injure_leader <id>")
        print("Control:          clear <id>, status, quit\n")
        
        while True:
            try:
                cmd = input("> ").strip().lower()
                parts = cmd.split()
                
                if not parts:
                    continue
                
                if parts[0] == 'quit':
                    break

                elif parts[0] == 'start':
                    self.send_start()

                elif parts[0] == 'status':
                    if self.active_faults:
                        print("Active faults:")
                        for rid, fault in self.active_faults.items():
                            print(f"  Robot {rid}: {fault}")
                    else:
                        print("No active faults")
                
                elif parts[0] in ['spin', 'stop', 'erratic', 'bad_leader', 'self_injure_leader'] and len(parts) >= 2:
                    rid = int(parts[1])
                    self.inject(rid, parts[0])
                    if parts[0] == 'bad_leader':
                        print("  -> Leader will now send robots to explored cells")
                        print("  -> Watch for trust decay in followers!")
                    elif parts[0] == 'self_injure_leader':
                        print("  -> Leader will now attract followers into an occupied peer zone")
                        print("  -> Watch whether REIP followers reject the unsafe command!")
                
                elif parts[0] == 'clear' and len(parts) >= 2:
                    rid = int(parts[1])
                    self.inject(rid, 'none')
                
                else:
                    print("Unknown command. Try: bad_leader 1, self_injure_leader 1, spin 2, clear 1, status, quit")
                    
            except ValueError:
                print("Invalid robot ID")
            except KeyboardInterrupt:
                break
        
        # Clear all faults on exit
        for rid in list(self.active_faults.keys()):
            self.inject(rid, 'none')
        
        print("\nExiting.")

    def find_leader(self, timeout=5.0) -> int:
        """Eavesdrop on peer broadcasts to find the current leader."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        sock.bind(("0.0.0.0", UDP_PEER_PORT))
        sock.settimeout(0.2)

        leader_votes: dict[int, int] = {}
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data, _ = sock.recvfrom(65536)
                msg = json.loads(data.decode())
                if msg.get('type') == 'peer_state':
                    lid = msg.get('leader_id')
                    if lid:
                        leader_votes[lid] = leader_votes.get(lid, 0) + 1
            except socket.timeout:
                pass
            except (json.JSONDecodeError, UnicodeDecodeError, KeyError):
                pass
        sock.close()

        if not leader_votes:
            return -1
        return max(leader_votes, key=leader_votes.get)

    def run_auto(self, fault_type: str, delay: float, delay2: float, duration: float):
        """Automatic mode matching isef_experiments.py schedule:
          t=delay:  inject fault on current leader
          t=delay2: inject fault on current leader (may be new after impeachment)
          t=duration: end trial, clear faults
        """
        print(f"=== AUTO MODE ===")
        print(f"  Fault:      {fault_type}")
        print(f"  1st inject: t={delay}s")
        print(f"  2nd inject: t={delay2}s")
        print(f"  Duration:   {duration}s\n")

        self.send_start()
        t0 = time.time()

        # ---- First fault injection ----
        wait1 = delay - (time.time() - t0)
        if wait1 > 0:
            print(f"Waiting {wait1:.0f}s before first injection...")
            time.sleep(wait1)

        print("\n--- FAULT #1 ---")
        leader1 = self.find_leader(timeout=3.0)
        if leader1 < 0:
            print("ERROR: Could not find leader. Aborting.")
            return
        print(f"Current leader: Robot {leader1}")
        self.inject(leader1, fault_type)
        print(f"  [t={time.time()-t0:.1f}s] Injected {fault_type} on Robot {leader1}")

        # ---- Second fault injection ----
        wait2 = delay2 - (time.time() - t0)
        if wait2 > 0:
            print(f"\nWaiting {wait2:.0f}s before second injection...")
            time.sleep(wait2)

        print("\n--- FAULT #2 ---")
        leader2 = self.find_leader(timeout=3.0)
        if leader2 < 0:
            print("WARNING: Could not find leader for second injection.")
        else:
            if leader2 != leader1:
                print(f"New leader after impeachment: Robot {leader2}")
            else:
                print(f"Same leader still: Robot {leader2} (re-injecting)")
            self.inject(leader2, fault_type)
            print(f"  [t={time.time()-t0:.1f}s] Injected {fault_type} on Robot {leader2}")

        # ---- Wait for trial end ----
        remaining = duration - (time.time() - t0)
        if remaining > 0:
            print(f"\nRunning for {remaining:.0f}s more (total {duration:.0f}s)...")
            time.sleep(remaining)

        print(f"\n[t={time.time()-t0:.1f}s] Trial complete. Clearing faults...")
        for rid in list(self.active_faults.keys()):
            self.inject(rid, 'none')
        print("Done.")


if __name__ == "__main__":
    sim_mode = "--sim" in sys.argv

    # --auto <fault_type> --delay <s> --delay2 <s> --duration <s>
    # Defaults match isef_experiments.py: inject at 10s and 30s, run 120s
    if "--auto" in sys.argv:
        idx = sys.argv.index("--auto")
        fault_type = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else "bad_leader"
        delay = 10.0
        delay2 = 30.0
        duration = 120.0
        if "--delay" in sys.argv:
            delay = float(sys.argv[sys.argv.index("--delay") + 1])
        if "--delay2" in sys.argv:
            delay2 = float(sys.argv[sys.argv.index("--delay2") + 1])
        if "--duration" in sys.argv:
            duration = float(sys.argv[sys.argv.index("--duration") + 1])
        fi = FaultInjector(sim_mode=sim_mode)
        fi.run_auto(fault_type, delay, delay2, duration)
    else:
        fi = FaultInjector(sim_mode=sim_mode)
        fi.run()
