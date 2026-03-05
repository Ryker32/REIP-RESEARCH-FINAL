#!/usr/bin/env python3
"""
Fully automated hardware trial runner.

Handles everything: deploy, launch, record video, inject faults on
schedule, stop robots, collect logs. Zero human intervention.

Usage:
  python run_trial.py                          # REIP + bad_leader (default)
  python run_trial.py --controller reip --fault bad_leader
  python run_trial.py --controller raft --fault bad_leader
  python run_trial.py --controller reip --fault none          # clean run
  python run_trial.py --controller decentralized --fault none
  python run_trial.py --batch                  # run full experiment matrix

Timing (matches isef_experiments.py):
  t=0s   : deploy + launch robots
  t=10s  : FAULT #1 on current leader
  t=30s  : FAULT #2 on current leader (may differ after impeachment)
  t=120s : stop robots, collect logs

Position server (aruco_position_server.py) must be running separately.
Press 'r' in its window to start/stop video recording, OR pass
--no-video to skip manual recording.
"""

import argparse
import json
import os
import socket
import sys
import time
from datetime import datetime

import paramiko

# ==================== Config (must match other scripts) ====================
HOSTS = {
    1: '192.168.20.16',
    2: '192.168.20.15',
    3: '192.168.20.112',
    4: '192.168.20.22',
    5: '192.168.20.18',
}
PASSWORD = 'clanker'
NUM_ROBOTS = 5

UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
BROADCAST_IP = "192.168.20.255"
WIFI_BIND_IP = "192.168.20.214"

EXPERIMENT_DURATION = 120
FAULT_INJECT_TIME_1 = 10
FAULT_INJECT_TIME_2 = 30

REIP_LOCAL = os.path.join(os.path.dirname(__file__), 'robot', 'reip_node.py')
RAFT_LOCAL = os.path.join(os.path.dirname(__file__), 'robot', 'baselines', 'raft_node.py')

# Full experiment matrix (matches isef_experiments.py)
EXPERIMENT_MATRIX = [
    # (controller, fault, description)
    ("reip",          "none",         "REIP clean baseline"),
    ("reip",          "bad_leader",   "REIP vs bad leader (KEY DEMO)"),
    ("raft",          "none",         "Raft clean baseline"),
    ("raft",          "bad_leader",   "Raft vs bad leader (no detection)"),
    ("decentralized", "none",         "Decentralized clean baseline"),
    ("decentralized", "bad_leader",   "Decentralized vs bad leader (immune)"),
]
TRIALS_PER_CONDITION = 3


# ==================== SSH helpers ====================
def _ssh_connect(host, timeout=4):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username='pi', password=PASSWORD, timeout=timeout)
    return ssh


def _kill_one(rid, host):
    try:
        ssh = _ssh_connect(host, timeout=3)
        ssh.exec_command('pkill -f reip_node; pkill -f raft_node')
        ssh.close()
    except Exception as e:
        print(f"  R{rid}: {e}")


def kill_all():
    print("[KILL] Stopping all robots...")
    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as ex:
        futs = {ex.submit(_kill_one, rid, host): rid for rid, host in HOSTS.items()}
        concurrent.futures.wait(futs, timeout=10)
    time.sleep(0.5)


def clear_robot_logs():
    """Remove old logs so collect_logs only gets this trial's data."""
    import concurrent.futures
    def _clear_one(rid, host):
        try:
            ssh = _ssh_connect(host, timeout=3)
            ssh.exec_command('rm -f /home/pi/reip/logs/*.jsonl')
            ssh.close()
        except Exception:
            pass
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as ex:
        futs = [ex.submit(_clear_one, rid, host) for rid, host in HOSTS.items()]
        concurrent.futures.wait(futs, timeout=10)


def deploy_and_launch(controller):
    """Two-phase deploy: upload code to ALL robots, then launch ALL simultaneously."""
    if controller == 'raft':
        local_file = RAFT_LOCAL
        remote_script = 'baselines/raft_node.py'
    else:
        local_file = REIP_LOCAL
        remote_script = 'reip_node.py'

    decentralized_flag = ' --decentralized' if controller == 'decentralized' else ''

    # --- Phase 1: Kill old processes + upload code (sequential, no launch yet) ---
    print(f"[DEPLOY] Phase 1: uploading {remote_script} to all robots...")
    ssh_connections = {}
    for rid, host in sorted(HOSTS.items()):
        try:
            ssh = _ssh_connect(host)
            ssh.exec_command('pkill -f reip_node; pkill -f raft_node')
            sftp = ssh.open_sftp()
            for d in ['/home/pi/reip', '/home/pi/reip/logs', '/home/pi/reip/baselines']:
                try:
                    sftp.mkdir(d)
                except:
                    pass
            sftp.put(local_file, f'/home/pi/reip/{remote_script}')
            sftp.close()
            ssh_connections[rid] = ssh
            print(f"  R{rid}: uploaded")
        except Exception as e:
            print(f"  R{rid}: ERROR - {e}")

    time.sleep(1)

    # --- Phase 2: Launch ALL robots at once (simultaneous start) ---
    print(f"[DEPLOY] Phase 2: launching all robots simultaneously...")
    for rid, ssh in ssh_connections.items():
        try:
            cmd = (f'cd /home/pi/reip && nohup python3 {remote_script} {rid}'
                   f'{decentralized_flag}'
                   f' > /tmp/reip_{rid}.log 2>&1 &')
            ssh.exec_command(cmd)
        except Exception as e:
            print(f"  R{rid}: launch error ({e}), reconnecting...")
            try:
                ssh.close()
                ssh2 = _ssh_connect(HOSTS[rid])
                ssh2.exec_command(cmd)
                ssh_connections[rid] = ssh2
                print(f"  R{rid}: relaunched OK")
            except Exception as e2:
                print(f"  R{rid}: relaunch FAILED - {e2}")

    # Brief pause then verify all running
    time.sleep(2)
    for rid, ssh in ssh_connections.items():
        try:
            _, stdout, _ = ssh.exec_command(f'pgrep -f "{remote_script}"')
            pid = stdout.read().decode().strip()
            status = f"RUNNING (pid={pid})" if pid else "FAILED"
            print(f"  R{rid}: {status}")
            ssh.close()
        except Exception as e:
            print(f"  R{rid}: verify error - {e}")


def collect_logs(trial_dir):
    os.makedirs(trial_dir, exist_ok=True)
    print(f"[LOGS] Collecting to {trial_dir}")
    for rid, host in HOSTS.items():
        try:
            ssh = _ssh_connect(host)
            sftp = ssh.open_sftp()
            try:
                for f in sftp.listdir('/home/pi/reip/logs'):
                    if f.endswith('.jsonl'):
                        sftp.get(f'/home/pi/reip/logs/{f}',
                                 os.path.join(trial_dir, f'r{rid}_{f}'))
                        print(f"  R{rid}: {f}")
            except FileNotFoundError:
                print(f"  R{rid}: no logs")
            sftp.close()
            ssh.close()
        except Exception as e:
            print(f"  R{rid}: {e}")


# ==================== Fault injection ====================
def find_leader(timeout=4.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((WIFI_BIND_IP, UDP_PEER_PORT))
    sock.settimeout(0.2)

    votes = {}
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(65536)
            msg = json.loads(data.decode())
            if msg.get('type') == 'peer_state':
                lid = msg.get('leader_id')
                if lid:
                    votes[lid] = votes.get(lid, 0) + 1
        except socket.timeout:
            pass
    sock.close()
    return max(votes, key=votes.get) if votes else -1


def inject_fault(robot_id, fault_type):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((WIFI_BIND_IP, 0))
    msg = json.dumps({
        'type': 'fault_inject',
        'robot_id': robot_id,
        'fault': fault_type,
        'timestamp': time.time()
    }).encode()
    sock.sendto(msg, (BROADCAST_IP, UDP_FAULT_PORT))
    sock.close()


def clear_faults(robot_ids):
    for rid in robot_ids:
        inject_fault(rid, 'none')


# ==================== Single trial ====================
def run_single_trial(controller, fault_type, trial_num, output_dir):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    trial_name = f"{controller}_{fault_type or 'none'}_t{trial_num}"
    trial_dir = os.path.join(output_dir, f"{trial_name}_{ts}")
    os.makedirs(trial_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f" TRIAL: {trial_name}")
    print(f" Controller: {controller}")
    print(f" Fault: {fault_type or 'none'}")
    print(f" Duration: {EXPERIMENT_DURATION}s")
    if fault_type and fault_type != 'none':
        print(f" Fault #1 at t={FAULT_INJECT_TIME_1}s")
        print(f" Fault #2 at t={FAULT_INJECT_TIME_2}s")
    print(f" Output: {trial_dir}")
    print(f"{'='*60}\n")

    # Save trial metadata
    meta = {
        'trial_name': trial_name, 'controller': controller,
        'fault_type': fault_type, 'trial_num': trial_num,
        'duration': EXPERIMENT_DURATION,
        'fault_time_1': FAULT_INJECT_TIME_1 if fault_type and fault_type != 'none' else None,
        'fault_time_2': FAULT_INJECT_TIME_2 if fault_type and fault_type != 'none' else None,
        'start_time': time.time(), 'start_ts': ts,
    }

    # Phase 0: Kill old processes + clear old logs
    kill_all()
    clear_robot_logs()

    # Phase 1: Deploy and launch
    deploy_and_launch(controller)

    # Give robots a few seconds to get localized and elect a leader,
    # then send the start signal so they all begin simultaneously.
    print(f"\n[WAIT] Giving robots 10s for boot + localization + leader election...")
    time.sleep(10)

    # Reset coverage on the position server overlay
    _reset_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _reset_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    _reset_sock.sendto(json.dumps({'type': 'reset_coverage'}).encode(),
                       (BROADCAST_IP, UDP_PEER_PORT))
    _reset_sock.close()

    print(f"[START] Sending start signal to all robots...")
    for _ in range(5):
        inject_fault(0, 'start')
        time.sleep(0.2)

    t0 = time.time()
    fault_robots = []

    print(f"\n[t=0.0s] Trial clock started.")
    print(f"         Position server should be running + recording.\n")

    try:
        while True:
            elapsed = time.time() - t0

            if elapsed >= EXPERIMENT_DURATION:
                break

            # Fault #1
            if (fault_type and fault_type != 'none'
                    and elapsed >= FAULT_INJECT_TIME_1
                    and len(fault_robots) == 0):
                print(f"[t={elapsed:.1f}s] Scanning for leader...")
                leader = find_leader(timeout=3.0)
                if leader > 0:
                    inject_fault(leader, fault_type)
                    fault_robots.append(leader)
                    meta['fault1_robot'] = leader
                    meta['fault1_actual_time'] = elapsed
                    print(f"[t={time.time()-t0:.1f}s] FAULT #1: {fault_type} on Robot {leader}")
                else:
                    print(f"[t={elapsed:.1f}s] WARNING: no leader found for fault #1")
                    fault_robots.append(None)

            # Fault #2
            if (fault_type and fault_type != 'none'
                    and elapsed >= FAULT_INJECT_TIME_2
                    and len(fault_robots) == 1):
                print(f"\n[t={elapsed:.1f}s] Scanning for leader (fault #2)...")
                leader2 = find_leader(timeout=3.0)
                if leader2 > 0:
                    if leader2 != fault_robots[0]:
                        print(f"  New leader after impeachment: Robot {leader2}")
                    else:
                        print(f"  Same leader: Robot {leader2} (re-injecting)")
                    inject_fault(leader2, fault_type)
                    fault_robots.append(leader2)
                    meta['fault2_robot'] = leader2
                    meta['fault2_actual_time'] = elapsed
                    print(f"[t={time.time()-t0:.1f}s] FAULT #2: {fault_type} on Robot {leader2}")
                else:
                    print(f"[t={elapsed:.1f}s] WARNING: no leader found for fault #2")
                    fault_robots.append(None)

            # Progress report every 15s
            if int(elapsed) % 15 == 0 and int(elapsed) > 0:
                remaining = EXPERIMENT_DURATION - elapsed
                if abs(elapsed - int(elapsed)) < 0.5:
                    print(f"[t={elapsed:.0f}s] {remaining:.0f}s remaining...")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n[!] Trial interrupted by user.")

    # Phase 3: Stop
    elapsed_final = time.time() - t0
    print(f"\n[t={elapsed_final:.1f}s] Trial complete.")
    meta['actual_duration'] = elapsed_final

    clear_faults([r for r in fault_robots if r])
    time.sleep(0.5)
    kill_all()

    # Phase 4: Collect logs
    time.sleep(2)
    collect_logs(trial_dir)

    # Save metadata
    meta_path = os.path.join(trial_dir, 'trial_meta.json')
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"\n[DONE] Trial data in {trial_dir}\n")
    return trial_dir


# ==================== Batch mode ====================
def run_batch(output_dir, trials_per=TRIALS_PER_CONDITION):
    total = len(EXPERIMENT_MATRIX) * trials_per
    print(f"\n{'#'*60}")
    print(f"  BATCH MODE: {len(EXPERIMENT_MATRIX)} conditions x {trials_per} trials = {total} runs")
    print(f"  Estimated time: {total * (EXPERIMENT_DURATION + 30) / 60:.0f} minutes")
    print(f"{'#'*60}\n")

    input("Place robots in arena and press ENTER to begin...")

    completed = 0
    for trial_num in range(1, trials_per + 1):
        for controller, fault, desc in EXPERIMENT_MATRIX:
            completed += 1
            print(f"\n{'*'*60}")
            print(f"  [{completed}/{total}] {desc}  (trial {trial_num})")
            print(f"{'*'*60}")

            run_single_trial(controller, fault, trial_num, output_dir)

            if completed < total:
                print("\n[PAUSE] Reposition robots in arena.")
                input("Press ENTER when ready for next trial...")

    print(f"\n{'#'*60}")
    print(f"  ALL {total} TRIALS COMPLETE")
    print(f"  Data in {output_dir}")
    print(f"{'#'*60}")


# ==================== Entry ====================
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Automated hardware trial runner")
    parser.add_argument('--controller', default='reip',
                        choices=['reip', 'raft', 'decentralized'])
    parser.add_argument('--fault', default='bad_leader',
                        help='Fault type: bad_leader, freeze_leader, spin, none')
    parser.add_argument('--trial', type=int, default=1, help='Trial number')
    parser.add_argument('--output', default='trials', help='Output directory')
    parser.add_argument('--batch', action='store_true',
                        help='Run full experiment matrix')
    parser.add_argument('--trials-per', type=int, default=TRIALS_PER_CONDITION,
                        help='Trials per condition in batch mode')
    parser.add_argument('--duration', type=int, default=EXPERIMENT_DURATION,
                        help='Trial duration in seconds')
    args = parser.parse_args()

    EXPERIMENT_DURATION = args.duration

    if args.batch:
        run_batch(args.output, args.trials_per)
    else:
        run_single_trial(args.controller, args.fault, args.trial, args.output)
