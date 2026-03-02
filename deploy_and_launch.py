"""Deploy reip_node.py to all robots and launch with nohup."""
import paramiko
import time
import os

HOSTS = {
    1: '192.168.20.16',
    2: '192.168.20.15',
    3: '192.168.20.112',
    4: '192.168.20.22',
    5: '192.168.20.18',
}
PASSWORD = 'clanker'
LOCAL_FILE = os.path.join(os.path.dirname(__file__), 'robot', 'reip_node.py')

def deploy_all(local_file=None, remote_script='reip_node.py'):
    """Upload code to all robots (no launch yet). Returns open SSH connections."""
    if local_file is None:
        local_file = LOCAL_FILE
    connections = {}
    for rid, host in sorted(HOSTS.items()):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, username='pi', password=PASSWORD, timeout=8)
            ssh.exec_command('pkill -f reip_node; pkill -f raft_node')
            sftp = ssh.open_sftp()
            for d in ['/home/pi/reip', '/home/pi/reip/logs', '/home/pi/reip/baselines']:
                try:
                    sftp.mkdir(d)
                except:
                    pass
            sftp.put(local_file, f'/home/pi/reip/{remote_script}')
            sftp.close()
            connections[rid] = ssh
            print(f"  R{rid}: uploaded {remote_script}")
        except Exception as e:
            print(f"  R{rid}: ERROR - {e}")
    return connections


def launch_all(connections, remote_script='reip_node.py'):
    """Launch on all robots simultaneously using pre-opened SSH connections."""
    print("  Launching all simultaneously...")
    for rid, ssh in connections.items():
        try:
            cmd = (f'nohup python3 /home/pi/reip/{remote_script} {rid} '
                   f'> /tmp/reip_{rid}.log 2>&1 &')
            ssh.exec_command(cmd)
        except Exception as e:
            print(f"  R{rid}: launch error - {e}")
    time.sleep(2)
    for rid, ssh in connections.items():
        try:
            _, stdout, _ = ssh.exec_command(f'pgrep -f "{remote_script}"')
            pid = stdout.read().decode().strip()
            print(f"  R{rid}: {'RUNNING (pid=' + pid + ')' if pid else 'FAILED'}")
            ssh.close()
        except Exception as e:
            print(f"  R{rid}: {e}")

def kill_all():
    """Kill reip_node / raft_node on all robots."""
    for rid, host in HOSTS.items():
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, username='pi', password=PASSWORD, timeout=8)
            ssh.exec_command('pkill -f reip_node; pkill -f raft_node')
            ssh.close()
            print(f"  R{rid}: killed")
        except Exception as e:
            print(f"  R{rid}: {e}")


def collect_logs(trial_dir):
    """SCP logs from all robots into trial_dir."""
    os.makedirs(trial_dir, exist_ok=True)
    for rid, host in HOSTS.items():
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, username='pi', password=PASSWORD, timeout=8)
            sftp = ssh.open_sftp()
            try:
                for f in sftp.listdir('/home/pi/reip/logs'):
                    if f.endswith('.jsonl'):
                        remote = f'/home/pi/reip/logs/{f}'
                        local = os.path.join(trial_dir, f'r{rid}_{f}')
                        sftp.get(remote, local)
                        print(f"  R{rid}: {f}")
            except FileNotFoundError:
                print(f"  R{rid}: no logs")
            sftp.close()
            ssh.close()
        except Exception as e:
            print(f"  R{rid}: {e}")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--kill', action='store_true', help='Kill all robots')
    parser.add_argument('--collect', type=str, help='Collect logs to directory')
    parser.add_argument('--raft', action='store_true', help='Deploy raft_node instead')
    args = parser.parse_args()

    if args.kill:
        print("Killing all robots...")
        kill_all()
    elif args.collect:
        print(f"Collecting logs to {args.collect}...")
        collect_logs(args.collect)
    else:
        if args.raft:
            remote_script = 'baselines/raft_node.py'
            local = os.path.join(os.path.dirname(__file__), 'robot', 'baselines', 'raft_node.py')
        else:
            remote_script = 'reip_node.py'
            local = LOCAL_FILE
        print(f"Deploying {remote_script} to all robots...")
        print()
        conns = deploy_all(local_file=local, remote_script=remote_script)
        time.sleep(1)
        launch_all(conns, remote_script=remote_script)
        print()
        print("Done. All robots started simultaneously.")
        print("Make sure the position server (aruco_position_server.py) is running!")
