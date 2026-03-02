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

def deploy_and_launch(rid, host):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, username='pi', password=PASSWORD, timeout=8)

        # Kill any existing process
        ssh.exec_command('pkill -f reip_node; sleep 0.5')
        time.sleep(1)

        # Deploy file
        sftp = ssh.open_sftp()
        try:
            sftp.mkdir('/home/pi/reip')
        except:
            pass
        try:
            sftp.mkdir('/home/pi/reip/logs')
        except:
            pass
        sftp.put(LOCAL_FILE, '/home/pi/reip/reip_node.py')
        sftp.close()
        print(f"  R{rid}: deployed")

        # Launch with nohup so it survives SSH disconnect
        cmd = (f'nohup python3 /home/pi/reip/reip_node.py {rid} '
               f'> /tmp/reip_{rid}.log 2>&1 &')
        ssh.exec_command(cmd)
        time.sleep(1)

        # Verify it started
        stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node')
        pid = stdout.read().decode().strip()
        if pid:
            print(f"  R{rid}: RUNNING (pid={pid})")
        else:
            print(f"  R{rid}: FAILED TO START")

        ssh.close()
        return True
    except Exception as e:
        print(f"  R{rid}: ERROR - {e}")
        return False

if __name__ == '__main__':
    print("Deploying and launching all robots...")
    print()
    for rid in sorted(HOSTS.keys()):
        deploy_and_launch(rid, HOSTS[rid])
    print()
    print("Done. All robots should be running.")
    print("Make sure the position server (aruco_position_server.py) is running!")
