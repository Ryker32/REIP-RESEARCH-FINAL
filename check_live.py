import paramiko, time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.20.22', username='pi', password='clanker', timeout=5)

# Check process
stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node')
pid = stdout.read().decode().strip()
print(f"R4 pid: {pid}")

# First position read
stdin, stdout, stderr = ssh.exec_command('tail -1 /tmp/reip_4.log')
p1 = stdout.read().decode().strip()
print(f"Snap 1: {p1}")

# Wait 3 seconds on the Pi side
stdin, stdout, stderr = ssh.exec_command('sleep 3 && tail -1 /tmp/reip_4.log')
p2 = stdout.read().decode().strip()
print(f"Snap 2: {p2}")
if p1 == p2:
    print("SAME - frozen!")
else:
    print("DIFFERENT - position updating!")

# Check startup for errors
stdin, stdout, stderr = ssh.exec_command('head -25 /tmp/reip_4.log')
startup = stdout.read().decode().strip()
print(f"\nStartup:\n{startup}")

ssh.close()
