#!/bin/bash
# Deploy REIP robot code to Pi Zero
# Usage: ./deploy.sh clanker-1.local [robot_id]

set -e

HOST="${1:-clanker-1.local}"
ROBOT_ID="${2:-0}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "==================================="
echo "REIP Robot Deployment"
echo "==================================="
echo "Target: $HOST"
echo "Robot ID: $ROBOT_ID"
echo ""

# Create remote directory
echo "[1/4] Creating remote directory..."
ssh pi@$HOST "mkdir -p ~/reip"

# Copy files
echo "[2/4] Copying files..."
scp "$SCRIPT_DIR/reip_controller.py" pi@$HOST:~/reip/
scp "$SCRIPT_DIR/test_hardware_quick.py" pi@$HOST:~/reip/
scp "$SCRIPT_DIR/requirements.txt" pi@$HOST:~/reip/

# Install dependencies (if not already installed)
echo "[3/4] Installing dependencies..."
ssh pi@$HOST "pip3 install -r ~/reip/requirements.txt --break-system-packages 2>/dev/null || true"

# Create systemd service for auto-start
echo "[4/4] Setting up service..."
ssh pi@$HOST "cat > /tmp/reip.service << 'EOF'
[Unit]
Description=REIP Robot Controller
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/reip/reip_controller.py --robot-id $ROBOT_ID
WorkingDirectory=/home/pi/reip
User=pi
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo mv /tmp/reip.service /etc/systemd/system/
sudo systemctl daemon-reload"

echo ""
echo "==================================="
echo "Deployment complete!"
echo "==================================="
echo ""
echo "Commands:"
echo "  Test hardware:   ssh pi@$HOST 'python3 ~/reip/test_hardware_quick.py'"
echo "  Run manually:    ssh pi@$HOST 'python3 ~/reip/reip_controller.py --robot-id $ROBOT_ID'"
echo "  Start service:   ssh pi@$HOST 'sudo systemctl start reip'"
echo "  Enable autorun:  ssh pi@$HOST 'sudo systemctl enable reip'"
echo "  View logs:       ssh pi@$HOST 'journalctl -u reip -f'"
echo ""
