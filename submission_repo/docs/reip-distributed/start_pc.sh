#!/bin/bash
# Start all REIP PC components in separate terminals
# Usage: ./start_pc.sh

echo "Starting REIP Validation System..."

# Check for required packages
python3 -c "import cv2" 2>/dev/null || { echo "Error: opencv-python not installed"; exit 1; }

# Start position server
echo "Starting Position Server..."
gnome-terminal --title="Position Server" -- python3 pc/aruco_position_server.py &
sleep 2

# Start visualizer
echo "Starting Visualizer..."
gnome-terminal --title="Visualizer" -- python3 pc/visualizer.py &
sleep 1

# Start logger
LOGNAME="experiment_$(date +%Y%m%d_%H%M%S)"
echo "Starting Logger ($LOGNAME)..."
gnome-terminal --title="Logger" -- python3 pc/logger.py $LOGNAME &
sleep 1

# Start fault injector
echo "Starting Fault Injector..."
gnome-terminal --title="Fault Injector" -- python3 pc/fault_inject.py &

echo ""
echo "All components started!"
echo "Now SSH to each robot and run: python3 reip_node.py <robot_id>"
echo ""
echo "Robot commands:"
echo "  ssh pi@clanker-1.local 'python3 reip_node.py 1'"
echo "  ssh pi@clanker-2.local 'python3 reip_node.py 2'"
echo "  ..."
