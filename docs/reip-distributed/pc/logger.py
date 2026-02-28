#!/usr/bin/env python3
"""
REIP Experiment Logger
Collects all robot broadcasts and saves to JSONL file for analysis.

Usage: python3 logger.py experiment_name
"""

import socket
import json
import time
import sys
from datetime import datetime

UDP_PEER_PORT = 5004

def main():
    if len(sys.argv) < 2:
        name = datetime.now().strftime("%Y%m%d_%H%M%S")
    else:
        name = sys.argv[1]
    
    filename = f"logs/{name}.jsonl"
    
    # Create logs directory
    import os
    os.makedirs("logs", exist_ok=True)
    
    # Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', UDP_PEER_PORT))
    sock.setblocking(False)
    
    print(f"=== REIP Experiment Logger ===")
    print(f"Logging to: {filename}")
    print("Press Ctrl+C to stop\n")
    
    start_time = time.time()
    msg_count = 0
    
    with open(filename, 'w') as f:
        try:
            while True:
                try:
                    data, addr = sock.recvfrom(4096)
                    msg = json.loads(data.decode())
                    
                    # Add metadata
                    msg['_log_time'] = time.time()
                    msg['_experiment_time'] = time.time() - start_time
                    msg['_source_ip'] = addr[0]
                    
                    f.write(json.dumps(msg) + '\n')
                    f.flush()
                    
                    msg_count += 1
                    
                except BlockingIOError:
                    pass
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            pass
    
    duration = time.time() - start_time
    print(f"\nLogged {msg_count} messages over {duration:.1f} seconds")
    print(f"Saved to: {filename}")

if __name__ == "__main__":
    main()
