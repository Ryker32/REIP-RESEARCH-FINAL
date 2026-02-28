#!/usr/bin/env python3
"""Quick headless test: multiroom layout, 5 robots, verify REIP works."""
import socket, json, time, subprocess, sys, os, math

UDP_POSITION_PORT = 5100
UDP_PEER_PORT     = 5200
UDP_FAULT_PORT    = 5300
NUM_ROBOTS  = 5
ARENA_WIDTH = 2000
ARENA_HEIGHT= 1500
CELL_SIZE   = 125
WALLS = [(1000, 0, 1000, 1200)]  # multiroom: passage at top

# ---------- wall collision ----------
def _lc(x1,y1,x2,y2,cx,cy,r):
    dx,dy=x2-x1,y2-y1; fx,fy=x1-cx,y1-cy
    a=dx*dx+dy*dy
    if a<1e-6: return math.hypot(cx-x1,cy-y1)<r
    b=2*(fx*dx+fy*dy); c=fx*fx+fy*fy-r*r; disc=b*b-4*a*c
    if disc<=0: return False  # tangent = no collision (wall-follow fix)
    disc=math.sqrt(disc); t1=(-b-disc)/(2*a); t2=(-b+disc)/(2*a)
    return (0<=t1<=1)or(0<=t2<=1)or(t1<0 and t2>1)
def collides(x,y,r=75):
    return any(_lc(*w,x,y,r) for w in WALLS)

# ---------- start robots ----------
procs = []
os.makedirs("logs", exist_ok=True)
for i in range(1, NUM_ROBOTS+1):
    lf = open(f"logs/robot_{i}_console.log", "w")
    p = subprocess.Popen(
        [sys.executable, "-u", "robot/reip_node.py", str(i), "--sim"],
        stdout=lf, stderr=subprocess.STDOUT,
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    procs.append(p)
print(f"Started {NUM_ROBOTS} robots")
time.sleep(2)

# ---------- sockets ----------
pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
state_sock.bind(("", UDP_PEER_PORT))
state_sock.setblocking(False)
fault_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# robot positions  (cluster in Room A)
positions = {}
for i in range(1, NUM_ROBOTS+1):
    col, row = (i-1)%2, (i-1)//2
    positions[i] = [200+col*250, 200+row*300, 0.0]

visited = set()
nav_targets = {}
SPEED = 25
stuck_counters = {i: 0 for i in range(1, NUM_ROBOTS+1)}
prev_pos = {i: [200+(i-1)%2*250, 200+(i-1)//2*300] for i in range(1, NUM_ROBOTS+1)}

def _ccw(ax,ay,bx,by,cx,cy):
    return (bx-ax)*(cy-ay)-(by-ay)*(cx-ax)
def path_crosses_wall(x1,y1,x2,y2):
    for wx1,wy1,wx2,wy2 in WALLS:
        d1=_ccw(x1,y1,x2,y2,wx1,wy1); d2=_ccw(x1,y1,x2,y2,wx2,wy2)
        d3=_ccw(wx1,wy1,wx2,wy2,x1,y1); d4=_ccw(wx1,wy1,wx2,wy2,x2,y2)
        if ((d1>0 and d2<0) or (d1<0 and d2>0)) and ((d3>0 and d4<0) or (d3<0 and d4>0)):
            return True
    return False
def find_reachable(x,y):
    best,bd = None,float('inf')
    for cx in range(ARENA_WIDTH//CELL_SIZE):
        for cy in range(ARENA_HEIGHT//CELL_SIZE):
            if (cx,cy) in visited: continue
            tx,ty = (cx+0.5)*CELL_SIZE,(cy+0.5)*CELL_SIZE
            if collides(tx,ty): continue
            d = math.sqrt((tx-x)**2+(ty-y)**2)
            if d<bd and not path_crosses_wall(x,y,tx,ty):
                bd=d; best=(tx,ty)
    return best

def send_positions():
    for rid,(x,y,th) in positions.items():
        msg = {"type":"position","robot_id":rid,"x":x,"y":y,"theta":th,"timestamp":time.time()}
        pos_sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", UDP_POSITION_PORT+rid))

def recv_states():
    states = {}
    try:
        while True:
            data,_=state_sock.recvfrom(8192)
            msg=json.loads(data.decode())
            if msg.get("type")=="peer_state":
                rid=msg.get("robot_id")
                if rid:
                    states[rid]=msg
                    for c in msg.get("visited_cells",[]):
                        visited.add(tuple(c))
                    nav=msg.get("navigation_target")
                    if nav: nav_targets[rid]=nav
    except: pass
    return states

def update_pos():
    for rid in range(1, NUM_ROBOTS+1):
        x,y,th = positions[rid]
        # Stuck detection
        px,py = prev_pos.get(rid,[x,y])
        if math.sqrt((x-px)**2+(y-py)**2) < 15:
            stuck_counters[rid] = stuck_counters.get(rid,0)+1
        else:
            stuck_counters[rid] = 0
            prev_pos[rid] = [x,y]
        tgt = nav_targets.get(rid)
        # Override if stuck
        if stuck_counters.get(rid,0) >= 40:
            alt = find_reachable(x,y)
            if alt: tgt = alt
            stuck_counters[rid]=0; prev_pos[rid]=[x,y]
        if tgt:
            dx,dy = tgt[0]-x, tgt[1]-y
            dist = math.sqrt(dx*dx+dy*dy)
            if dist > 20:
                th = math.atan2(dy,dx)
                nx,ny = x+SPEED*math.cos(th), y+SPEED*math.sin(th)
                if collides(nx,ny):
                    moved = False
                    if not collides(nx,y): x=nx; moved=True
                    if not collides(x,ny) and abs(ny-y)>0.5: y=ny; moved=True
                    if not moved:
                        for perp in [th+math.pi/2, th-math.pi/2]:
                            tx_=x+SPEED*math.cos(perp); ty_=y+SPEED*math.sin(perp)
                            if not collides(tx_,ty_) and 75<=tx_<=ARENA_WIDTH-75 and 75<=ty_<=ARENA_HEIGHT-75:
                                x,y = tx_,ty_; break
                else:
                    x,y = nx,ny
        x=max(75,min(ARENA_WIDTH-75,x)); y=max(75,min(ARENA_HEIGHT-75,y))
        positions[rid]=[x,y,th]
        visited.add((int(x/CELL_SIZE),int(y/CELL_SIZE)))

# ===== PHASE 1: Clean run (30s) =====
total_cells = (ARENA_WIDTH//CELL_SIZE)*(ARENA_HEIGHT//CELL_SIZE)
start = time.time()
leader_seen = False

print("\n=== PHASE 1: Clean run (30s, multiroom) ===")
for tick in range(600):
    send_positions()
    states = recv_states()
    update_pos()
    elapsed = time.time()-start
    for s in states.values():
        if s.get("state")=="leader" and not leader_seen:
            print(f"  [{elapsed:.1f}s] Leader elected: Robot {s.get('robot_id')}")
            leader_seen = True
    if tick % 100 == 99:
        cov = len(visited)/total_cells*100
        print(f"  [{elapsed:.1f}s] Coverage: {cov:.1f}% ({len(visited)}/{total_cells})")
    time.sleep(0.05)

cov_clean = len(visited)/total_cells*100
print(f"\n  Clean result: {cov_clean:.1f}% in {time.time()-start:.1f}s")

# Check false positives
fp_count = 0
for i in range(1, NUM_ROBOTS+1):
    with open(f"logs/robot_{i}_console.log") as f:
        content = f.read()
        fp_count += content.count("Bad command")
print(f"  False positives (bad command triggers): {fp_count}")

# ===== PHASE 2: Inject bad_leader on current leader (20s) =====
print("\n=== PHASE 2: Inject bad_leader (20s) ===")
leader_id = None
for s in states.values():
    if s.get("state") == "leader":
        leader_id = s.get("robot_id")
        break
if leader_id is None:
    leader_id = 1  # fallback

fault_msg = json.dumps({"type":"fault_inject","robot_id":leader_id,"fault":"bad_leader","timestamp":time.time()})
fault_sock.sendto(fault_msg.encode(), ("127.0.0.1", UDP_FAULT_PORT + leader_id))
print(f"  Injected bad_leader on Robot {leader_id}")

fault_start = time.time()
detected = False
new_leader = None

for tick in range(400):
    send_positions()
    states = recv_states()
    update_pos()
    elapsed = time.time()-fault_start

    # Check if leader was impeached
    for s in states.values():
        rid = s.get("robot_id")
        if rid and rid != leader_id and s.get("state") == "leader":
            if not detected:
                detected = True
                new_leader = rid
                print(f"  [{elapsed:.1f}s] DETECTED! New leader: Robot {rid}")
    
    if tick % 100 == 99:
        cov = len(visited)/total_cells*100
        print(f"  [{elapsed:.1f}s] Coverage: {cov:.1f}%")
    time.sleep(0.05)

cov_final = len(visited)/total_cells*100
print(f"\n  Fault result: {cov_final:.1f}% total coverage")
print(f"  Detection: {'YES' if detected else 'NO'}")
if detected:
    print(f"  New leader: Robot {new_leader}")

# Check logs for detection details
print("\n=== Detection details from logs ===")
for i in range(1, NUM_ROBOTS+1):
    if i == leader_id:
        continue
    with open(f"logs/robot_{i}_console.log") as f:
        for line in f:
            line = line.strip()
            if "DETECT" in line or "IMPEACH" in line or "ELECTION" in line:
                print(f"  R{i}: {line}")

# Cleanup
for p in procs: p.terminate()
for p in procs:
    try: p.wait(timeout=2)
    except: p.kill()

print(f"\n{'='*50}")
print(f"SUMMARY")
print(f"  Clean coverage:  {cov_clean:.1f}%")
print(f"  Final coverage:  {cov_final:.1f}%")
print(f"  False positives: {fp_count}")
print(f"  Fault detected:  {detected}")
print(f"{'='*50}")
