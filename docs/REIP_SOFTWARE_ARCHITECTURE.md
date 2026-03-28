# REIP Software Architecture - Block Diagram

## Ultra-Simple Version

```
Position -> Coverage -> Trust -> Election -> Assignment -> Navigation -> Motors
Sensors ->  Merging   Assessment         (Leader)     (A* + Avoid)   Commands
Peers  ->                                                             
```

## Complete REIP Software Flow (Including Leader Commands)

```
        ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
        │  Position   │  │   ToF       │  │   Peer      │
        │  Server     │  │  Sensors    │  │   States    │
        └──────┬──────┘  └──────┬──────┘  └──────┬──────┘
               │                │                 │
                                                
        ┌─────────────────────────────────────────────┐
        │         Coverage Merging                    │
        │  • Personal visited cells (my_visited)      │
        │  • Peer reported cells (known_visited)      │
        │  • Obstacle cells (tof_obstacles)           │
        └──────────────────┬──────────────────────────┘
                           │
                           
        ┌─────────────────────────────────────────────┐
        │      REIP Control Logic                     │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Leader Election                    │  │
        │  │   & Impeachment                      │  │
        │  │  • Check trust threshold             │  │
        │  │  • Vote counting                     │  │
        │  │  • Elect new leader                  │  │
        │  └───────────┬──────────────────────────┘  │
        │              │                               │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Task Assignment (Leader Only)      │  │
        │  │                                      │  │
        │  │  • Detect frontiers                  │  │
        │  │  • Greedy assignment                 │  │
        │  │  • Generate commands                 │  │
        │  │    {robot_id: (x, y) target}        │  │
        │  └───────────┬──────────────────────────┘  │
        │              │                               │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Command Broadcast (UDP 5200)       │  │
        │  │   • Include assignments in message   │  │
        │  │   • Send to all followers            │  │
        │  └───────────┬──────────────────────────┘  │
        │              │                               │
        │              │                                 │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Command Reception (Followers)      │  │
        │  │   • Extract assigned_target          │  │
        │  │   • Store as leader_assigned_target  │  │
        │  └───────────┬──────────────────────────┘  │
        │              │                               │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Proactive Command Verification     │  │
        │  │   (assess_leader_command)            │  │
        │  │                                      │  │
        │  │  ┌──────────────────────────────┐  │  │
        │  │  │  Three-Tier Trust Check      │  │  │
        │  │  │                              │  │  │
        │  │  │  Tier 1: Personal History    │  │  │
        │  │  │  • Cell in my_visited?      │  │  │
        │  │  │  • Weight: 1.0              │  │  │
        │  │  │                              │  │  │
        │  │  │  Tier 2: ToF Sensor         │  │  │
        │  │  │  • Cell in tof_obstacles?   │  │  │
        │  │  │  • Weight: 1.0              │  │  │
        │  │  │                              │  │  │
        │  │  │  Tier 3: Peer Reports        │  │  │
        │  │  │  • Cell in known_visited?   │  │  │
        │  │  │  • Weight: 0.3              │  │  │
        │  │  └──────────────────────────────┘  │  │
        │  │                                      │  │
        │  │  ┌──────────────────────────────┐  │  │
        │  │  │  MPC Direction Check          │  │  │
        │  │  │  • Command vs. nearest        │  │  │
        │  │  │    unexplored area            │  │  │
        │  │  │  • Severe (>135deg) or          │  │  │
        │  │  │    Moderate (90-135deg)         │  │  │
        │  │  └──────────────────────────────┘  │  │
        │  │                                      │  │
        │  │  -> Updates: suspicion_of_leader    │  │
        │  │     trust_in_leader                 │  │
        │  └───────────┬──────────────────────────┘  │
        │              │                               │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Navigation Decision                │  │
        │  │                                      │  │
        │  │  If valid command:                   │  │
        │  │    Use leader_assigned_target        │  │
        │  │  Else:                               │  │
        │  │    Use local frontier                │  │
        │  └───────────┬──────────────────────────┘  │
        │              │                               │
        │                                             │
        │  ┌──────────────────────────────────────┐  │
        │  │   Navigation                         │  │
        │  │   • A* Pathfinding                  │  │
        │  │   • Wall Avoidance (ToF)             │  │
        │  │   • Stuck Detection                  │  │
        │  │   • Motor Command Generation         │  │
        │  └──────────────────────────────────────┘  │
        └──────────────────┬───────────────────────────┘
                           │
                           
        ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
        │   Motor     │  │   Peer      │  │   Logging   │
        │  Commands   │  │  Broadcast  │  │   (JSON)    │
        │  (UART)     │  │  (UDP 5200) │  │             │
        └─────────────┘  └─────────────┘  └─────────────┘
```

## Leader Command Flow (Detailed)

```
LEADER SIDE:
┌─────────────────────────────────────────────────────┐
│  Leader Robot                                       │
│                                                     │
│  1. compute_task_assignments()                     │
│     • Detect frontiers (unexplored boundaries)     │
│     • Greedy assignment: nearest frontier          │
│     • Returns: {robot_id: (x, y) target}          │
│                                                     │
│  2. broadcast_state()                              │
│     • Include assignments in peer_state message    │
│     • Send via UDP 5200 to all followers          │
│                                                     │
│  Message Format:                                   │
│  {                                                 │
│    "robot_id": 1,                                  │
│    "state": "leader",                              │
│    "assignments": {                                │
│      "2": [1200.0, 800.0],  // Follower 2         │
│      "3": [500.0, 1200.0]   // Follower 3         │
│    },                                              │
│    ...                                             │
│  }                                                 │
└─────────────────────────────────────────────────────┘
                    │
                    │ UDP 5200
                    │
                    
┌─────────────────────────────────────────────────────┐
│  Follower Robot                                    │
│                                                     │
│  1. receive_peer_states()                          │
│     • Receives leader's peer_state message        │
│     • Extracts assigned_target for self           │
│     • Stores as leader_assigned_target            │
│                                                     │
│  2. assess_leader_command()                        │
│     • Called when leader_assigned_target changes  │
│     • BEFORE execution (proactive)                │
│                                                     │
│     Three-Tier Check:                              │
│     ├─ Tier 1: Cell in my_visited?                │
│     ├─ Tier 2: Cell in tof_obstacles?            │
│     ├─ Tier 3: Cell in known_visited?             │
│     └─ MPC: Command direction vs. frontiers       │
│                                                     │
│     -> Updates suspicion_of_leader                 │
│     -> Updates trust_in_leader                      │
│                                                     │
│  3. Navigation Decision                            │
│     If command valid:                              │
│       Use leader_assigned_target                   │
│     Else:                                          │
│       Use local frontier (get_my_frontier)        │
│                                                     │
│  4. Navigation & Motor Commands                    │
│     • A* pathfinding to target                    │
│     • Wall avoidance                              │
│     • Generate motor PWM                          │
└─────────────────────────────────────────────────────┘
```

## Simplified Version

```
Inputs -> Coverage Merging -> Trust Assessment -> Election -> Task Assignment -> Navigation -> Outputs
         (Position,         (3-Tier Check)   (Leader    (Frontier        (A* +        (Motors,
          Sensors,          + MPC)           Selection)  Assignment)      Avoidance)   Broadcast,
          Peers)                                                                       Log)
```

## Component Details

### Input Processing
- **Position Server**: ArUco camera -> robot pose (x, y, theta) -> visited cell marking
- **ToF Sensors**: Distance readings -> obstacle cell map
- **Peer States**: Network broadcasts -> coverage merging, trust signals

### Coverage Merging
- Combines personal visits, peer reports, and sensor obstacles
- Maintains `my_visited`, `known_visited`, `tof_obstacles` sets
- Tracks timestamps for causality-aware checking

### Trust Assessment (Proactive Verification)
- **Tier 1**: Personal visit history (weight 1.0) - ground truth
- **Tier 2**: ToF sensor obstacles (weight 1.0) - physically verifiable
- **Tier 3**: Peer-reported coverage (weight 0.3) - may be stale
- **MPC Check**: Compares command direction to local optimum
- Updates `suspicion_of_leader` and `trust_in_leader`

### Leader Election & Impeachment
- Checks impeachment if `trust_in_leader < 0.3`
- Builds candidate list (robots with `trust > 0.5`)
- Counts votes from peers
- Elects new leader if consensus reached

### Task Assignment (Leader Only)
- Detects frontiers (boundaries between explored/unexplored)
- Greedy assignment: nearest frontier to each robot
- Broadcasts assignments to followers

### Navigation
- A* pathfinding to assigned target
- Wall avoidance using ToF sensor data
- Stuck detection with encoder fallback
- Generates left/right motor PWM commands

### Output
- **Motor Commands**: UART to Pico -> DRV8833 -> Motors
- **Peer Broadcast**: UDP to other robots (state, trust, assignments)
- **Logging**: JSON file for visualization and analysis

## Data Flow Summary

```
Position Updates -> Coverage Map -> Trust Assessment -> Leader Decision
                                                          │
                                                          ├─-> If Leader: Task Assignment
                                                          │
                                                          └─-> If Follower: Receive Assignment
                                                                      │
                                                                      
                                                              Navigation -> Motor Commands
```

## Key Design Features

1. **Proactive Verification**: Commands verified before execution (not reactive)
2. **Three-Tier Trust**: Confidence-weighted evidence (personal > sensor > peer)
3. **Causality-Aware**: Accounts for network delays to prevent false positives
4. **Distributed Consensus**: Each robot independently assesses and votes
5. **Real-Time Control**: 10 Hz control loop with 5 Hz peer broadcasts
