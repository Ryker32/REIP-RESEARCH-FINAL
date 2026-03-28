# REIP System Architecture Diagram

## Mermaid Diagram (Renders in GitHub/Markdown)

```mermaid
graph TB
    subgraph External["External Systems"]
        Camera["Overhead Camera<br/>(ArUco Detection)"]
        PositionServer["Position Server<br/>(aruco_position_server.py)"]
        Peers["Peer Robots<br/>(UDP Broadcast)"]
        FaultInjector["Fault Injector<br/>(UDP Port 5300)"]
    end

    subgraph REIPNode["REIP Node (reip_node.py)"]
        subgraph Threads["Concurrent Threads"]
            SensorThread["sensor_loop()<br/>~8-10 Hz<br/>ToF Reading"]
            NetworkThread["network_loop()<br/>100 Hz Polling<br/>UDP I/O"]
            ControlThread["control_loop()<br/>10 Hz<br/>Main Logic"]
        end

        subgraph Inputs["Input Processing"]
            PositionRx["receive_position()<br/>UDP 5100<br/>Updates: x, y, theta"]
            PeerRx["receive_peer_states()<br/>UDP 5200<br/>Updates: peers[]"]
            FaultRx["receive_fault_injection()<br/>UDP 5300<br/>Sets fault modes"]
            ToFProc["update_tof_obstacles()<br/>Converts ToF -> cells"]
        end

        subgraph Core["Core REIP Logic"]
            Trust["assess_leader_command()<br/>Three-Tier Verification<br/>Tier 1: Personal (1.0)<br/>Tier 2: ToF (1.0)<br/>Tier 3: Peer (0.3)"]
            Election["run_election()<br/>Trust-Weighted Voting<br/>Every 2s"]
            Impeach["check_impeachment()<br/>Trust < 0.3"]
            Assignment["compute_task_assignments()<br/>Greedy Nearest-Frontier<br/>(Leader Only)"]
        end

        subgraph State["State Management"]
            Coverage["Coverage Maps<br/>my_visited: Dict[cell, time]<br/>known_visited: Set[cell]"]
            TrustState["Trust State<br/>trust_in_leader: float<br/>suspicion_of_leader: float"]
            Leadership["Leadership State<br/>current_leader: int<br/>my_vote: int<br/>leader_failures: Dict"]
        end

        subgraph Navigation["Navigation & Control"]
            Frontier["get_my_frontier()<br/>Nearest Unexplored Cell"]
            MotorCmd["compute_motor_command()<br/>A* Pathfinding<br/>Wall Avoidance<br/>Peer Avoidance"]
            StuckDetect["_check_stuck()<br/>Position + Encoder"]
        end

        subgraph Outputs["Outputs"]
            MotorOut["hw.set_motors()<br/>UART to Pico<br/>PWM Commands"]
            Broadcast["broadcast_state()<br/>UDP 5200<br/>5 Hz"]
            Logging["log_state()<br/>JSON File<br/>For Visualization"]
        end
    end

    subgraph Hardware["Hardware Layer"]
        Pico["Raspberry Pi Pico<br/>(main.py)"]
        Motors["DRV8833<br/>Motor Driver"]
        ToFSensors["VL53L0X ToF<br/>Sensors (x5)"]
        Encoders["Quadrature<br/>Encoders"]
    end

    %% External to REIP
    Camera -->|USB| PositionServer
    PositionServer -->|UDP 5100| PositionRx
    Peers -->|UDP 5200| PeerRx
    FaultInjector -->|UDP 5300| FaultRx

    %% Thread connections
    SensorThread --> ToFProc
    NetworkThread --> PositionRx
    NetworkThread --> PeerRx
    NetworkThread --> FaultRx
    NetworkThread --> Broadcast
    ControlThread --> Trust
    ControlThread --> Election
    ControlThread --> Impeach
    ControlThread --> Assignment
    ControlThread --> MotorCmd

    %% Data flow
    PositionRx --> Coverage
    PeerRx --> Coverage
    PeerRx --> Leadership
    ToFProc --> State
    Coverage --> Trust
    Coverage --> Assignment
    Coverage --> Frontier
    TrustState --> Trust
    TrustState --> Election
    TrustState --> Impeach
    Leadership --> Election
    Assignment --> MotorCmd
    Frontier --> MotorCmd
    Trust --> TrustState
    Election --> Leadership
    MotorCmd --> StuckDetect
    StuckDetect --> MotorCmd
    MotorCmd --> MotorOut
    MotorCmd --> Broadcast
    MotorCmd --> Logging

    %% Hardware connections
    SensorThread --> ToFSensors
    MotorOut -->|UART| Pico
    Pico --> Motors
    Pico --> Encoders
    Encoders -.->|UART| StuckDetect

    style REIPNode fill:#e1f5ff
    style Core fill:#fff4e1
    style Trust fill:#ffcccc
    style Election fill:#ccffcc
    style Hardware fill:#f0f0f0
```

## Detailed Component Diagram (Mermaid)

```mermaid
graph LR
    subgraph "REIP Node Architecture"
        A[Position Server<br/>UDP 5100] -->|x, y, theta| B[receive_position]
        B --> C[Coverage Tracking<br/>my_visited<br/>known_visited]
        
        D[Peer Broadcasts<br/>UDP 5200] -->|state messages| E[receive_peer_states]
        E --> F[update_peer]
        F --> C
        F --> G[Peer State<br/>peers[]]
        
        H[ToF Sensors] -->|distances| I[read_tof_all]
        I --> J[update_tof_obstacles]
        J --> K[tof_obstacles Set]
        
        L[Fault Injector<br/>UDP 5300] -->|fault modes| M[receive_fault_injection]
        M --> N[bad_leader_mode<br/>oscillate_leader_mode<br/>freeze_leader_mode]
        
        C --> O[assess_leader_command<br/>Three-Tier Trust]
        K --> O
        G --> O
        N --> P[compute_task_assignments]
        
        O --> Q[Trust State<br/>trust_in_leader<br/>suspicion_of_leader]
        Q --> R[check_impeachment]
        Q --> S[run_election]
        R --> S
        S --> T[Leadership State<br/>current_leader<br/>my_vote]
        
        T --> P
        C --> P
        G --> P
        P --> U[leader_assigned_target]
        
        U --> V[compute_motor_command]
        C --> W[get_my_frontier]
        W --> V
        K --> V
        G --> V
        V --> X[hw.set_motors<br/>left, right PWM]
        
        V --> Y[broadcast_state<br/>UDP 5200]
        Y --> D
    end

    style O fill:#ffcccc
    style S fill:#ccffcc
    style P fill:#ccccff
    style V fill:#ffffcc
```

## ASCII Text Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         REIP SYSTEM ARCHITECTURE                              │
└─────────────────────────────────────────────────────────────────────────────┘

EXTERNAL SYSTEMS
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│ Overhead Camera │      │  Peer Robots    │      │ Fault Injector  │
│  (ArUco Tags)   │      │  (UDP 5200)     │      │  (UDP 5300)     │
└────────┬────────┘      └────────┬────────┘      └────────┬────────┘
         │                        │                        │
         │ USB                    │                        │
                                 │                        │
┌─────────────────┐              │                        │
│ Position Server │              │                        │
│ (PC/Laptop)     │              │                        │
│ UDP 5100        │              │                        │
└────────┬────────┘              │                        │
         │                        │                        │
         └────────────────────────┼────────────────────────┘
                                  │
                                  
┌─────────────────────────────────────────────────────────────────────────────┐
│                          REIP NODE (reip_node.py)                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  CONCURRENT THREADS                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                     │
│  │ sensor_loop  │  │ network_loop │  │control_loop  │                     │
│  │ ~8-10 Hz     │  │ 100 Hz poll  │  │ 10 Hz        │                     │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                     │
│         │                 │                  │                              │
│                                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                     │
│  │ read_tof_all │  │receive_pos   │  │assess_leader │                     │
│  │              │  │receive_peer  │  │_command()    │                     │
│  │              │  │receive_fault │  │              │                     │
│  │              │  │broadcast_    │  │run_election()│                     │
│  │              │  │state()        │  │              │                     │
│  └──────┬───────┘  └──────┬───────┘  │compute_motor │                     │
│         │                 │           │_command()    │                     │
│                                     └──────┬───────┘                     │
│  ┌──────────────┐  ┌──────────────┐         │                              │
│  │update_tof_   │  │update_peer() │         │                              │
│  │obstacles()   │  │              │         │                              │
│  └──────┬───────┘  └──────┬───────┘         │                              │
│         │                 │                  │                              │
│         └─────────────────┼──────────────────┘                              │
│                           │                                                 │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────┐              │
│  │              STATE MANAGEMENT                            │              │
│  ├──────────────────────────────────────────────────────────┤              │
│  │                                                           │              │
│  │  Coverage Maps:                                          │              │
│  │    • my_visited: Dict[cell, timestamp]                   │              │
│  │    • known_visited: Set[cell]                           │              │
│  │    • known_visited_time: Dict[cell, timestamp]           │              │
│  │                                                           │              │
│  │  Trust State:                                            │              │
│  │    • trust_in_leader: float [0.0, 1.0]                   │              │
│  │    • suspicion_of_leader: float                          │              │
│  │    • bad_commands_received: int                          │              │
│  │                                                           │              │
│  │  Leadership State:                                        │              │
│  │    • current_leader: Optional[int]                       │              │
│  │    • my_vote: Optional[int]                               │              │
│  │    • leader_failures: Dict[int, int]                    │              │
│  │                                                           │              │
│  │  Peer State:                                             │              │
│  │    • peers: Dict[int, PeerInfo]                          │              │
│  │      - x, y, theta, trust_score, vote, etc.              │              │
│  │                                                           │              │
│  │  Sensor State:                                           │              │
│  │    • tof: Dict[str, int]  (sensor_name: distance_mm)    │              │
│  │    • tof_obstacles: Set[cell]                            │              │
│  └──────────────────────────────────────────────────────────┘              │
│                           │                                                 │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────┐              │
│  │         CORE REIP PROCESSES                              │              │
│  ├──────────────────────────────────────────────────────────┤              │
│  │                                                           │              │
│  │  1. TRUST ASSESSMENT (assess_leader_command)             │              │
│  │     ┌─────────────────────────────────────┐              │              │
│  │     │ Three-Tier Verification:           │              │              │
│  │     │  Tier 1: Personal visit (weight 1.0)│              │              │
│  │     │  Tier 2: ToF obstacle (weight 1.0) │              │              │
│  │     │  Tier 3: Peer report (weight 0.3)  │              │              │
│  │     │                                       │              │              │
│  │     │ MPC Direction Check:                 │              │              │
│  │     │  Command vs. nearest frontier      │              │              │
│  │     │                                       │              │              │
│  │     │ Updates: suspicion_of_leader       │              │              │
│  │     │          trust_in_leader (on threshold)│              │              │
│  │     └─────────────────────────────────────┘              │              │
│  │                                                           │              │
│  │  2. ELECTION (run_election)                               │              │
│  │     ┌─────────────────────────────────────┐              │              │
│  │     │ Build candidates (trust > 0.5)     │              │              │
│  │     │ Sort by: trust, failures, id        │              │              │
│  │     │ Count votes from peers              │              │              │
│  │     │ Elect winner                        │              │              │
│  │     │ Update: current_leader, my_vote     │              │              │
│  │     └─────────────────────────────────────┘              │              │
│  │                                                           │              │
│  │  3. TASK ASSIGNMENT (compute_task_assignments)          │              │
│  │     ┌─────────────────────────────────────┐              │              │
│  │     │ Find frontiers (unexplored cells)   │              │              │
│  │     │ Greedy nearest-frontier assignment │              │              │
│  │     │ Spatial diversity (both rooms)     │              │              │
│  │     │ Returns: {robot_id: (x, y)}        │              │              │
│  │     └─────────────────────────────────────┘              │              │
│  │                                                           │              │
│  │  4. NAVIGATION (compute_motor_command)                    │              │
│  │     ┌─────────────────────────────────────┐              │              │
│  │     │ Get target (leader assignment OR   │              │              │
│  │     │              local frontier)        │              │              │
│  │     │ A* pathfinding                      │              │              │
│  │     │ Wall avoidance                      │              │              │
│  │     │ Peer avoidance                      │              │              │
│  │     │ Stuck detection & escape            │              │              │
│  │     │ Returns: (left_pwm, right_pwm)      │              │              │
│  │     └─────────────────────────────────────┘              │              │
│  └──────────────────────────────────────────────────────────┘              │
│                           │                                                 │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────┐              │
│  │                    OUTPUTS                               │              │
│  ├──────────────────────────────────────────────────────────┤              │
│  │  • hw.set_motors(left, right) -> UART -> Pico             │              │
│  │  • broadcast_state() -> UDP 5200 -> Peers                  │              │
│  │  • log_state() -> JSON file -> Visualization              │              │
│  └──────────────────────────────────────────────────────────┘              │
└─────────────────────────────────────────────────────────────────────────────┘
                                  │
                                  
┌─────────────────────────────────────────────────────────────────────────────┐
│                         HARDWARE LAYER                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐              │
│  │ Pi Pico      │      │ DRV8833      │      │ VL53L0X ToF  │              │
│  │ (main.py)    │      │ Motor Driver │      │ Sensors (x5) │              │
│  │              │      │              │      │              │              │
│  │ • PWM Gen    │─────│ • Left Motor │      │ • Front      │              │
│  │ • Encoder    │      │ • Right Motor│      │ • Front-L    │              │
│  │   Reading    │      │              │      │ • Front-R     │              │
│  │              │      │              │      │ • Left        │              │
│  │ UART 115200  │      └──────────────┘      │ • Right      │              │
│  └──────┬───────┘                            └──────────────┘              │
│         │                                                                    │
│         │ UART                                                               │
│         │                                                                    │
│         └───────────────────────────────────────────────────────────────────┘
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Sequence Diagram

```mermaid
sequenceDiagram
    participant PS as Position Server
    participant RN as REIP Node
    participant Peer as Peer Robot
    participant HW as Hardware (Pico)

    Note over RN: Startup Phase
    PS->>RN: Position (x, y, theta)
    RN->>RN: Mark cell as visited
    RN->>Peer: Broadcast state (5 Hz)
    Peer->>RN: Broadcast state
    RN->>RN: Merge coverage maps
    RN->>RN: run_election() -> Elect leader

    Note over RN: Normal Operation
    RN->>RN: compute_task_assignments() (if leader)
    RN->>Peer: Broadcast assignments
    Peer->>RN: Assignment received
    RN->>RN: assess_leader_command()
    RN->>RN: Three-tier check
    alt Bad Command Detected
        RN->>RN: suspicion_of_leader += weight
        RN->>RN: trust_in_leader -= 0.2 (if threshold crossed)
    else Good Command
        RN->>RN: suspicion_of_leader -= 0.1 (recovery)
    end
    RN->>RN: compute_motor_command()
    RN->>HW: set_motors(left, right)
    HW->>RN: Encoder feedback
    RN->>RN: Update position -> Mark coverage

    Note over RN: Fault Detection
    RN->>RN: trust_in_leader < 0.3
    RN->>RN: check_impeachment()
    RN->>RN: run_election() -> Exclude bad leader
    RN->>RN: Elect new leader
    RN->>RN: Reset trust for new leader
```

## How to Use These Diagrams

### Option 1: Mermaid (Recommended for GitHub/Markdown)
- Copy the Mermaid code blocks above
- Paste into any Markdown file (GitHub will render automatically)
- Or use [Mermaid Live Editor](https://mermaid.live) to export as PNG/SVG

### Option 2: Draw.io / diagrams.net
1. Go to [diagrams.net](https://app.diagrams.net/)
2. Create new diagram
3. Use the ASCII diagram above as a reference
4. Create boxes for each component
5. Connect with arrows showing data flow

### Option 3: PlantUML
- Similar to Mermaid, text-based
- Good for sequence diagrams
- Can export to PNG/SVG

### Option 4: PowerPoint/Keynote
- Use the ASCII diagram as a template
- Create boxes and arrows manually
- Good for presentations

## Key Components to Highlight

1. **Three Concurrent Threads**: sensor_loop, network_loop, control_loop
2. **Core REIP Processes**: Trust assessment, Election, Task assignment, Navigation
3. **State Management**: Coverage maps, Trust state, Leadership state
4. **External Interfaces**: Position server, Peer communication, Fault injection
5. **Hardware Layer**: Pico, Motors, ToF sensors, Encoders

## Color Coding Suggestions

- **Blue**: Input/Output interfaces
- **Yellow**: Core REIP logic (trust, election)
- **Green**: State management
- **Orange**: Navigation/Control
- **Gray**: Hardware layer
