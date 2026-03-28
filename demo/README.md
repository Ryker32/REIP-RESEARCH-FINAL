# Live Demo

Presentation-ready side-by-side REIP vs Raft comparison.

Based on `test/visual_sim.py` -- spawns **real robot node processes**
(`reip_node.py` and `raft_node.py`) on separate UDP port ranges,
relays peer messages, and renders two VisualSimulation panels in one
Pygame window.

## Run

```bash
python demo/live_comparison_demo.py [num_robots] [layout] [fault_delay_s]
```

Defaults: 5 robots, `open` layout, auto-fault at 15 seconds.

Examples:
```bash
python demo/live_comparison_demo.py               # defaults
python demo/live_comparison_demo.py 5 open 20     # fault at 20s
python demo/live_comparison_demo.py 5 multiroom   # two-room layout
```

## What It Shows

- REIP on the left, Raft on the right
- Same arena, same starting positions, same walls
- Automatic `bad_leader` fault injected on **both** sides simultaneously
- Per-robot trust readouts, leader halos, suspicion bars
- Live coverage history showing REIP's recovery gap

## Controls

| Key | Action |
|-----|--------|
| `F` | Inject `bad_leader` on current leader (both sides) |
| `Shift+F` | Inject `freeze_leader` on current leader (both sides) |
| `C` | Clear all faults |
| `Space` | Pause / resume |
| `Up` / `Down` | Speed up / slow down |
| `Q` or `Esc` | Quit |
