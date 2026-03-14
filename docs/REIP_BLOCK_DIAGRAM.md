# REIP System - High-Level Block Diagram

## Ultra-Simple Version

```
Camera → Position Server → Pi Zero (REIP Brain) → Pico → Motors
                              ↑                        ↓
                         Other Robots              Encoders
                              ↑                        ↓
                         (WiFi/UDP)              (Feedback)
                              │
                         ┌────┴────┐
                         │  ToF   │
                         │Sensors │
                         └────────┘
```

## Detailed Component Flow

```
                    ┌─────────────┐
                    │   Camera    │
                    │  (ArUco)    │
                    └──────┬──────┘
                           │
                           ▼
                    ┌─────────────┐
                    │  Position   │
                    │   Server    │
                    │   (Laptop)  │
                    └──────┬──────┘
                           │ UDP 5100
                           │ (Position)
                           ▼
        ┌──────────────────────────────────────┐
        │      RASPBERRY PI ZERO 2W             │
        │      (Main Controller)                │
        │                                       │
        │  ┌──────────────────────────────┐    │
        │  │    REIP Control Logic        │    │
        │  │  • Trust • Election          │    │
        │  │  • Navigation • Coverage     │    │
        │  └──────────────────────────────┘    │
        │         │        │        │          │
        └─────────┼────────┼────────┼──────────┘
                  │        │        │
         ┌────────┘        │        └────────┐
         │                 │                  │
         ▼                 ▼                  ▼
    ┌─────────┐      ┌─────────┐      ┌─────────┐
    │   UDP   │      │   I2C   │      │  UART    │
    │ Network │      │   Bus   │      │ Serial   │
    └────┬────┘      └────┬────┘      └────┬────┘
         │                │                 │
         │                │                 │
    ┌────┴────┐      ┌────┴────┐      ┌────┴────┐
    │  Other  │      │ I2C Mux │      │  Pico   │
    │ Robots  │      │         │      │         │
    │ (WiFi)  │      │  ToF    │      │ Motor   │
    │         │      │ Sensors │      │ Control │
    └─────────┘      └─────────┘      └────┬────┘
                                           │
                                           ▼
                                    ┌─────────────┐
                                    │  DRV8833    │
                                    │   Driver    │
                                    └──────┬──────┘
                                           │
                                           ▼
                                    ┌─────────────┐
                                    │   Motors    │
                                    │  (N20)      │
                                    └──────┬──────┘
                                           │
                                           ▼
                                    ┌─────────────┐
                                    │  Encoders   │
                                    │ (Feedback)  │
                                    └──────┬──────┘
                                           │
                                           └──────▶ Back to Pico
```

## Data Flow Summary

### Inputs (Sensors & External Data)
```
Camera → Position Server → UDP → Pi Zero
ToF Sensors → I2C Mux → I2C → Pi Zero
Encoders → Pico → UART → Pi Zero
Other Robots → WiFi → UDP → Pi Zero
```

### Processing
```
Pi Zero: REIP Control Logic
  • Processes all inputs
  • Makes decisions (trust, navigation, election)
  • Generates motor commands
```

### Outputs (Actuators & Communication)
```
Pi Zero → UART → Pico → DRV8833 → Motors
Pi Zero → UDP → Other Robots (peer state)
```

## Component Roles

| Component | Role | Key Function |
|-----------|------|--------------|
| **Camera** | Vision | Detects robot positions via ArUco tags |
| **Position Server** | Processing | Converts camera data to coordinates, broadcasts via UDP |
| **Pi Zero** | Brain | Runs REIP algorithm, coordinates team, makes decisions |
| **I2C Mux** | Router | Routes I2C bus to 5 different ToF sensors |
| **ToF Sensors** | Proximity | Detects obstacles (walls, robots) within 200mm |
| **Pico** | Motor Controller | Handles PWM generation and encoder reading |
| **DRV8833** | Power | Drives motors with current protection |
| **Motors** | Actuators | Physical movement (differential drive) |
| **Encoders** | Feedback | Measures wheel rotation for odometry |

## Communication Paths

```
┌─────────────┐
│   Camera    │──UDP 5100──▶ Pi Zero (Position)
└─────────────┘

┌─────────────┐
│  Pi Zero    │──UDP 5200──▶ Other Robots (Peer State)
└─────────────┘

┌─────────────┐
│  Pi Zero    │──I2C──────▶ I2C Mux → ToF Sensors
└─────────────┘

┌─────────────┐
│  Pi Zero    │──UART─────▶ Pico (Motor Commands)
└─────────────┘              │
                             │
                             ▼
                    ┌─────────────┐
                    │    Pico     │──PWM──▶ DRV8833 → Motors
                    └─────────────┘
                             │
                             │
                             ▼
                    ┌─────────────┐
                    │  Encoders   │──IRQ──▶ Pico (Feedback)
                    └─────────────┘
```

## Simple One-Liner Flow

**Sensors → Pi Zero (REIP Logic) → Pico (Motors) → Physical Motion**

With feedback loops:
- **Position**: Camera → Pi Zero (for navigation)
- **Obstacles**: ToF → Pi Zero (for avoidance)
- **Movement**: Encoders → Pico → Pi Zero (for stuck detection)
- **Coordination**: Pi Zero ↔ Other Robots (for team behavior)
