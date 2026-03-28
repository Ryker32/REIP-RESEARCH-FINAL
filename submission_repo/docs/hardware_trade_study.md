# Hardware Trade Study: REIP Multi-Robot System

## Executive Summary

**Recommended**: **Raspberry Pi Zero 2 W** - Best balance of performance, cost, and development ease.

**Budget Option**: **Raspberry Pi Pico W** - If cost is critical, but requires more optimization.

**Overkill**: **Raspberry Pi 4B** - Only if you need more processing for future features (camera, ML).

---

## 1. Requirements Analysis

### Computational Requirements

| Task | Follower | Leader | Notes |
|------|---------|--------|-------|
| **Sensor Reading** | 1-2 ms | 1-2 ms | VL53L0X ToF (I2C) |
| **SLAM Update** | 5-10 ms | 5-10 ms | Belief map, conservative writeback |
| **Communication** | 2-3 ms | 2-3 ms | UDP multicast, map sharing |
| **Pathfinding** | 1-2 ms | 1-2 ms | BFS to target |
| **MPC Trust** | 2.5 ms | 0 ms | Only followers compute |
| **Frontier Detection** | 0 ms | 10-20 ms | Leader only |
| **Frontier Assignment** | 0 ms | 5-10 ms | Leader only |
| **Trust Update** | 0.1 ms | 0.1 ms | Exponential decay |
| **Motor Control** | 0.5 ms | 0.5 ms | PID, encoder feedback |
| **Total** | **12-20 ms** | **27-50 ms** | Per timestep |

### Control Loop Requirements

- **Frequency**: 1-5 Hz (200-1000 ms per timestep)
- **Real-time**: Must complete computation within timestep
- **Reliability**: 99%+ success rate

### System Requirements

- **Number of robots**: 6 (1 leader + 5 followers)
- **Communication**: WiFi (UDP multicast)
- **Sensors**: 2-4* VL53L0X ToF (I2C)
- **Motors**: 2* N20 micro gear motors with encoders
- **Power**: Battery-powered (prefer low power)
- **Size**: Micromouse form factor (compact)

---

## 2. Hardware Options Comparison

### Option 1: Raspberry Pi Zero 2 W * **RECOMMENDED**

#### Specifications

| Metric | Value |
|--------|-------|
| **CPU** | 1.0 GHz quad-core ARM Cortex-A53 |
| **RAM** | 512 MB |
| **Storage** | MicroSD (8+ GB recommended) |
| **Connectivity** | WiFi 802.11n, Bluetooth 4.2 |
| **GPIO** | 40-pin header (I2C, SPI, UART) |
| **Power** | 5V @ 200mA (~1W) |
| **Size** | 65mm * 30mm * 5mm |
| **Cost** | ~$15 |

#### Performance Analysis

**CPU Performance**:
- Single-threaded: ~500-800 MIPS
- Multi-threaded: ~2000-3200 MIPS
- **Our load**: 12-50 ms per timestep
- **Capacity**: 20-83 timesteps/second
- **Usage**: 1.2-25% (1-5 Hz control loop)

**Memory**:
- OS: ~100 MB
- Python runtime: ~50 MB
- Application: ~1 MB
- **Headroom**: ~360 MB

**Power Consumption**:
- Idle: ~100 mW
- Active (our load): ~200-300 mW
- **Battery life**: ~10-15 hours (2000 mAh battery)

#### Pros

[x] **Excellent performance/price ratio**  
[x] **Full Linux OS** (Raspberry Pi OS)  
[x] **Python support** (easy development)  
[x] **Built-in WiFi** (no additional module)  
[x] **Large community** (lots of tutorials/examples)  
[x] **GPIO support** (I2C, SPI, UART)  
[x] **Low power** (~1W)  
[x] **Small form factor** (fits micromouse)  
[x] **Sufficient headroom** (5-20x capacity)  

#### Cons

[ ] **More expensive than Pico** ($15 vs $6)  
[ ] **Higher power than Pico** (1W vs 0.1W)  
[ ] **Requires microSD card** (additional cost)  

#### Suitability Score: **9.5/10**

**Best for**: Most users - best balance of performance, cost, and ease of development.

---

### Option 2: Raspberry Pi Pico W (Budget Option)

#### Specifications

| Metric | Value |
|--------|-------|
| **CPU** | 133 MHz dual-core ARM Cortex-M0+ |
| **RAM** | 264 KB |
| **Storage** | 2 MB flash (internal) |
| **Connectivity** | WiFi 802.11n |
| **GPIO** | 26-pin header (I2C, SPI, UART) |
| **Power** | 5V @ 20mA (~0.1W) |
| **Size** | 51mm * 21mm * 1mm |
| **Cost** | ~$6 |

#### Performance Analysis

**CPU Performance**:
- Single-threaded: ~100-150 MIPS
- Multi-threaded: ~200-300 MIPS
- **Our load**: 12-50 ms per timestep
- **Capacity**: 2-8 timesteps/second
- **Usage**: 12.5-50% (1 Hz), 62.5-250% (5 Hz) [!]

**Memory**:
- OS: None (bare metal / MicroPython)
- Application: ~50-100 KB
- **Headroom**: ~164 KB

**Power Consumption**:
- Idle: ~10 mW
- Active (our load): ~50-100 mW
- **Battery life**: ~20-40 hours (2000 mAh battery)

#### Pros

[x] **Very low cost** ($6)  
[x] **Very low power** (~0.1W)  
[x] **Very small** (smallest option)  
[x] **Built-in WiFi**  
[x] **GPIO support** (I2C, SPI, UART)  
[x] **No microSD needed** (internal flash)  

#### Cons

[ ] **Limited CPU** (might be tight at 5 Hz)  
[ ] **Limited RAM** (264 KB - sufficient but tight)  
[ ] **No Linux** (MicroPython or C/C++ only)  
[ ] **Less software support** (smaller community)  
[ ] **Harder development** (no full OS)  
[ ] **May need optimization** (Strategy 3 required)  

#### Suitability Score: **6.5/10**

**Best for**: Budget-constrained projects, if you're willing to optimize heavily and test thoroughly.

**Recommendation**: Use for followers only, Pi Zero 2 W for leader.

---

### Option 3: Raspberry Pi 4B (Overkill)

#### Specifications

| Metric | Value |
|--------|-------|
| **CPU** | 1.5-1.8 GHz quad-core ARM Cortex-A72 |
| **RAM** | 2-8 GB (2 GB minimum) |
| **Storage** | MicroSD (16+ GB recommended) |
| **Connectivity** | WiFi 802.11ac, Bluetooth 5.0, Gigabit Ethernet |
| **GPIO** | 40-pin header (I2C, SPI, UART) |
| **Power** | 5V @ 1.2-2.5A (~6-12W) |
| **Size** | 88mm * 58mm * 19.5mm |
| **Cost** | ~$55 (2 GB), ~$75 (4 GB) |

#### Performance Analysis

**CPU Performance**:
- Single-threaded: ~2000-3000 MIPS
- Multi-threaded: ~8000-12000 MIPS
- **Our load**: 12-50 ms per timestep
- **Capacity**: 160-667 timesteps/second
- **Usage**: 0.15-3% (1-5 Hz control loop)

**Memory**:
- OS: ~200 MB
- Python runtime: ~100 MB
- Application: ~1 MB
- **Headroom**: ~1.7-7.7 GB

**Power Consumption**:
- Idle: ~500 mW
- Active (our load): ~1-2W
- **Battery life**: ~2-4 hours (2000 mAh battery) [!]

#### Pros

[x] **Extremely powerful** (10-30x headroom)  
[x] **Lots of RAM** (2-8 GB)  
[x] **Full Linux OS**  
[x] **Python support**  
[x] **Future-proof** (can add camera, ML, etc.)  
[x] **Gigabit Ethernet** (if needed)  

#### Cons

[ ] **Expensive** ($55-75)  
[ ] **High power** (6-12W - needs larger battery)  
[ ] **Large size** (might not fit micromouse)  
[ ] **Overkill** (10-30x more than needed)  
[ ] **Short battery life** (2-4 hours)  

#### Suitability Score: **5/10**

**Best for**: Future expansion (camera, ML, advanced features), but overkill for current requirements.

---

### Option 4: ESP32 (Ultra-Low Cost)

#### Specifications

| Metric | Value |
|--------|-------|
| **CPU** | 240 MHz dual-core Xtensa LX6 |
| **RAM** | 520 KB |
| **Storage** | 4-16 MB flash (external) |
| **Connectivity** | WiFi 802.11n, Bluetooth 4.2 |
| **GPIO** | 34-pin (I2C, SPI, UART) |
| **Power** | 3.3V @ 80-240mA (~0.3-0.8W) |
| **Size** | 25mm * 18mm * 3mm |
| **Cost** | ~$5-8 |

#### Performance Analysis

**CPU Performance**:
- Single-threaded: ~200-300 MIPS
- Multi-threaded: ~400-600 MIPS
- **Our load**: 12-50 ms per timestep
- **Capacity**: 8-50 timesteps/second
- **Usage**: 2-12.5% (1 Hz), 10-62.5% (5 Hz)

**Memory**:
- OS: None (FreeRTOS / Arduino)
- Application: ~100-200 KB
- **Headroom**: ~320-420 KB

**Power Consumption**:
- Idle: ~50 mW
- Active (our load): ~200-500 mW
- **Battery life**: ~4-8 hours (2000 mAh battery)

#### Pros

[x] **Very low cost** ($5-8)  
[x] **Low power** (~0.3-0.8W)  
[x] **Built-in WiFi/Bluetooth**  
[x] **GPIO support**  
[x] **Small form factor**  
[x] **Real-time OS** (FreeRTOS)  

#### Cons

[ ] **No Linux** (Arduino/FreeRTOS only)  
[ ] **Limited RAM** (520 KB - tight)  
[ ] **Python not native** (MicroPython possible but limited)  
[ ] **Less software support** (smaller community)  
[ ] **Harder development** (C/C++ or MicroPython)  
[ ] **May need porting** (code written for Linux)  

#### Suitability Score: **6/10**

**Best for**: Ultra-budget projects, if you're willing to rewrite code for Arduino/FreeRTOS.

---

### Option 5: BeagleBone Black (Alternative Linux)

#### Specifications

| Metric | Value |
|--------|-------|
| **CPU** | 1.0 GHz single-core ARM Cortex-A8 |
| **RAM** | 512 MB |
| **Storage** | 4 GB eMMC + MicroSD |
| **Connectivity** | Ethernet (WiFi via USB dongle) |
| **GPIO** | 92-pin header (I2C, SPI, UART, PRU) |
| **Power** | 5V @ 500mA (~2.5W) |
| **Size** | 86mm * 53mm * 15mm |
| **Cost** | ~$55 |

#### Performance Analysis

**CPU Performance**:
- Single-threaded: ~500-800 MIPS
- **Our load**: 12-50 ms per timestep
- **Capacity**: 10-83 timesteps/second
- **Usage**: 1.2-10% (1 Hz), 6-50% (5 Hz)

**Memory**:
- OS: ~100 MB
- Application: ~1 MB
- **Headroom**: ~410 MB

**Power Consumption**:
- Idle: ~200 mW
- Active (our load): ~500-1000 mW
- **Battery life**: ~2-4 hours (2000 mAh battery)

#### Pros

[x] **Full Linux OS**  
[x] **Python support**  
[x] **PRU (Programmable Real-time Units)** - for real-time tasks  
[x] **Lots of GPIO** (92 pins)  
[x] **Ethernet** (reliable)  

#### Cons

[ ] **Expensive** ($55)  
[ ] **Higher power** (~2.5W)  
[ ] **No built-in WiFi** (needs USB dongle)  
[ ] **Single-core** (less parallel processing)  
[ ] **Larger size** (might not fit micromouse)  
[ ] **Less popular** (smaller community)  

#### Suitability Score: **5.5/10**

**Best for**: Projects needing real-time PRU capabilities, but Pi Zero 2 W is better overall.

---

## 3. Comparison Matrix

| Metric | Pi Zero 2 W | Pi Pico W | Pi 4B | ESP32 | BeagleBone |
|--------|-------------|-----------|-------|-------|------------|
| **Cost** | $15 | $6 | $55 | $5-8 | $55 |
| **CPU Performance** | **** | ** | ***** | *** | *** |
| **Memory** | 512 MB | 264 KB | 2-8 GB | 520 KB | 512 MB |
| **Power** | 1W | 0.1W | 6-12W | 0.3-0.8W | 2.5W |
| **Linux OS** | [x] | [ ] | [x] | [ ] | [x] |
| **Python Support** | [x] | [!] | [x] | [!] | [x] |
| **WiFi Built-in** | [x] | [x] | [x] | [x] | [ ] |
| **Development Ease** | ***** | ** | ***** | ** | **** |
| **Community Support** | ***** | *** | ***** | **** | ** |
| **Size** | Small | Tiny | Large | Tiny | Medium |
| **Battery Life** | 10-15h | 20-40h | 2-4h | 4-8h | 2-4h |
| **Suitability** | **9.5/10** | 6.5/10 | 5/10 | 6/10 | 5.5/10 |

---

## 4. Cost Analysis (6 Robots)

| Option | Cost per Robot | Total Cost (6 robots) | Notes |
|-------|----------------|----------------------|-------|
| **Pi Zero 2 W** | $15 | **$90** | Recommended |
| **Pi Pico W** | $6 | **$36** | Budget, but tight |
| **Pi 4B** | $55 | **$330** | Overkill |
| **ESP32** | $5-8 | **$30-48** | Requires porting |
| **BeagleBone** | $55 | **$330** | Not recommended |

**Additional costs** (same for all):
- Sensors: 2-4* VL53L0X ($5 each) = $10-20
- I2C Multiplexer: TCA9548A ($3)
- Motor Driver: TB6612FNG ($5)
- Motors: 2* N20 ($10)
- **Per robot**: $28-38
- **6 robots**: $168-228

**Total system cost**:
- **Pi Zero 2 W**: $90 + $168-228 = **$258-318**
- **Pi Pico W**: $36 + $168-228 = **$204-264**
- **Pi 4B**: $330 + $168-228 = **$498-558**
- **ESP32**: $30-48 + $168-228 = **$198-276**

---

## 5. Power Analysis

### Battery Life (2000 mAh @ 3.7V = 7.4 Wh)

| Option | Power | Battery Life | Notes |
|--------|-------|--------------|-------|
| **Pi Zero 2 W** | 1W | **10-15 hours** | Good |
| **Pi Pico W** | 0.1W | **20-40 hours** | Excellent |
| **Pi 4B** | 6-12W | **0.6-1.2 hours** | [!] Poor |
| **ESP32** | 0.3-0.8W | **9-25 hours** | Good |
| **BeagleBone** | 2.5W | **3 hours** | [!] Poor |

**For 6 robots** (all active):
- **Pi Zero 2 W**: 6W total
- **Pi Pico W**: 0.6W total
- **Pi 4B**: 36-72W total [!]
- **ESP32**: 1.8-4.8W total
- **BeagleBone**: 15W total

---

## 6. Development Considerations

### Software Ecosystem

| Option | OS | Python | Libraries | Community | Development Time |
|--------|----|--------|-----------|-----------|------------------|
| **Pi Zero 2 W** | Linux | [x] Full | [x] All | ***** | **1-2 weeks** |
| **Pi Pico W** | Bare metal | [!] MicroPython | [!] Limited | *** | **4-6 weeks** |
| **Pi 4B** | Linux | [x] Full | [x] All | ***** | **1-2 weeks** |
| **ESP32** | FreeRTOS | [!] MicroPython | [!] Limited | **** | **6-8 weeks** |
| **BeagleBone** | Linux | [x] Full | [x] Most | ** | **2-3 weeks** |

**Key Insight**: Pi Zero 2 W and Pi 4B have the best software support, making development fastest.

---

## 7. Real-Time Performance

### Control Loop Timing (1 Hz = 1000 ms per timestep)

| Option | Computation Time | Headroom | Real-Time Capable? |
|--------|-----------------|----------|---------------------|
| **Pi Zero 2 W** | 12-50 ms | 950-988 ms (95-98.8%) | [x] **Yes** |
| **Pi Pico W** | 12-50 ms | 950-988 ms (95-98.8%) | [!] **Maybe** (at 1 Hz) |
| **Pi 4B** | 12-50 ms | 950-988 ms (95-98.8%) | [x] **Yes** |
| **ESP32** | 12-50 ms | 950-988 ms (95-98.8%) | [!] **Maybe** (test first) |
| **BeagleBone** | 12-50 ms | 950-988 ms (95-98.8%) | [x] **Yes** |

**At 5 Hz** (200 ms per timestep):
- **Pi Zero 2 W**: 12-50 ms = 6-25% usage [x]
- **Pi Pico W**: 12-50 ms = 6-25% usage [!] (might be tight)
- **Pi 4B**: 12-50 ms = 6-25% usage [x]
- **ESP32**: 12-50 ms = 6-25% usage [!] (test first)
- **BeagleBone**: 12-50 ms = 6-25% usage [x]

---

## 8. Recommendations by Use Case

### Use Case 1: ISEF Science Fair (Recommended)

**Option**: **Raspberry Pi Zero 2 W**

**Why**:
- [x] Best balance of performance, cost, and development time
- [x] Full Linux/Python support (easy to demonstrate)
- [x] Sufficient headroom for reliable operation
- [x] Good battery life (10-15 hours)
- [x] Large community (easy to get help)

**Total Cost**: ~$258-318 for 6 robots

---

### Use Case 2: Budget-Constrained Project

**Option**: **Raspberry Pi Pico W** (followers) + **Pi Zero 2 W** (leader)

**Why**:
- [x] Lower cost ($6 * 5 + $15 = $45 vs $90)
- [x] Leader gets more compute (frontier detection/assignment)
- [x] Followers still sufficient (MPC trust is lightweight)
- [!] Requires testing and optimization

**Total Cost**: ~$204-264 for 6 robots

---

### Use Case 3: Future Expansion (Camera, ML)

**Option**: **Raspberry Pi 4B**

**Why**:
- [x] Extremely powerful (can add camera, ML features)
- [x] Lots of RAM (2-8 GB)
- [!] Expensive ($330 vs $90)
- [!] High power (needs larger battery)
- [!] Large size (might not fit micromouse)

**Total Cost**: ~$498-558 for 6 robots

---

### Use Case 4: Ultra-Low Power (Long Battery Life)

**Option**: **Raspberry Pi Pico W**

**Why**:
- [x] Very low power (0.1W - 20-40 hour battery life)
- [x] Very low cost ($6)
- [!] Limited CPU (might be tight)
- [!] No Linux (harder development)

**Total Cost**: ~$204-264 for 6 robots

---

## 9. Final Recommendation

### Primary Recommendation: **Raspberry Pi Zero 2 W** *

**Score**: 9.5/10

**Reasons**:
1. [x] **Best performance/price ratio**
2. [x] **Full Linux/Python support** (easy development)
3. [x] **Sufficient headroom** (5-20x capacity)
4. [x] **Good battery life** (10-15 hours)
5. [x] **Large community** (lots of support)
6. [x] **Built-in WiFi** (no additional module)
7. [x] **Small form factor** (fits micromouse)

**Total System Cost**: ~$258-318

---

### Alternative: **Hybrid Approach** (Budget Option)

**Leader**: Raspberry Pi Zero 2 W ($15)  
**Followers**: Raspberry Pi Pico W ($6 * 5 = $30)

**Total**: $45 (vs $90 for all Pi Zero 2 W)

**Pros**:
- [x] Lower cost ($45 vs $90)
- [x] Leader has more compute (needed for frontier detection)
- [x] Followers sufficient (MPC trust is lightweight)

**Cons**:
- [!] Requires testing (Pico W might be tight)
- [!] Two different platforms (more complexity)

**Recommendation**: Test Pico W first, use Pi Zero 2 W if tight.

---

## 10. Decision Matrix

| Criteria | Weight | Pi Zero 2 W | Pi Pico W | Pi 4B | ESP32 | BeagleBone |
|----------|--------|-------------|-----------|-------|-------|------------|
| **Cost** | 20% | 8/10 | 10/10 | 2/10 | 10/10 | 2/10 |
| **Performance** | 25% | 9/10 | 5/10 | 10/10 | 6/10 | 7/10 |
| **Development Ease** | 20% | 10/10 | 4/10 | 10/10 | 4/10 | 7/10 |
| **Power Efficiency** | 15% | 8/10 | 10/10 | 2/10 | 9/10 | 5/10 |
| **Community Support** | 10% | 10/10 | 7/10 | 10/10 | 8/10 | 5/10 |
| **Size** | 10% | 9/10 | 10/10 | 5/10 | 10/10 | 6/10 |
| **Weighted Score** | 100% | **8.9/10** | 7.1/10 | 6.5/10 | 7.4/10 | 5.3/10 |

**Winner**: **Raspberry Pi Zero 2 W** (8.9/10)

---

## 11. Next Steps

1. **Order 1* Pi Zero 2 W** for testing
2. **Profile actual CPU usage** on hardware
3. **Test at 1 Hz and 5 Hz** control loops
4. **Measure power consumption** (verify battery life)
5. **If budget is tight**: Test Pi Pico W as alternative
6. **Order remaining 5*** once validated

---

## 12. Conclusion

**Recommended Hardware**: **Raspberry Pi Zero 2 W**

**Why**: Best balance of performance, cost, development ease, and power efficiency for the REIP system with MPC trust.

**Total System Cost**: ~$258-318 for 6 robots

**Confidence Level**: **95%+** - Pi Zero 2 W is the clear winner for this application.
