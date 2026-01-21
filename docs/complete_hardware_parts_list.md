# Complete Hardware Parts List: REIP Micromouse Robot

## Overview

This document provides a complete parts list for building a REIP micromouse robot using perfboards (perforated prototyping boards). All parts are selected for:
- **Cost-effectiveness**
- **Availability** (common suppliers)
- **Compatibility** with Raspberry Pi Zero 2 W
- **Micromouse form factor** (compact, lightweight)

---

## 1. Main Controller & Power

### Primary Controller

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **Raspberry Pi Zero 2 W** | 1 | RPI-ZERO-2-W | Adafruit, SparkFun, DigiKey | $15 | Main controller, includes WiFi |
| **MicroSD Card** | 1 | 16GB Class 10 | Any | $5 | For OS and code storage |
| **MicroSD to USB Adapter** | 1 | Generic | Any | $2 | For flashing OS (one-time use) |

**Subtotal**: $22

### Power Management

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **LiPo Battery** | 1 | 2000mAh 3.7V | Adafruit #1578, SparkFun PRT-13855 | $10 | Main power source |
| **USB-C Power Bank Module** | 1 | TP4056 | Adafruit #259, SparkFun PRT-14411 | $3 | Charging circuit |
| **5V Boost Converter** | 1 | MT3608 | Adafruit #4654, SparkFun PRT-14411 | $2 | 3.7V → 5V for Pi Zero |
| **Power Switch** | 1 | SPST Toggle | Any | $1 | On/off switch |
| **JST-PH Connector** | 2 | JST-PH 2-pin | Adafruit #261, SparkFun PRT-09315 | $2 | Battery connector |

**Subtotal**: $18

**Power Notes**:
- Pi Zero 2 W needs 5V @ 200mA minimum (1W)
- 2000mAh battery provides ~10-15 hours runtime
- TP4056 provides safe charging with overcharge protection
- MT3608 boost converter steps up 3.7V to 5V

---

## 2. Sensors (ToF Distance)

### Time-of-Flight Sensors

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **VL53L0X ToF Sensor** | 4 per robot | VL53L0X | Amazon (see below) | $2.80 each | Primary distance sensor |
| **I2C Multiplexer** | 1 | TCA9548A | Adafruit #2717, SparkFun COM-16784 | $3 | Allows 4 sensors on one I2C bus |
| **Breakout Board** | Included | VL53L0X breakout | Amazon (pre-soldered) | Included | Pre-soldered, easy to use |

**Subtotal**: $14.20 (4 sensors @ $2.80 each + 1 multiplexer @ $3)

**Amazon Options (Recommended)**:
- **ACEIRMC 5-pack**: $13.99 for 5 sensors = **$2.80 per sensor** ⭐ **BEST VALUE**
  - Search: "ACEIRMC VL53L0X 5pcs Black"
  - 4.4/5 stars, 54 reviews
  - **For 6 robots**: Buy 5 packs (25 sensors) = $69.95 total
- **Starry 5-pack**: $16.14 for 5 sensors = $3.23 per sensor
  - Search: "Starry VL53L0X 5pcs"
  - 5.0/5 stars, 2 reviews
- **HiLetgo single**: $6.79 per sensor (not recommended - too expensive)
  - Only buy if you need just 1-2 sensors

**Bulk Purchase for 6 Robots**:
- **Total sensors needed**: 24 (4 per robot × 6 robots)
- **Recommended**: Buy 5× ACEIRMC 5-packs = 25 sensors (1 extra spare)
- **Total cost**: $69.95 (vs $162.96 if buying singles)
- **Savings**: $92.96 (57% discount!)

**Sensor Configuration**:
- **Front**: 1× VL53L0X (forward detection)
- **Left**: 1× VL53L0X (side wall detection)
- **Right**: 1× VL53L0X (side wall detection)
- **Back**: 1× VL53L0X (optional, for reverse detection)

**Alternative (Budget)**: Use 2-3 sensors (front + left + right) = $10-15

---

## 3. Motors & Motor Control

### Motors

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **N20 Micro Gear Motor** | 2 | N20 6V 100RPM | Pololu #3052, Amazon | $5 each | Main drive motors |
| **Motor Encoder** | 2 | N20 with encoder | Pololu #3052, Amazon | Included | Quadrature encoder for odometry |
| **Motor Mounts** | 2 | N20 mount | Pololu #2734, 3D print | $2 | Secure motors to chassis |

**Subtotal**: $12

### Motor Driver

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **Dual Motor Driver** | 1 | TB6612FNG | Adafruit #2448, SparkFun ROB-14450 | $5 | H-bridge driver for 2 motors |
| **Heat Sink** | 1 | Small TO-220 | Any | $1 | Optional, for high current |

**Subtotal**: $6

**Motor Specifications**:
- **Voltage**: 6V (can run on 5V)
- **Speed**: 100 RPM (adjustable with PWM)
- **Torque**: ~0.5 kg-cm (sufficient for micromouse)
- **Encoder**: 12 CPR (counts per revolution)

---

## 4. Chassis & Mechanical

### Base Structure

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **Perfboard** | 1 | 5cm × 5cm | Adafruit #2670, SparkFun PRT-12070 | $2 | Main chassis base |
| **Standoffs** | 4-6 | M3 × 10mm | Any | $2 | Mount Pi Zero above board |
| **Screws** | 10-15 | M3 × 6mm | Any | $1 | Assembly screws |
| **Wheels** | 2 | 32mm diameter | Pololu #1084, Amazon | $3 | Drive wheels |
| **Caster Ball** | 1 | 15mm ball caster | Pololu #953, Amazon | $2 | Front/rear support |

**Subtotal**: $10

**Chassis Notes**:
- Perfboard allows easy soldering and component mounting
- Standoffs create space for wiring underneath
- Wheels should match motor shaft diameter (3mm or 6mm)
- Caster provides stability (can use simple ball bearing)

---

## 5. Electronics & Interfacing

### GPIO & Interfacing

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **GPIO Header** | 1 | 40-pin female | Adafruit #2222, SparkFun PRT-12953 | $2 | Connect to Pi Zero GPIO |
| **Breadboard** | 1 | Mini breadboard | Adafruit #65, SparkFun PRT-12045 | $2 | Prototyping (optional) |
| **Jumper Wires** | 20 | M-M, M-F | Adafruit #1957, SparkFun PRT-11026 | $3 | Connections |
| **Resistors** | 10 | 10kΩ, 4.7kΩ | Any | $1 | Pull-up resistors for I2C |
| **Capacitors** | 5 | 100µF, 10µF | Any | $1 | Power filtering |

**Subtotal**: $9

### I2C Pull-ups

| Part | Quantity | Part Number | Supplier | Cost | Notes |
|------|----------|-------------|----------|------|-------|
| **Pull-up Resistors** | 4 | 4.7kΩ | Any | $1 | I2C bus pull-ups (on multiplexer) |

**Subtotal**: $1

---

## 6. Communication (WiFi Built-in)

### WiFi (Built-in to Pi Zero 2 W)

✅ **No additional parts needed** - Pi Zero 2 W has built-in WiFi 802.11n

**Optional**: External antenna for better range (if needed)
- **Antenna**: 2.4GHz external antenna ($3)

---

## 7. Assembly Hardware

### Fasteners & Hardware

| Part | Quantity | Notes |
|------|----------|-------|
| **M3 Screws** | 20 | Various lengths (6mm, 10mm, 15mm) |
| **M3 Nuts** | 20 | Matching screws |
| **M3 Washers** | 10 | For secure mounting |
| **Double-sided Tape** | 1 roll | Mounting sensors, small components |
| **Heat Shrink** | 1 pack | Insulate connections |
| **Wire** | 1 roll | 22-24 AWG, various colors |

**Subtotal**: $5

---

## 8. Tools & Supplies

### Essential Tools

| Tool | Purpose | Cost | Notes |
|------|---------|------|-------|
| **Soldering Iron** | Component soldering | $20-30 | Temperature-controlled recommended |
| **Solder** | Connections | $5 | Lead-free, 0.8mm diameter |
| **Wire Strippers** | Wire preparation | $10 | |
| **Multimeter** | Testing circuits | $15 | Verify connections, check voltage |
| **Screwdriver Set** | Assembly | $10 | Phillips, flathead, hex |
| **Helping Hands** | Soldering aid | $10 | Hold components while soldering |

**Subtotal**: $80 (one-time purchase, shared across all robots)

**Per Robot Tool Cost**: $80 / 6 = **$13.33**

---

## 9. Complete Parts List Summary

### Per Robot Cost Breakdown

| Category | Cost | Notes |
|----------|------|-------|
| **Main Controller** | $22 | Pi Zero 2 W + MicroSD |
| **Power Management** | $18 | Battery, charging, boost converter |
| **Sensors** | $14.20 | 4× VL53L0X + multiplexer (Amazon bulk pricing) |
| **Motors** | $12 | 2× N20 motors with encoders |
| **Motor Driver** | $6 | TB6612FNG |
| **Chassis** | $10 | Perfboard, wheels, caster |
| **Electronics** | $9 | GPIO, wires, resistors |
| **Assembly Hardware** | $5 | Screws, nuts, wire |
| **Tools (1/6 share)** | $13 | Soldering iron, multimeter, etc. |
| **Subtotal** | **$109.20** | Per robot (with Amazon sensor pricing) |
| **Contingency (10%)** | $11 | Spare parts, mistakes |
| **Total** | **$120.20** | Per robot |

### For 6 Robots

| Item | Cost |
|------|------|
| **6× Robots** | $655.20 ($109.20 × 6) |
| **Tools (one-time)** | $80 |
| **Contingency** | $66 ($11 × 6) |
| **Total** | **$801.20** |

**Note**: Sensors bought in bulk (5× 5-packs = 25 sensors for $69.95) saves $92.96 vs buying singles!

---

## 10. Part Numbers & Suppliers

### Recommended Suppliers

1. **Adafruit** (adafruit.com)
   - Excellent documentation
   - Pre-soldered breakouts
   - Good for beginners

2. **SparkFun** (sparkfun.com)
   - Similar to Adafruit
   - Good tutorials
   - Competitive pricing

3. **Pololu** (pololu.com)
   - Best for motors
   - Good quality
   - Fast shipping

4. **DigiKey** (digikey.com)
   - Largest selection
   - Best for bulk
   - Professional components

5. **Amazon**
   - Convenient
   - Often cheaper
   - Check reviews carefully

### Specific Part Numbers

#### Main Controller
- **Raspberry Pi Zero 2 W**: Adafruit #5851, SparkFun DEV-17745
- **MicroSD 16GB**: Any Class 10 card

#### Power
- **2000mAh LiPo**: Adafruit #1578, SparkFun PRT-13855
- **TP4056 Charger**: Adafruit #259, SparkFun PRT-14411
- **MT3608 Boost**: Adafruit #4654, Amazon (search "MT3608 boost")

#### Sensors
- **VL53L0X**: Adafruit #3317, SparkFun SEN-14786
- **TCA9548A Multiplexer**: Adafruit #2717, SparkFun COM-16784

#### Motors
- **N20 Motor with Encoder**: Pololu #3052, Amazon (search "N20 encoder motor")
- **TB6612FNG Driver**: Adafruit #2448, SparkFun ROB-14450

#### Chassis
- **Perfboard**: Adafruit #2670 (5cm × 5cm), SparkFun PRT-12070
- **Wheels**: Pololu #1084 (32mm), Amazon (search "32mm robot wheels")
- **Caster**: Pololu #953, Amazon (search "ball caster 15mm")

---

## 11. Assembly Guide Overview

### Step 1: Power System
1. Solder TP4056 charging module to perfboard
2. Solder MT3608 boost converter (3.7V → 5V)
3. Connect battery via JST-PH connector
4. Add power switch between battery and boost converter
5. Test: Verify 5V output before connecting Pi Zero

### Step 2: Main Controller
1. Flash Raspberry Pi OS to MicroSD card
2. Configure WiFi and enable SSH
3. Mount Pi Zero 2 W on standoffs above perfboard
4. Connect 5V power from boost converter to Pi Zero (pin 2 or 4)
5. Connect ground (pin 6 or any ground)

### Step 3: Sensors
1. Mount 4× VL53L0X sensors (front, left, right, back)
2. Connect all sensors to TCA9548A multiplexer
3. Connect multiplexer to Pi Zero I2C (SDA=pin 3, SCL=pin 5)
4. Add 4.7kΩ pull-up resistors on I2C bus
5. Test: Use `i2cdetect` to verify all sensors

### Step 4: Motors
1. Mount 2× N20 motors on chassis
2. Attach wheels to motor shafts
3. Connect motors to TB6612FNG driver
4. Connect driver to Pi Zero GPIO:
   - PWM pins for speed control
   - Direction pins for forward/reverse
   - Encoder pins for odometry
5. Test: Run motors at low speed

### Step 5: Chassis Assembly
1. Mount caster ball for stability
2. Secure all components with screws
3. Route wires neatly (use zip ties)
4. Insulate connections with heat shrink
5. Final check: All connections secure, no shorts

---

## 12. Wiring Diagram Summary

### Power Connections
```
Battery (3.7V) → TP4056 Charger → MT3608 Boost (5V) → Pi Zero
```

### I2C Bus (Sensors)
```
Pi Zero (SDA/SCL) → TCA9548A Multiplexer → 4× VL53L0X Sensors
```

### Motor Control
```
Pi Zero GPIO → TB6612FNG Driver → 2× N20 Motors
Pi Zero GPIO → Motor Encoders (for odometry)
```

### GPIO Pin Assignments (Suggested)

| Function | Pi Zero Pin | Notes |
|----------|-------------|-------|
| **I2C SDA** | Pin 3 (GPIO 2) | Sensors |
| **I2C SCL** | Pin 5 (GPIO 3) | Sensors |
| **Motor 1 PWM** | Pin 12 (GPIO 18) | Left motor speed |
| **Motor 1 Dir** | Pin 16 (GPIO 23) | Left motor direction |
| **Motor 2 PWM** | Pin 13 (GPIO 27) | Right motor speed |
| **Motor 2 Dir** | Pin 15 (GPIO 22) | Right motor direction |
| **Encoder 1A** | Pin 18 (GPIO 24) | Left encoder |
| **Encoder 1B** | Pin 22 (GPIO 25) | Left encoder |
| **Encoder 2A** | Pin 24 (GPIO 8) | Right encoder |
| **Encoder 2B** | Pin 26 (GPIO 7) | Right encoder |
| **5V Power** | Pin 2 or 4 | From boost converter |
| **Ground** | Pin 6 or any GND | Common ground |

---

## 13. Budget Options

### Option A: Minimal Sensors (3 instead of 4)
- **Savings**: $2.80 (1× VL53L0X at bulk price)
- **Total**: $117.40 per robot

### Option B: Smaller Battery (1000mAh)
- **Savings**: $5
- **Trade-off**: 5-7 hour runtime (vs 10-15 hours)
- **Total**: $125 per robot

### Option C: Skip Caster (Use Simple Bearing)
- **Savings**: $2
- **Trade-off**: Less smooth movement
- **Total**: $128 per robot

### Option D: All Budget Options
- **Total**: $109.20 per robot
- **6 Robots**: $655.20 + $80 tools = **$735.20 total**

---

## 14. Where to Buy

### Recommended Order

1. **Adafruit** or **SparkFun**: Main components (Pi Zero, sensors, motor driver)
2. **Pololu**: Motors and wheels
3. **DigiKey** or **Mouser**: Passive components (resistors, capacitors)
4. **Amazon**: Tools, screws, wire (often cheaper)

### Buying Tips

- **Buy in bulk**: Some suppliers offer discounts for 6+ units
- **Check shipping**: Combine orders to save on shipping
- **Compare prices**: Same part can vary significantly
- **Read reviews**: Especially on Amazon
- **Buy spares**: Get 1-2 extra sensors/motors for testing

---

## 15. Next Steps

1. **Order 1× complete set** for prototyping
2. **Build and test** first robot
3. **Verify all components work** before ordering 5 more
4. **Order remaining 5×** once validated
5. **Build all 6 robots** in parallel

---

## 16. Additional Resources

### Documentation
- **Raspberry Pi Zero 2 W**: https://www.raspberrypi.com/documentation/
- **VL53L0X**: Adafruit guide #3317
- **TB6612FNG**: Adafruit guide #2448
- **TCA9548A**: Adafruit guide #2717

### Software
- **Raspberry Pi OS**: Download from raspberrypi.com
- **Python libraries**: `adafruit-circuitpython-vl53l0x`, `adafruit-circuitpython-tca9548a`

### Community
- **r/raspberry_pi**: Reddit community
- **Adafruit Forums**: Component-specific help
- **Pololu Forums**: Motor/mechanical help

---

## 17. Final Checklist

Before ordering, verify:
- [ ] All part numbers match your suppliers
- [ ] Quantities are correct (6 robots)
- [ ] Tools are accounted for (one-time purchase)
- [ ] Budget includes contingency (10%)
- [ ] You have soldering experience (or plan to learn)
- [ ] You have a multimeter for testing

**Total Budget**: **$801.20 for 6 complete robots** (with Amazon bulk sensor pricing)

**Savings**: $58.80 vs original estimate (7% reduction)

---

**Good luck with your build!** 🚀
