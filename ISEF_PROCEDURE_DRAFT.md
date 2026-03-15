# ISEF Procedure Section - Streamlined Version

## Hardware Platform

I assembled five differential-drive robots, each consisting of:
- Raspberry Pi Zero 2W (onboard computation)
- Raspberry Pi Pico H (motor control via UART)
- Five VL53L0X time-of-flight sensors (obstacle detection)
- Two N20 geared DC motors with encoders (1:100 ratio)
- 2S 1000 mAh LiPo battery
- 3D-printed PLA chassis (under 300g per robot)

Each robot is color-coded and carries an ArUco identification tag (IDs 1-5, 4×4 library, 50mm). Robots communicate via WiFi peer-to-peer and are named clanker1-5.

## Experimental Arena

A 2000 × 1500 mm arena was constructed from 38 mm EPS foamboard, covered with masking tape to reduce ToF sensor reflections. A central divider (1000 mm from left edge, 1300 mm tall) creates a two-room layout. Four ArUco corner markers (IDs 40-43, 80mm) enable camera-based localization.

## Data Collection

A Logitech C922 webcam positioned 2042 mm above the arena center provides overhead imaging. An OpenCV-based script running on a laptop performs:
- Real-time homography transformation
- Per-robot pose tracking (ArUco-based localization)
- Live visualization overlay (coverage, robot positions, leader status, velocity vectors)

The script broadcasts each robot's position via UDP to that robot only (no cross-robot position sharing).

## Experimental Protocol

[Add your actual experimental procedure here: number of trials, conditions tested, fault injection methodology, etc.]

---

# Original Version (Too Detailed for ISEF)

Your original version is **too technical and verbose** for ISEF judges. Issues:

1. **Excessive component detail**: Listing every part number (e.g., "Pololu D30V30F5 5V step down regulator") is unnecessary for judges
2. **Missing experimental procedure**: You describe the setup but not what you actually DO (trials, conditions, data collection)
3. **Too much technical jargon**: "proprietary code comment system" is unnecessary detail
4. **Exact measurements everywhere**: Some precision is good, but 2042.16 mm is excessive

**What ISEF judges need:**
- Clear understanding of your methodology
- Reproducibility (but not every component spec)
- What experiments you ran, not just how you built the robots
- Appropriate technical level (not too simple, not too detailed)

**Recommendation**: Use the streamlined version above, then add:
- Number of trials per condition
- Conditions tested (clean, bad-leader, freeze-leader, etc.)
- How faults are injected
- How data is collected and analyzed
- Duration of trials
- Success/failure criteria
