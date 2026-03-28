#!/usr/bin/env python3
"""
Generate a clean block-diagram wiring schematic for one REIP robot.
Output: paper_docs/Ryker_Kollmyer___UPDATED_PAPER/circuit_schematic.png
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

plt.rcParams.update({'font.family': 'serif', 'font.size': 8})

fig, ax = plt.subplots(figsize=(7.16, 7.5), dpi=300)
ax.set_xlim(-2, 18)
ax.set_ylim(-2, 13)
ax.set_aspect('equal')
ax.axis('off')

# -- Colors ------------------------------------------------------------
C = dict(
    pizero='#B3CDE3', pico='#CCEBC5', drv='#FBB4AE', tof='#DECBE4',
    mux='#FED9A6', motor='#E5D8BD', batt='#FFFFCC',
    uart='#1B7837', i2c='#5E3C99', pwm='#E66101', pwr='#D62728',
    enc='#555555', gnd='#888888',
)


def block(x, y, w, h, title, sub, color, fs=9, sfs=6.5):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.12",
        facecolor=color, edgecolor='#333', lw=1.0, zorder=2))
    ax.text(x + w / 2, y + h * 0.62, title,
            ha='center', va='center', fontsize=fs, fontweight='bold', zorder=3)
    if sub:
        ax.text(x + w / 2, y + h * 0.30, sub,
                ha='center', va='center', fontsize=sfs,
                color='#555', style='italic', zorder=3)


def wire(pts, color='#333', lw=0.9, ls='-'):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=3,
            solid_capstyle='round')


def arrw(x1, y1, x2, y2, color='#333', lw=0.9):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw), zorder=4)


def bus(x, y, text, color, fs=6, rot=0):
    ax.text(x, y, text, ha='center', va='center', fontsize=fs,
            color=color, fontweight='bold', rotation=rot,
            bbox=dict(boxstyle='round,pad=0.15', fc='white',
                      ec=color, lw=0.6), zorder=6)


def pin_text(x, y, text, ha='left', fs=5.5, color='#333'):
    ax.text(x, y, text, ha=ha, va='center', fontsize=fs,
            fontfamily='monospace', color=color, zorder=5)


# =======================================================================
#  LAYOUT
# =======================================================================

# Row 1: Battery (top center)
batt_x, batt_y = 5.5, 11.0
block(batt_x, batt_y, 3.5, 1.2, 'Battery Pack', '4\u00D7AA  (6 V)', C['batt'])

# Row 2: Pi Zero 2W
pz_x, pz_y, pz_w, pz_h = 4.0, 7.5, 7.0, 2.5
block(pz_x, pz_y, pz_w, pz_h, 'Raspberry Pi Zero 2W',
      'reip_node.py  (robot brain)', C['pizero'], fs=11, sfs=7)

# Row 3: Pi Pico
pc_x, pc_y, pc_w, pc_h = 4.0, 2.5, 7.0, 2.5
block(pc_x, pc_y, pc_w, pc_h, 'Raspberry Pi Pico',
      'main.py  (MicroPython)', C['pico'], fs=11, sfs=7)

# Right col: DRV8833 (custom layout for pin space)
drv_x, drv_y, drv_w, drv_h = 14.0, 3.8, 2.8, 3.2
ax.add_patch(FancyBboxPatch(
    (drv_x, drv_y), drv_w, drv_h, boxstyle="round,pad=0.12",
    facecolor=C['drv'], edgecolor='#333', lw=1.0, zorder=2))
ax.text(drv_x + drv_w / 2, drv_y + drv_h - 0.4, 'DRV8833',
        ha='center', va='center', fontsize=9, fontweight='bold', zorder=3)
ax.text(drv_x + drv_w / 2, drv_y + drv_h - 0.8, 'Dual H-Bridge',
        ha='center', va='center', fontsize=6.5, color='#555',
        style='italic', zorder=3)

# Left col: TCA9548A
mux_x, mux_y, mux_w, mux_h = -0.5, 6.2, 2.8, 1.5
block(mux_x, mux_y, mux_w, mux_h, 'TCA9548A', 'I\u00B2C Mux  (0x70)',
      C['mux'], fs=8.5, sfs=6)

# Bottom-left: Left Motor
ml_x, ml_y = -0.5, -0.5
block(ml_x, ml_y, 2.8, 1.5, 'Left Motor', 'N20 DC + Encoder', C['motor'])

# Bottom-right: Right Motor
mr_x, mr_y = 14.0, -0.5
block(mr_x, mr_y, 2.8, 1.5, 'Right Motor', 'N20 DC + Encoder', C['motor'])

# Far-left: ToF sensors stacked
tof_names = ['Right', 'Front-R', 'Front', 'Front-L', 'Left']
tof_chs   = ['Ch 0', 'Ch 1', 'Ch 2', 'Ch 3', 'Ch 4']
tof_base = 2.0
for i, (nm, ch) in enumerate(zip(tof_names, tof_chs)):
    ty = tof_base + i * 0.75
    ax.add_patch(FancyBboxPatch(
        (-1.8, ty), 1.6, 0.6, boxstyle="round,pad=0.05",
        fc=C['tof'], ec='#555', lw=0.5, zorder=2))
    ax.text(-1.0, ty + 0.30, nm, ha='center', va='center',
            fontsize=5.5, fontweight='bold', zorder=3)
    ax.text(-0.05, ty + 0.30, ch, ha='left', va='center',
            fontsize=4.5, color='#666', fontfamily='monospace', zorder=3)

ax.text(-2.0, tof_base + 1.8, 'VL53L0X\n\u00D75', ha='center', va='center',
        fontsize=7, fontweight='bold', color='#555', rotation=90)

# =======================================================================
#  PIN LABELS
# =======================================================================

pzl = pz_x - 0.15
pin_text(pzl, pz_y + 2.05, 'GPIO14 (TX)', ha='right')
pin_text(pzl, pz_y + 1.65, 'GPIO15 (RX)', ha='right')
pin_text(pzl, pz_y + 0.85, 'GPIO2 (SDA)', ha='right')
pin_text(pzl, pz_y + 0.45, 'GPIO3 (SCL)', ha='right')

pzr = pz_x + pz_w + 0.15
pin_text(pzr, pz_y + 1.80, '5 V in', ha='left', color=C['pwr'])
pin_text(pzr, pz_y + 0.60, 'GND', ha='left', color=C['gnd'])

pcl = pc_x - 0.15
pin_text(pcl, pc_y + 2.05, 'GP1  (RX)', ha='right')
pin_text(pcl, pc_y + 1.65, 'GP0  (TX)', ha='right')
pin_text(pcl, pc_y + 0.85, 'GP3  (Enc L-A)', ha='right')
pin_text(pcl, pc_y + 0.45, 'GP4  (Enc L-B)', ha='right')

pcr = pc_x + pc_w + 0.15
pin_text(pcr, pc_y + 2.15, 'GP18 \u2192 AIN1', ha='left', color=C['pwm'])
pin_text(pcr, pc_y + 1.85, 'GP19 \u2192 AIN2', ha='left', color=C['pwm'])
pin_text(pcr, pc_y + 1.50, 'GP20 \u2192 BIN1', ha='left', color=C['pwm'])
pin_text(pcr, pc_y + 1.15, 'GP21 \u2192 BIN2', ha='left', color=C['pwm'])
pin_text(pcr, pc_y + 0.60, 'GP7  (Enc R-A)', ha='left', color=C['enc'])
pin_text(pcr, pc_y + 0.25, 'GP8  (Enc R-B)', ha='left', color=C['enc'])
pin_text(pc_x + 0.4, pc_y - 0.35, 'VSYS (5 V)', ha='left', fs=5.5, color=C['pwr'])

# DRV inside pin labels
for i, lbl in enumerate(['AIN1', 'AIN2', 'BIN1', 'BIN2']):
    pin_text(drv_x + 0.2, drv_y + 1.55 - i * 0.35, lbl, ha='left', fs=5, color='#444')

# =======================================================================
#  WIRES
# =======================================================================

# UART
ux1, ux2 = pz_x - 0.8, pz_x - 1.3
wire([(pz_x, pz_y + 2.05), (ux1, pz_y + 2.05), (ux1, pc_y + 2.05)], C['uart'], 1.1)
arrw(ux1, pc_y + 2.05, pc_x, pc_y + 2.05, C['uart'], 1.1)
wire([(pc_x, pc_y + 1.65), (ux2, pc_y + 1.65), (ux2, pz_y + 1.65)], C['uart'], 1.1)
arrw(ux2, pz_y + 1.65, pz_x, pz_y + 1.65, C['uart'], 1.1)
bus(ux1 - 0.35, (pz_y + pc_y + 2.5) / 2, 'UART\n115 200\nbaud', C['uart'], fs=5.5)

# I2C
ix1, ix2 = 2.8, 3.2
wire([(pz_x, pz_y + 0.85), (ix1, pz_y + 0.85), (ix1, mux_y + 1.1)], C['i2c'], 1.0)
arrw(ix1, mux_y + 1.1, mux_x + mux_w, mux_y + 1.1, C['i2c'])
wire([(pz_x, pz_y + 0.45), (ix2, pz_y + 0.45), (ix2, mux_y + 0.5)], C['i2c'], 1.0)
arrw(ix2, mux_y + 0.5, mux_x + mux_w, mux_y + 0.5, C['i2c'])
bus(3.0, pz_y - 0.2, 'I\u00B2C\nBus 1', C['i2c'], fs=5.5)

# TCA9548A -> ToF
for i in range(5):
    ty = tof_base + i * 0.75 + 0.30
    my = mux_y + 0.2 + i * 0.22
    wire([(mux_x, my), (-0.05, ty)], C['i2c'], 0.5)

# PWM: Pico -> DRV8833
route_x = 13.2
pico_r = pc_x + pc_w
drv_l = drv_x
for py_off, dy_off in [(2.15, 1.55), (1.85, 1.20), (1.50, 0.85), (1.15, 0.50)]:
    wire([(pico_r, pc_y + py_off), (route_x, pc_y + py_off),
          (route_x, drv_y + dy_off)], C['pwm'], 0.9)
    arrw(route_x, drv_y + dy_off, drv_l, drv_y + dy_off, C['pwm'], 0.9)
bus(route_x, pc_y - 0.15, 'PWM\n1 kHz', C['pwm'], fs=5.5)

# DRV -> Left Motor
wire([(drv_l, drv_y + 1.20), (drv_l - 0.4, drv_y + 1.20),
      (drv_l - 0.4, 1.8), (ml_x + 2.8 + 0.4, 1.8),
      (ml_x + 2.8 + 0.4, ml_y + 1.0)], C['pwm'], 1.2)
arrw(ml_x + 2.8 + 0.4, ml_y + 1.0, ml_x + 2.8, ml_y + 1.0, C['pwm'], 1.2)
ax.text(7.0, 1.5, 'AOUT1/2', fontsize=5.5, color=C['pwm'], ha='center',
        bbox=dict(fc='white', ec='none', pad=1))

# DRV -> Right Motor
drv_cx = drv_x + drv_w / 2
wire([(drv_cx, drv_y), (drv_cx, mr_y + 1.5)], C['pwm'], 1.2)
ax.text(drv_cx + 0.3, (drv_y + mr_y + 1.5) / 2, 'BOUT1/2', fontsize=5.5,
        color=C['pwm'], ha='left')

# Left Encoder -> Pico
wire([(ml_x + 2.8, ml_y + 0.55), (3.4, ml_y + 0.55), (3.4, pc_y + 0.85)], C['enc'], 0.7, ls='--')
arrw(3.4, pc_y + 0.85, pc_x, pc_y + 0.85, C['enc'], 0.7)
wire([(ml_x + 2.8, ml_y + 0.25), (3.7, ml_y + 0.25), (3.7, pc_y + 0.45)], C['enc'], 0.7, ls='--')
arrw(3.7, pc_y + 0.45, pc_x, pc_y + 0.45, C['enc'], 0.7)
ax.text(3.0, 1.3, 'Enc L\n(A, B)', fontsize=5, color=C['enc'], ha='center')

# Right Encoder -> Pico
wire([(mr_x, mr_y + 0.55), (12.4, mr_y + 0.55), (12.4, pc_y + 0.60)], C['enc'], 0.7, ls='--')
arrw(12.4, pc_y + 0.60, pico_r, pc_y + 0.60, C['enc'], 0.7)
wire([(mr_x, mr_y + 0.25), (12.7, mr_y + 0.25), (12.7, pc_y + 0.25)], C['enc'], 0.7, ls='--')
arrw(12.7, pc_y + 0.25, pico_r, pc_y + 0.25, C['enc'], 0.7)
ax.text(13.2, 1.3, 'Enc R\n(A, B)', fontsize=5, color=C['enc'], ha='center')

# Power rails
batt_r = batt_x + 3.5
wire([(batt_r, batt_y + 0.6), (pz_x + pz_w + 0.6, batt_y + 0.6),
      (pz_x + pz_w + 0.6, pz_y + 1.80)], C['pwr'], 1.3, ls='--')
wire([(batt_x, batt_y + 0.6), (pz_x - 2.0, batt_y + 0.6),
      (pz_x - 2.0, pc_y - 0.15), (pc_x, pc_y - 0.15)], C['pwr'], 1.1, ls='--')
wire([(batt_r, batt_y + 0.3), (drv_cx, batt_y + 0.3),
      (drv_cx, drv_y + drv_h)], C['pwr'], 1.1, ls='--')
pin_text(drv_cx, drv_y + drv_h + 0.2, 'VM', ha='center', fs=5.5, color=C['pwr'])
bus(pz_x - 2.0, 9.5, '6 V\nrail', C['pwr'], fs=5.5)

# GND rail
gnd_y = -1.2
ax.plot([-1.8, 17.5], [gnd_y, gnd_y], color=C['gnd'], lw=2.0, ls=':', zorder=1)
ax.text(17.3, gnd_y - 0.3, 'GND (common rail)', fontsize=6, ha='right',
        color=C['gnd'], style='italic')
for gx in [0.9, 7.5, 15.4]:
    wire([(gx, -0.5), (gx, gnd_y)], C['gnd'], 1.0, ls=':')

# =======================================================================
#  ANNOTATIONS
# =======================================================================

ax.text(8.0, 12.8, 'REIP Robot \u2014 Wiring Schematic  (\u00D75 identical units)',
        ha='center', va='center', fontsize=13, fontweight='bold')

ax.annotate('WiFi (UDP)\nPorts 5100 / 5200 / 5300',
            xy=(pz_x + pz_w - 0.5, pz_y + pz_h),
            xytext=(pz_x + pz_w + 1.2, pz_y + pz_h + 1.0),
            fontsize=6, ha='center', color='#0571B0',
            arrowprops=dict(arrowstyle='->', color='#0571B0', lw=0.8),
            bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='#0571B0', lw=0.6))

ax.annotate('Heartbeat LED (GP25)',
            xy=(pc_x + pc_w / 2 + 2, pc_y + pc_h),
            xytext=(pc_x + pc_w / 2 + 2, pc_y + pc_h + 0.7),
            fontsize=5.5, ha='center', color='#666',
            arrowprops=dict(arrowstyle='->', color='#666', lw=0.6),
            bbox=dict(boxstyle='round,pad=0.15', fc='#FFFFDD', ec='#666', lw=0.5))

ax.text(pc_x + pc_w / 2, pc_y - 0.7,
        'Safety: auto-stop after 500 ms with no commands',
        ha='center', fontsize=6, color='#666', style='italic')

# Legend
lx, ly = 13.8, 10.2
for i, (col, ls, label) in enumerate([
    (C['uart'], '-',  'UART (115 200 baud)'),
    (C['i2c'],  '-',  'I\u00B2C Bus 1'),
    (C['pwm'],  '-',  'PWM / Motor drive'),
    (C['enc'],  '--', 'Encoder signals'),
    (C['pwr'],  '--', 'Power (6 V battery)'),
    (C['gnd'],  ':',  'Common ground'),
]):
    yy = ly - i * 0.4
    ax.plot([lx, lx + 0.8], [yy, yy], color=col, lw=1.5, ls=ls)
    ax.text(lx + 1.0, yy, label, va='center', fontsize=6)

# Save
out = r'paper_docs\Ryker_Kollmyer___UPDATED_PAPER\circuit_schematic.png'
plt.tight_layout(pad=0.3)
plt.savefig(out, dpi=300, bbox_inches='tight', facecolor='white')
print(f"Saved: {out}")
plt.close()
