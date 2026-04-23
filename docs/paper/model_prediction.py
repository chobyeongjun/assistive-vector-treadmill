"""
Model-based prediction of shank-level cable-driven assist torques.

3-link planar biomechanical model with Winter 2009 normative gait trajectory.
For each (attachment_position, force_direction, force_magnitude) combination,
compute the hip and knee torque contribution from a half-sine phase-triggered
cable force profile, and compare against natural gait peak joint moments.

Conventions follow docs/paper/biomechanical_model.md:
  - Origin at hip joint. x = forward (walking direction), y = upward.
  - Absolute angle theta_i measured from downward vertical, CCW positive.
  - phi_hip = theta1,  phi_knee = theta1 - theta2,  phi_ankle = theta2 - theta3 + pi/2
  - Cable direction fixed in LAB frame at neutral standing pose (first-order
    approximation; exact time-varying direction would require pulley position).

Usage:
    python model_prediction.py

Outputs (written to same directory):
    model_prediction.png, .pdf    : 3x2 torque figure
    model_prediction_moment_arm.png : effective moment-arm figure
    model_prediction_summary.txt  : printed numerical summary
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import os

# ==============================================================
# 1. Subject + segment parameters (70 kg, 1.75 m, Winter 2009)
# ==============================================================
BW_kg = 70.0
BW_N = BW_kg * 9.81                 # ~ 686.7 N
m1, m2, m3 = 7.000, 3.255, 1.015    # thigh, shank, foot mass (kg)
l1, l2, l3 = 0.4288, 0.4305, 0.2660 # segment lengths (m)

# Natural swing-phase peak joint moments (Winter 2009, ~N*m/kg scaled)
# Hip flexion peak during swing initiation: ~0.35 N*m/kg
# Knee flexion peak in early swing: ~0.20 N*m/kg
# Knee extension peak in terminal swing (eccentric): ~0.15 N*m/kg
NAT_HIP_FLEX_PEAK  = 0.35 * BW_kg   # ~24.5 N*m
NAT_KNEE_FLEX_PEAK = 0.20 * BW_kg   # ~14 N*m
NAT_KNEE_EXT_PEAK  = 0.15 * BW_kg   # ~10.5 N*m
NAT_HIP_REF  = NAT_HIP_FLEX_PEAK
NAT_KNEE_REF = NAT_KNEE_FLEX_PEAK   # use the larger of the two for comparison band

# ==============================================================
# 2. Winter 2009 normative gait kinematics (clinical angles, deg)
#    Digitized from standard tables; flexion + / extension -
# ==============================================================
gait_pct_key = np.array([0, 10, 20, 30, 40, 50, 60, 70, 75, 80, 90, 100])
hip_key   = np.array([25, 15,  5, -5, -10, -15, -10,  15, 25, 28, 28, 25])
knee_key  = np.array([ 5, 18, 10,  5,   5,  10,  40,  62, 65, 55, 15,  5])
ankle_key = np.array([-5,  0,  5,  8,  10,   0, -15,  -8, -5, -2, -5, -5])

cs_hip   = CubicSpline(gait_pct_key, np.deg2rad(hip_key),   bc_type='periodic')
cs_knee  = CubicSpline(gait_pct_key, np.deg2rad(knee_key),  bc_type='periodic')
cs_ankle = CubicSpline(gait_pct_key, np.deg2rad(ankle_key), bc_type='periodic')

gc = np.linspace(0, 100, 501)
phi_hip   = cs_hip(gc)
phi_knee  = cs_knee(gc)
phi_ankle = cs_ankle(gc)

theta1 = phi_hip
theta2 = theta1 - phi_knee
theta3 = theta2 - phi_ankle + np.pi/2

# ==============================================================
# 3. Assist configurations
# ==============================================================
positions = {'HIGH': 0.25, 'MID': 0.50, 'LOW': 0.75}
directions_deg = {'0deg_horizontal': 0, '30deg_up': 30}
magnitudes_pct = [5, 7, 9]   # % raw body weight

# onset / release windows per position (% gait cycle)
# Release fixed at 85%GC across all positions (hip-moment deceleration onset);
# only onset shifts to match each attachment's knee-flexion vs knee-extension role.
profile_windows = {'HIGH': (60, 85), 'MID': (65, 85), 'LOW': (70, 85)}

# ==============================================================
# 4. Kinematic + torque functions
# ==============================================================
def attachment_world(s, th1, th2):
    x = l1*np.sin(th1) + s*np.sin(th2)
    y = -l1*np.cos(th1) - s*np.cos(th2)
    return x, y

def knee_world(th1):
    return l1*np.sin(th1), -l1*np.cos(th1)

def cable_torques_at_peak(s, dir_deg, F_peak):
    """Torques at each gait-cycle sample assuming full F_peak cable tension.
    Direction is constant in lab frame (neutral-pose approximation)."""
    d_rad = np.deg2rad(dir_deg)
    Fx = F_peak * np.cos(d_rad)
    Fy = F_peak * np.sin(d_rad)
    ax, ay = attachment_world(s, theta1, theta2)
    kx, ky = knee_world(theta1)
    # Hip torque (origin at hip)
    tau_hip = ax*Fy - ay*Fx
    # Knee torque
    tau_knee = (ax - kx)*Fy - (ay - ky)*Fx
    return tau_hip, tau_knee

def half_sine_envelope(onset_pct, release_pct):
    env = np.zeros_like(gc)
    mask = (gc >= onset_pct) & (gc <= release_pct)
    t_norm = (gc[mask] - onset_pct) / (release_pct - onset_pct)
    env[mask] = np.sin(np.pi * t_norm)
    return env

# ==============================================================
# 5. Compute all combinations
# ==============================================================
results = {}
for pos_name, s_ratio in positions.items():
    s = s_ratio * l2
    onset, release = profile_windows[pos_name]
    env = half_sine_envelope(onset, release)
    for dir_name, dir_deg in directions_deg.items():
        for mag_pct in magnitudes_pct:
            F_peak = BW_N * mag_pct / 100.0
            tau_hip_peakcable, tau_knee_peakcable = cable_torques_at_peak(s, dir_deg, F_peak)
            tau_hip_profile  = tau_hip_peakcable  * env
            tau_knee_profile = tau_knee_peakcable * env
            key = (pos_name, dir_name, mag_pct)
            # metric: peak absolute torque magnitude during profile window
            pk_hip  = np.max(np.abs(tau_hip_profile))
            pk_knee = np.max(np.abs(tau_knee_profile))
            impulse_hip  = np.trapz(tau_hip_profile,  gc/100.0)  # area in N*m per cycle-fraction
            impulse_knee = np.trapz(tau_knee_profile, gc/100.0)
            results[key] = dict(
                tau_hip=tau_hip_profile, tau_knee=tau_knee_profile,
                F_peak=F_peak, peak_hip=pk_hip, peak_knee=pk_knee,
                impulse_hip=impulse_hip, impulse_knee=impulse_knee,
            )

# ==============================================================
# 6. Main figure: 3 (positions) x 2 (hip/knee torque) panels
# ==============================================================
fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
fig.suptitle('Model-predicted assist joint torques (half-sine, 3-link model + Winter 2009 gait)',
             fontsize=12)

pos_order = ['HIGH', 'MID', 'LOW']
dir_colors = {'0deg_horizontal': 'steelblue', '30deg_up': 'coral'}
mag_alpha  = {5: 0.35, 7: 1.0, 9: 0.6}
mag_style  = {5: ':',  7: '-', 9: '--'}

for row, pos_name in enumerate(pos_order):
    ax_hip  = axes[row, 0]
    ax_knee = axes[row, 1]

    # Natural peak reference bands
    ax_hip.axhspan(-NAT_HIP_REF, NAT_HIP_REF, color='gray', alpha=0.12,
                   label=f'natural swing hip peak (+/-{NAT_HIP_REF:.0f} Nm)')
    ax_knee.axhspan(-NAT_KNEE_REF, NAT_KNEE_REF, color='gray', alpha=0.12,
                    label=f'natural swing knee peak (+/-{NAT_KNEE_REF:.0f} Nm)')

    # Shade profile window
    on, off = profile_windows[pos_name]
    ax_hip.axvspan(on, off, color='yellow', alpha=0.06)
    ax_knee.axvspan(on, off, color='yellow', alpha=0.06)

    for dir_name in ['0deg_horizontal', '30deg_up']:
        for mag in magnitudes_pct:
            r = results[(pos_name, dir_name, mag)]
            lbl = f'{dir_name.replace("_"," ")}, {mag}%BW' if row == 0 and mag == 7 else None
            ax_hip.plot(gc, r['tau_hip'], color=dir_colors[dir_name],
                         linestyle=mag_style[mag], alpha=mag_alpha[mag], lw=1.6, label=lbl)
            ax_knee.plot(gc, r['tau_knee'], color=dir_colors[dir_name],
                          linestyle=mag_style[mag], alpha=mag_alpha[mag], lw=1.6)

    ax_hip.set_ylabel(f'{pos_name} (s={positions[pos_name]:.2f}*l2)\nhip torque [N*m]')
    ax_knee.set_ylabel('knee torque [N*m]')
    ax_hip.grid(alpha=0.3); ax_knee.grid(alpha=0.3)
    if row == 0:
        ax_hip.legend(fontsize=7, loc='upper left', framealpha=0.9)
        ax_knee.legend(fontsize=7, loc='upper left', framealpha=0.9)

axes[-1, 0].set_xlabel('Gait cycle [%]')
axes[-1, 1].set_xlabel('Gait cycle [%]')
plt.tight_layout()
HERE = os.path.dirname(os.path.abspath(__file__))
plt.savefig(os.path.join(HERE, 'model_prediction.png'), dpi=150, bbox_inches='tight')
plt.savefig(os.path.join(HERE, 'model_prediction.pdf'), bbox_inches='tight')
plt.close(fig)

# ==============================================================
# 7. Effective moment-arm figure (hip vs knee) across gait cycle
# ==============================================================
fig2, axes2 = plt.subplots(1, 2, figsize=(12, 4.5), sharex=True)
for pos_name, s_ratio in positions.items():
    s = s_ratio * l2
    for dir_name, dir_deg in directions_deg.items():
        # unit torque (F_peak = 1 N) to get moment arm in m
        th, tk = cable_torques_at_peak(s, dir_deg, 1.0)
        lbl = f'{pos_name}, {dir_name.replace("_"," ")}'
        ls = '-' if dir_name == '0deg_horizontal' else '--'
        c  = {'HIGH':'tab:blue','MID':'tab:green','LOW':'tab:red'}[pos_name]
        axes2[0].plot(gc, th, ls, color=c, label=lbl, lw=1.4)
        axes2[1].plot(gc, tk, ls, color=c, label=lbl, lw=1.4)
axes2[0].set_title('Effective hip moment arm d_hip [m]  (torque per N cable tension)')
axes2[1].set_title('Effective knee moment arm d_knee [m]')
for a in axes2:
    a.set_xlabel('Gait cycle [%]')
    a.grid(alpha=0.3)
    a.legend(fontsize=7, loc='best')
plt.tight_layout()
plt.savefig(os.path.join(HERE, 'model_prediction_moment_arm.png'), dpi=150, bbox_inches='tight')
plt.close(fig2)

# ==============================================================
# 8. Numerical summary
# ==============================================================
lines = []
lines.append("="*78)
lines.append(f" 3-link model prediction -- 70 kg subject, Winter 2009 gait")
lines.append(f" Natural swing peak hip flexion  moment: ~{NAT_HIP_FLEX_PEAK:.1f} Nm")
lines.append(f" Natural swing peak knee flexion moment: ~{NAT_KNEE_FLEX_PEAK:.1f} Nm")
lines.append(f" Natural swing peak knee extension moment: ~{NAT_KNEE_EXT_PEAK:.1f} Nm")
lines.append("="*78)
lines.append(f"{'Pos':<5} {'Direction':<17} {'F_peak(N)':>9}  "
             f"{'peak_|tau_hip|':>14} ({'%natHip':>7})  "
             f"{'peak_|tau_knee|':>15} ({'%natKneeF':>9})")
lines.append("-"*78)
for pos_name in pos_order:
    for dir_name in ['0deg_horizontal', '30deg_up']:
        for mag in magnitudes_pct:
            r = results[(pos_name, dir_name, mag)]
            lines.append(
                f"{pos_name:<5} {dir_name:<17} {r['F_peak']:>9.1f}  "
                f"{r['peak_hip']:>14.2f} ({100*r['peak_hip']/NAT_HIP_REF:>6.1f}%)  "
                f"{r['peak_knee']:>15.2f} ({100*r['peak_knee']/NAT_KNEE_REF:>8.1f}%)"
            )
    lines.append("-"*78)

lines.append("")
lines.append("Interpretation:")
lines.append(" - HIGH attachment dominated by HIP moment (short knee lever).")
lines.append(" - LOW attachment dominated by KNEE moment (long knee lever).")
lines.append(" - 0deg (forward) produces larger knee extension torque than 30deg (up) for")
lines.append("   the same cable tension; 30deg adds a lifting component at the hip.")
lines.append(" - 'Partial assist' guideline (20-60% of natural peak) is the characterization")
lines.append("   sweet spot; values >80% approach replacement rather than modulation.")

summary_txt = "\n".join(lines)
with open(os.path.join(HERE, 'model_prediction_summary.txt'), 'w') as f:
    f.write(summary_txt)
print(summary_txt)
