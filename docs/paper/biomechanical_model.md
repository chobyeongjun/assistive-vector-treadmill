# 3-Link Planar Biomechanical Model — Complete Mathematical Derivation

## 1. Model Overview

A planar (sagittal plane) model of the human lower limb during gait, consisting of three rigid links:

| Link | Segment | Proximal Joint | Distal Joint |
|------|---------|---------------|--------------|
| 1 | Thigh | Hip | Knee |
| 2 | Shank | Knee | Ankle |
| 3 | Foot | Ankle | Toe (2nd metatarsal) |

### Assumptions
- **Planar motion**: All movement occurs in the sagittal plane
- **Rigid bodies**: No deformation of segments
- **Frictionless joints**: Ideal revolute joints at hip, knee, ankle
- **Point masses at CoM**: Distributed mass represented by CoM + rotational inertia
- **Fixed hip**: Hip joint is the origin (valid for stance leg analysis, or treadmill walking)

### Notation

| Symbol | Description | Unit |
|--------|-------------|------|
| m₁, m₂, m₃ | Segment masses (thigh, shank, foot) | kg |
| l₁, l₂, l₃ | Segment lengths | m |
| lc₁, lc₂, lc₃ | CoM distance from proximal joint | m |
| I₁, I₂, I₃ | Moment of inertia about CoM | kg·m² |
| θ₁, θ₂, θ₃ | Absolute angles from downward vertical | rad |
| τ₁, τ₂, τ₃ | Joint torques (hip, knee, ankle) | N·m |
| g | Gravitational acceleration (9.81) | m/s² |

Shorthand:
- sᵢ = sin(θᵢ), cᵢ = cos(θᵢ)
- sᵢⱼ = sin(θᵢ - θⱼ), cᵢⱼ = cos(θᵢ - θⱼ)

---

## 2. Coordinate System

```
    y ↑
      |
      |___→ x
    (hip)
      \  θ₁ (from vertical)
       \
        ● knee
         \  θ₂
          \
           ● ankle
            \  θ₃
             ● toe
```

- **Origin**: Hip joint
- **y-axis**: Upward
- **x-axis**: Forward (direction of walking)
- **Angles**: Measured from negative y-axis (downward vertical), positive = counterclockwise (forward flexion)
- **θ = 0**: All links hanging straight down

---

## 3. Position Kinematics

### Joint Positions (Hip at Origin)

**Knee:**
```
x_knee = l₁·sin(θ₁)
y_knee = -l₁·cos(θ₁)
```

**Ankle:**
```
x_ankle = l₁·sin(θ₁) + l₂·sin(θ₂)
y_ankle = -l₁·cos(θ₁) - l₂·cos(θ₂)
```

**Toe (foot endpoint):**
```
x_toe = l₁·sin(θ₁) + l₂·sin(θ₂) + l₃·sin(θ₃)
y_toe = -l₁·cos(θ₁) - l₂·cos(θ₂) - l₃·cos(θ₃)
```

### Center of Mass Positions

**Thigh CoM:**
```
x₁ = lc₁·sin(θ₁)
y₁ = -lc₁·cos(θ₁)
```

**Shank CoM:**
```
x₂ = l₁·sin(θ₁) + lc₂·sin(θ₂)
y₂ = -l₁·cos(θ₁) - lc₂·cos(θ₂)
```

**Foot CoM:**
```
x₃ = l₁·sin(θ₁) + l₂·sin(θ₂) + lc₃·sin(θ₃)
y₃ = -l₁·cos(θ₁) - l₂·cos(θ₂) - lc₃·cos(θ₃)
```

---

## 4. Velocity Kinematics

Time derivatives of CoM positions:

**Thigh CoM velocity:**
```
ẋ₁ = lc₁·cos(θ₁)·θ̇₁
ẏ₁ = lc₁·sin(θ₁)·θ̇₁
```

**Shank CoM velocity:**
```
ẋ₂ = l₁·cos(θ₁)·θ̇₁ + lc₂·cos(θ₂)·θ̇₂
ẏ₂ = l₁·sin(θ₁)·θ̇₁ + lc₂·sin(θ₂)·θ̇₂
```

**Foot CoM velocity:**
```
ẋ₃ = l₁·cos(θ₁)·θ̇₁ + l₂·cos(θ₂)·θ̇₂ + lc₃·cos(θ₃)·θ̇₃
ẏ₃ = l₁·sin(θ₁)·θ̇₁ + l₂·sin(θ₂)·θ̇₂ + lc₃·sin(θ₃)·θ̇₃
```

### Squared Velocities (v² = ẋ² + ẏ²)

**Thigh:**
```
v₁² = lc₁²·θ̇₁²
```

**Shank (using cos(A)cos(B) + sin(A)sin(B) = cos(A-B)):**
```
v₂² = l₁²·θ̇₁² + lc₂²·θ̇₂² + 2·l₁·lc₂·cos(θ₁ - θ₂)·θ̇₁·θ̇₂
```

**Foot:**
```
v₃² = l₁²·θ̇₁² + l₂²·θ̇₂² + lc₃²·θ̇₃²
     + 2·l₁·l₂·cos(θ₁ - θ₂)·θ̇₁·θ̇₂
     + 2·l₁·lc₃·cos(θ₁ - θ₃)·θ̇₁·θ̇₃
     + 2·l₂·lc₃·cos(θ₂ - θ₃)·θ̇₂·θ̇₃
```

---

## 5. Lagrangian Derivation

### 5.1 Kinetic Energy

```
T = Σᵢ [½mᵢ(ẋᵢ² + ẏᵢ²) + ½Iᵢθ̇ᵢ²]
```

Substituting v²:

```
T = ½(m₁·lc₁² + I₁)·θ̇₁²                           ← Thigh
  + ½m₂·l₁²·θ̇₁² + ½(m₂·lc₂² + I₂)·θ̇₂²            ← Shank translational
  + m₂·l₁·lc₂·cos(θ₁-θ₂)·θ̇₁·θ̇₂                   ← Shank coupling
  + ½m₃·l₁²·θ̇₁² + ½m₃·l₂²·θ̇₂² + ½(m₃·lc₃²+I₃)·θ̇₃²  ← Foot translational
  + m₃·l₁·l₂·cos(θ₁-θ₂)·θ̇₁·θ̇₂                    ← Foot thigh-shank coupling
  + m₃·l₁·lc₃·cos(θ₁-θ₃)·θ̇₁·θ̇₃                   ← Foot thigh-foot coupling
  + m₃·l₂·lc₃·cos(θ₂-θ₃)·θ̇₂·θ̇₃                   ← Foot shank-foot coupling
```

Collecting by θ̇ᵢθ̇ⱼ terms:

```
T = ½·θ̇ᵀ·M(q)·θ̇
```

### 5.2 Potential Energy

```
V = m₁·g·y₁ + m₂·g·y₂ + m₃·g·y₃

  = -m₁·g·lc₁·cos(θ₁)
    -m₂·g·[l₁·cos(θ₁) + lc₂·cos(θ₂)]
    -m₃·g·[l₁·cos(θ₁) + l₂·cos(θ₂) + lc₃·cos(θ₃)]

  = -(m₁·lc₁ + m₂·l₁ + m₃·l₁)·g·cos(θ₁)
    -(m₂·lc₂ + m₃·l₂)·g·cos(θ₂)
    -m₃·lc₃·g·cos(θ₃)
```

### 5.3 Lagrangian

```
L = T - V
```

### 5.4 Euler-Lagrange Equations

```
d/dt(∂L/∂θ̇ᵢ) - ∂L/∂θᵢ = τᵢ    for i = 1, 2, 3
```

This yields:

```
M(q)·q̈ + C(q,q̇)·q̇ + G(q) = τ
```

---

## 6. Mass Matrix M(q) — Full Expansion

M(q) is a 3×3 symmetric positive definite matrix.

### Diagonal Elements (always positive):

```
M₁₁ = m₁·lc₁² + (m₂ + m₃)·l₁² + I₁
```
> Physical meaning: Total rotational inertia about the hip.
> Includes thigh's own inertia plus the inertia of shank+foot
> treated as point masses at the knee.

```
M₂₂ = m₂·lc₂² + m₃·l₂² + I₂
```
> Total rotational inertia about the knee (shank + foot contribution).

```
M₃₃ = m₃·lc₃² + I₃
```
> Total rotational inertia about the ankle (foot only).

### Off-Diagonal Elements (configuration-dependent coupling):

Define coupling coefficients:
```
α₁₂ = (m₂·lc₂ + m₃·l₂)·l₁
α₁₃ = m₃·lc₃·l₁
α₂₃ = m₃·lc₃·l₂
```

Then:
```
M₁₂ = M₂₁ = α₁₂·cos(θ₁ - θ₂)
M₁₃ = M₃₁ = α₁₃·cos(θ₁ - θ₃)
M₂₃ = M₃₂ = α₂₃·cos(θ₂ - θ₃)
```

> Physical meaning: M₁₂ represents the inertial coupling between thigh and shank motion.
> When θ₁ = θ₂ (links aligned), coupling is maximum (cos(0) = 1).
> When θ₁ - θ₂ = π/2 (perpendicular), coupling vanishes.

### Matrix Form:

```
M(q) = ┌                                              ┐
       │ M₁₁           α₁₂·c₁₂       α₁₃·c₁₃       │
       │ α₁₂·c₁₂      M₂₂            α₂₃·c₂₃       │
       │ α₁₃·c₁₃      α₂₃·c₂₃       M₃₃            │
       └                                              ┘
```

---

## 7. Coriolis and Centrifugal Terms

### Derivation via Christoffel Symbols

The Christoffel symbols of the first kind:
```
cᵢⱼₖ = ½(∂Mᵢⱼ/∂qₖ + ∂Mᵢₖ/∂qⱼ - ∂Mⱼₖ/∂qᵢ)
```

### Partial Derivatives of M

```
∂M₁₂/∂θ₁ = -α₁₂·sin(θ₁ - θ₂)    ∂M₁₂/∂θ₂ = α₁₂·sin(θ₁ - θ₂)
∂M₁₃/∂θ₁ = -α₁₃·sin(θ₁ - θ₃)    ∂M₁₃/∂θ₃ = α₁₃·sin(θ₁ - θ₃)
∂M₂₃/∂θ₂ = -α₂₃·sin(θ₂ - θ₃)    ∂M₂₃/∂θ₃ = α₂₃·sin(θ₂ - θ₃)
```

All diagonal elements are constant → ∂Mᵢᵢ/∂θⱼ = 0.

### Coupling Terms h

Define:
```
h₁₂ = -α₁₂·sin(θ₁ - θ₂) = -(m₂·lc₂ + m₃·l₂)·l₁·sin(θ₁ - θ₂)
h₁₃ = -α₁₃·sin(θ₁ - θ₃) = -m₃·lc₃·l₁·sin(θ₁ - θ₃)
h₂₃ = -α₂₃·sin(θ₂ - θ₃) = -m₃·lc₃·l₂·sin(θ₂ - θ₃)
```

### Coriolis Matrix C(q,q̇)

Using the Christoffel symbol formulation (ensures Ṁ - 2C is skew-symmetric):

```
C(q,q̇) = ┌                                      ┐
          │  0           h₁₂·θ̇₂      h₁₃·θ̇₃    │
          │ -h₁₂·θ̇₁     0           h₂₃·θ̇₃    │
          │ -h₁₃·θ̇₁    -h₂₃·θ̇₂      0         │
          └                                      ┘
```

### Coriolis/Centrifugal Vector b = C·q̇

```
b₁ = h₁₂·θ̇₂² + h₁₃·θ̇₃²
b₂ = -h₁₂·θ̇₁² + h₂₃·θ̇₃²
b₃ = -h₁₃·θ̇₁² - h₂₃·θ̇₂²
```

Physical interpretation:
- **b₁**: Centrifugal forces on thigh from shank (h₁₂θ̇₂²) and foot (h₁₃θ̇₃²) rotations
- **b₂**: Centrifugal force from thigh (-h₁₂θ̇₁²) and foot (h₂₃θ̇₃²) on shank
- **b₃**: Centrifugal forces from thigh (-h₁₃θ̇₁²) and shank (-h₂₃θ̇₂²) on foot

> Note: The sign pattern shows Newton's third law — the centrifugal effect
> of link j on link i has opposite sign to the effect of link i on link j.

### Skew-Symmetry Property

The matrix N = Ṁ - 2C is skew-symmetric:
```
xᵀ·N·x = 0  for all x
```
This is the **passivity property**, crucial for control design:
- Guarantees Lyapunov stability of computed-torque control
- Enables model-based impedance control design

---

## 8. Gravity Vector G(q)

```
G(q) = ∂V/∂q
```

Fully expanded:

```
G₁ = (m₁·lc₁ + m₂·l₁ + m₃·l₁)·g·sin(θ₁)
G₂ = (m₂·lc₂ + m₃·l₂)·g·sin(θ₂)
G₃ = m₃·lc₃·g·sin(θ₃)
```

Physical interpretation:
- **G₁**: Gravity torque about hip from entire leg (thigh + shank + foot mass)
- **G₂**: Gravity torque about knee from shank + foot
- **G₃**: Gravity torque about ankle from foot only
- At equilibrium (q = 0, hanging straight down): G = [0, 0, 0]
- Maximum gravity torque when segment is horizontal (θ = ±π/2)

---

## 9. Complete Equations of Motion

### Compact Form
```
M(q)·q̈ + C(q,q̇)·q̇ + G(q) = τ
```

### Fully Expanded (Each Equation)

**Equation 1 — Hip:**
```
τ₁ = [m₁·lc₁² + (m₂+m₃)·l₁² + I₁]·θ̈₁
   + [(m₂·lc₂ + m₃·l₂)·l₁·cos(θ₁-θ₂)]·θ̈₂
   + [m₃·lc₃·l₁·cos(θ₁-θ₃)]·θ̈₃
   - (m₂·lc₂ + m₃·l₂)·l₁·sin(θ₁-θ₂)·θ̇₂²
   - m₃·lc₃·l₁·sin(θ₁-θ₃)·θ̇₃²
   + (m₁·lc₁ + m₂·l₁ + m₃·l₁)·g·sin(θ₁)
```

**Equation 2 — Knee:**
```
τ₂ = [(m₂·lc₂ + m₃·l₂)·l₁·cos(θ₁-θ₂)]·θ̈₁
   + [m₂·lc₂² + m₃·l₂² + I₂]·θ̈₂
   + [m₃·lc₃·l₂·cos(θ₂-θ₃)]·θ̈₃
   + (m₂·lc₂ + m₃·l₂)·l₁·sin(θ₁-θ₂)·θ̇₁²
   - m₃·lc₃·l₂·sin(θ₂-θ₃)·θ̇₃²
   + (m₂·lc₂ + m₃·l₂)·g·sin(θ₂)
```

**Equation 3 — Ankle:**
```
τ₃ = [m₃·lc₃·l₁·cos(θ₁-θ₃)]·θ̈₁
   + [m₃·lc₃·l₂·cos(θ₂-θ₃)]·θ̈₂
   + [m₃·lc₃² + I₃]·θ̈₃
   + m₃·lc₃·l₁·sin(θ₁-θ₃)·θ̇₁²
   + m₃·lc₃·l₂·sin(θ₂-θ₃)·θ̇₂²
   + m₃·lc₃·g·sin(θ₃)
```

### Forward Dynamics (compute accelerations from torques):
```
q̈ = M⁻¹(q)·[τ - C(q,q̇)·q̇ - G(q)]
```

### Inverse Dynamics (compute torques from trajectory):
```
τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)
```

---

## 10. Properties of the Model

### 10.1 Positive Definiteness of M(q)

M(q) is positive definite for all q. This can be verified by:
1. All diagonal elements are positive (sum of positive terms)
2. The Schur complement conditions hold
3. Physically: kinetic energy T = ½q̇ᵀMq̇ > 0 for any q̇ ≠ 0

### 10.2 Passivity (Ṁ - 2C is skew-symmetric)

```
d/dt[½q̇ᵀMq̇] = q̇ᵀτ - q̇ᵀGq̇ · 0 (no dissipation terms)

Actually: d/dt[½q̇ᵀMq̇] = q̇ᵀMq̈ + ½q̇ᵀṀq̇
                        = q̇ᵀ[τ - Cq̇ - G] + ½q̇ᵀṀq̇
                        = q̇ᵀτ - q̇ᵀG + ½q̇ᵀ(Ṁ - 2C)q̇
                        = q̇ᵀτ - q̇ᵀG   (since Ṁ-2C is skew)
```

### 10.3 Energy Conservation

For τ = 0 (no external torques):
```
E = T + V = constant
dE/dt = q̇ᵀ·τ = 0 ✓
```

### 10.4 Gravity Compensation

A controller that simply cancels gravity:
```
τ = G(q)
```
results in a system with no gravitational effects:
```
M(q)·q̈ + C(q,q̇)·q̇ = 0
```
which conserves kinetic energy: d(T)/dt = 0.

---

## 11. Angle Conventions (Absolute ↔ Clinical)

### Absolute → Clinical
```
φ_hip   = θ₁                      (hip flexion from vertical)
φ_knee  = θ₁ - θ₂                 (knee flexion, 0 = full extension)
φ_ankle = θ₂ - θ₃ + π/2           (dorsiflexion, 0 = neutral)
```

### Clinical → Absolute
```
θ₁ = φ_hip
θ₂ = φ_hip - φ_knee
θ₃ = φ_hip - φ_knee - φ_ankle + π/2
```

### Normal Gait Ranges (Winter 2009)
| Joint | Min | Max | Range |
|-------|-----|-----|-------|
| Hip | -10° (extension) | 30° (flexion) | 40° |
| Knee | 0° (extension) | 65° (swing flexion) | 65° |
| Ankle | -20° (plantarflexion) | 10° (dorsiflexion) | 30° |

---

## 12. Numerical Values (70kg, 1.75m Adult)

### Segment Parameters (Winter 2009)

| Parameter | Thigh | Shank | Foot |
|-----------|-------|-------|------|
| Mass [kg] | 7.000 | 3.255 | 1.015 |
| Length [m] | 0.4288 | 0.4305 | 0.2660 |
| CoM from proximal [m] | 0.1857 | 0.1864 | 0.1330 |
| I_CoM [kg·m²] | 0.1340 | 0.0553 | 0.0162 |

### Precomputed Coefficients

| Coefficient | Value | Unit | Meaning |
|-------------|-------|------|---------|
| M₁₁_const | ~2.04 | kg·m² | Hip inertia (constant part) |
| M₂₂_const | ~0.166 | kg·m² | Knee inertia (constant part) |
| M₃₃_const | ~0.034 | kg·m² | Ankle inertia (constant part) |
| α₁₂ | ~0.447 | kg·m² | Thigh-shank coupling |
| α₁₃ | ~0.058 | kg·m² | Thigh-foot coupling |
| α₂₃ | ~0.058 | kg·m² | Shank-foot coupling |
| G₁_coeff | ~17.5 | N·m | Hip gravity coefficient |
| G₂_coeff | ~10.2 | N·m | Knee gravity coefficient |
| G₃_coeff | ~1.33 | N·m | Ankle gravity coefficient |

---

## 13. Cable Force Mapping (H-Walker Specific)

### Cable Configuration
- Cable attaches near knee (posterior side of shank)
- Routes upward to walker frame above hip
- Unidirectional force (tension only, T ≥ 0)

### Mapping
```
τ_joints = Jᵀ_cable · F_cable
```

For 2D cross product:
```
τ_hip   = r_hip × F_cable   = (r_x · F_y - r_y · F_x)
τ_knee  = r_knee × F_cable
τ_ankle = 0  (cable above ankle)
```

where rᵢ = p_attach - p_jointᵢ

### Effective Moment Arm
```
d_eff(q) = |r × û_cable|
```
Varies with joint configuration → moment arm changes during gait cycle.

---

## 14. References

1. Winter, D.A. (2009). *Biomechanics and Motor Control of Human Movement*, 4th ed. Wiley.
2. Dempster, W.T. (1955). *Space Requirements of the Seated Operator*. WADC Technical Report 55-159.
3. Spong, M.W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
4. Murray, R.M., Li, Z., & Sastry, S.S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.
5. Perry, J. & Burnfield, J.M. (2010). *Gait Analysis: Normal and Pathological Function*, 2nd ed. SLACK.
6. Umberger, B.R. (2010). Stance and Swing Phase Costs in Human Walking. *J. Royal Society Interface*, 7(50), 1329-1340.
