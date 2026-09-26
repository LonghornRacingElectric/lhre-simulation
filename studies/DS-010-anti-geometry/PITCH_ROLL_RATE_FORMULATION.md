# Pitch and Roll Rate Formulation

This note derives how suspension geometry, springs, and anti-roll bars (ARBs)
set the roll and pitch rates. It then shows how those rates, together with
roll-centre and anti geometry, set the roll and pitch gradients. It uses the same
definitions as BobSim's FourPostEval and `dyn_py`, so every formula can be
checked against the metrics those tools report. §6 does this for the 2027 WIP
vehicle. The pitch half is the forward model behind the DS-010 anti-geometry
inversion in [README.md](README.md).

**Rate** means stiffness (moment per angle), not angular velocity. **Gradient**
means angle per g of acceleration. The rates here are quasi-static; dampers only
matter for the transient, see §7.

## 1. Conventions

- Axes follow `vehicle.yml`: x forward, y left, z up, front axle at x ≈ 0.
- Roll φ is about +x: positive lifts the left side. Pitch θ is about +y:
  positive is nose down.
- Rotational rates are derived in N·m/rad. Multiply by π/180 for N·m/deg.
  1 lb·in/deg = 6.4736 N·m/rad, which is how the `vehicle.yml` comment
  "40 lb-in/deg" becomes `rate_n_m_per_rad: 258.942154`.
- Lower-case `k` is a per-wheel linear rate (N/m). Upper-case `K` is a per-axle
  linear rate or a rotational rate. Subscript `i ∈ {f, r}` is the axle.

| Symbol | Meaning | Source |
| --- | --- | --- |
| $m$, $h$ | total mass, total CG height | `vehicle.yml` masses |
| $m_s$, $h_s$ | sprung mass (chassis + driver), its CG height | `sprung_mass`, `driver_mass` |
| $m_{u,i}$, $z_{u,i}$ | unsprung mass per axle, its CG height (≈ wheel centre) | `<axle>.masses` |
| $a$, $b$, $l$ | CG to front axle, CG to rear axle, wheelbase ($a+b=l$) | |
| $a_s$, $b_s$ | same, for the sprung CG | |
| $t_i$ | track | 2 × `wheel_center_m[1]` |
| $k_s$ | spring rate at the spring | slope of `actuation.shock.spring_table` |
| $k_{bar}$ | ARB torsional rate: moment per radian of twist between its two arms | `actuation.stabar.rate_n_m_per_rad` |
| $k_t$ | tire vertical rate | `tire.vertical_stiffness_n_per_m` |
| $K_c$ | chassis torsional rate | `body.torsional_stiff_n_m_per_rad` |
| $z_{rc,i}$ | roll-centre height | FourPostEval `avg_fbrc_equivalent_height_*` |
| $\beta_f$ | front share of friction braking | `brake.front_bias` |

## 2. From hardpoints to installation ratios

Geometry enters the rates almost entirely through installation ratios: how far
each elastic element moves per unit of wheel travel.

### 2.1 Definition

For any element $j$ (spring, ARB drop link, pushrod), the installation ratio is
the element's length change per unit of wheel-centre vertical travel relative to
the body:

```math
IR_j = \frac{\partial x_j}{\partial z_w}
```

By virtual work, a force $F_j$ in the element appears at the wheel as
$F_w = IR_j\,F_j$. Differentiating gives the wheel rate:

```math
k_w = \frac{\partial F_w}{\partial z_w} = k_s\,IR_s^2 + F_s\,\frac{\partial IR_s}{\partial z_w}
```

The first term is the installed rate. The second is the geometric rising or
falling rate, and it scales with spring preload $F_s$. BobSim reports the
inverse ratio, wheel travel per spring travel, as $MR = 1/IR_s$
(`static_motion_ratio_*`, `avg_motion_ratio_*`). FourPostEval and `dyn_py` keep
only the first term:

```math
k_w = \frac{k_s}{MR^2}
```

### 2.2 Through the bellcrank

Let the bellcrank have pivot $\mathbf p$ and unit axis $\hat a$. Each pickup
$\mathbf r_j$ drives a link with unit vector $\hat u_j$ (rod, shock, or ARB drop
link). The link's length change per radian of bellcrank rotation is its
effective lever arm:

```math
\lambda_j = \hat u_j \cdot \big(\hat a \times (\mathbf r_j - \mathbf p)\big)
```

This is the perpendicular distance from the pickup to the axis times the cosine
of the transmission angle. The rod drives the bellcrank, so:

```math
IR_s = IR_{rod}\,\frac{\lambda_s}{\lambda_{rod}}, \qquad
IR_d = IR_{rod}\,\frac{\lambda_d}{\lambda_{rod}}, \qquad
\frac{IR_d}{IR_s} = \frac{\lambda_d}{\lambda_s}
```

$IR_{rod}$ is the pushrod or pullrod length change per unit wheel travel. It is set by
where `rod_mount_m` sits on the control arm and comes from the kinematic solve
(`kin_py`, FourPostSim). A planar front-view estimate is
$IR_{rod} \approx (d_{rod}/d_{bj})\cos\gamma$. Here $d_{rod}$ and $d_{bj}$ are
distances from the arm's inboard pivot axis to the rod mount and to the ball
joint, and $\gamma$ is the angle between the rod and the rod mount's direction
of travel.

The ratio of ARB motion to spring motion is set by the bellcrank alone.

### 2.3 Anti-roll bar

Let $\hat b$ be the torsion-tube axis and
$\mathbf a = \mathbf r_{arm\_end} - \mathbf r_{bar\_end}$ the ARB arm. Let
$\hat u_d$ be the drop-link unit vector from the arm end to the bellcrank
`stabar` pickup. Arm rotation per unit drop-link travel is $1/\lambda_a$, with:

```math
\lambda_a = \hat u_d \cdot (\hat b \times \mathbf a)
```

Each arm rotates $\psi = (IR_d/\lambda_a)\,z_w$, and the bar twists by
$\Delta\psi = \psi_L - \psi_R$. In roll $z_L - z_R = t\,\phi$, so the twist
ratio is:

```math
r_{arb} \equiv \frac{\partial\,\Delta\psi}{\partial\phi} = \frac{IR_d\,t}{\lambda_a}
```

FourPostEval reports `avg_stabar_motion_ratio_*` $= 1/r_{arb}$ (roll angle per
radian of bar twist, fitted over a roll sweep).

### 2.4 Tires in series

The tire acts in series with the suspension:

```math
k_{ride} = \frac{k_w\,k_t}{k_w + k_t}
```

## 3. Roll

### 3.1 Suspension roll rate per axle

When the body rolls by φ relative to the axle, $z_L = -z_R = t\phi/2$. The
stored energy is then:

```math
U = \tfrac12 k_w\left(z_L^2 + z_R^2\right) + \tfrac12 k_{bar}\,\Delta\psi^2
  = \tfrac12\Big(\underbrace{\tfrac12 k_w t^2}_{K_{\phi,spr}} + \underbrace{k_{bar}\,r_{arb}^2}_{K_{\phi,arb}}\Big)\phi^2
```

So the suspension roll rate of an axle is:

```math
K_{\phi,s} = \underbrace{\frac{k_s}{2\,MR^2}\,t^2}_{\text{springs}}
\;+\; \underbrace{k_{bar}\left(\frac{IR_d\,t}{\lambda_a}\right)^2}_{\text{ARB}}
```

These are FourPostEval's `spring_roll_stiffness_*` and `arb_roll_stiffness_*`.
Their sum is `elastic_roll_stiffness_*`.

- **The ARB counts twice the spring form.** Hold one wheel and move the other,
  and the bar rate felt at the wheel is $k_{arb,w} = k_{bar}(IR_d/\lambda_a)^2$.
  Then $K_{\phi,arb} = k_{arb,w}\,t^2$, not $k\,t^2/2$ as for springs, because
  one wheel's travel twists the whole bar. `dyn_py` uses this form in
  `_antiroll_forces`: force on the left wheel is $K_{\phi,arb}(z_L - z_R)/t^2$.
- **The ARB does nothing in heave or pitch.** Both arms rotate together, so
  $\Delta\psi = 0$. It only acts in roll and warp.
- **Track enters squared** for both springs and ARB.
- **Roll-centre height does not appear.** $K_{\phi,s}$ belongs to the springs and
  bar alone. Roll-centre height changes the moment they have to carry (§3.3).

### 3.2 Tires and chassis

Each axle's tires add a roll rate $K_{\phi,t} = k_t\,t^2/2$ in series. The
overall roll rate of the body relative to the ground, per axle, is:

```math
K_{\phi,i} = \frac{K_{\phi,s,i}\,K_{\phi,t,i}}{K_{\phi,s,i} + K_{\phi,t,i}}
```

This ignores the load that bypasses the springs through the links; §3.4 handles
that properly.

A flexible chassis ($K_c$) connects the front and rear suspensions in series.
Assume the elastic roll moment $M_e$ (§3.3) is applied at each end in proportion
to the sprung mass: $A_f = M_e\,b_s/l$ and $A_r = M_e\,a_s/l$. The front
suspension then carries:

```math
E_f = K_{\phi,f}\;\frac{A_f\,K_{\phi,r} + K_c\,M_e}{K_{\phi,f}K_{\phi,r} + K_c\,(K_{\phi,f} + K_{\phi,r})}
```

As $K_c \to \infty$ this becomes $M_e K_{\phi,f}/(K_{\phi,f}+K_{\phi,r})$. As
$K_c \to 0$ each end keeps its own $A_i$. A flexible chassis pulls the
distribution toward the sprung-mass distribution and weakens the ARBs' control
over balance.

### 3.3 Roll moment, roll axis, and where load transfer goes

The roll axis passes through the front and rear roll centres. Under the sprung
CG its height and the roll moment arm are:

```math
z_{ra} = z_{rc,f} + (z_{rc,r} - z_{rc,f})\,\frac{a_s}{l}, \qquad h' = h_s - z_{ra}
```

Lateral load transfer at each axle has three paths:

```math
\Delta F_{z,i}\,t_i = G_i + E_i + U_i
```

| Path | Front | Rear | Reacted by |
| --- | --- | --- | --- |
| Geometric $G_i$ | $m_s a_y\,\tfrac{b_s}{l}\,z_{rc,f}$ | $m_s a_y\,\tfrac{a_s}{l}\,z_{rc,r}$ | links, instantly, no roll |
| Elastic $E_i$ | share of $M_e = m_s a_y h'$ | $M_e - E_f$ | springs + ARB, through roll |
| Unsprung $U_i$ | $m_{u,f}\,a_y\,z_{u,f}$ | $m_{u,r}\,a_y\,z_{u,r}$ | tires directly |

The paths always sum to the whole-vehicle overturning moment:

```math
\sum_i (G_i + E_i + U_i) = m_s a_y (z_{ra} + h') + \sum_i m_{u,i} a_y z_{u,i} = m\,a_y\,h
```

Total lateral load transfer is fixed by $m$, $h$, and $a_y$. The design only
chooses where it goes:

- **Roll-centre heights** move load transfer between the geometric path (instant,
  no roll) and the elastic path (through roll). $z_{ra}/h_s$ is the roll
  analogue of anti percentage: it is the share of sprung-mass load transfer that
  bypasses the springs.
- **Springs and ARBs** only split the elastic part between the axles. With rigid
  tires and chassis:

```math
E_f = M_e\,\frac{K_{\phi,s,f}}{K_{\phi,s,f} + K_{\phi,s,r}}
```

### 3.4 Roll gradient

**Rigid tires.** Take moments about the roll axis, including the sprung weight
moving off-centre as the body rolls:

```math
\frac{\phi}{a_y} = \frac{m_s\,h'}{K_{\phi,s,f} + K_{\phi,s,r} - m_s g h'}
```

Multiply by $g$ for rad/g.

**With tires.** The springs and ARB carry only $E_i$, but the tires carry the whole
axle moment $G_i + E_i + U_i$. The body is rigid, so at each axle the
suspension roll plus the tire roll must equal the same φ:

```math
\phi = \frac{E_f}{K_{\phi,s,f}} + \frac{G_f + E_f + U_f}{K_{\phi,t,f}}
     = \frac{E_r}{K_{\phi,s,r}} + \frac{G_r + E_r + U_r}{K_{\phi,t,r}},
\qquad E_f + E_r = M_e
```

Write $C_i = 1/K_{\phi,s,i} + 1/K_{\phi,t,i}$ for each axle's compliance.
Solving gives:

```math
E_f = \frac{M_e\,C_r + (G_r + U_r)/K_{\phi,t,r} - (G_f + U_f)/K_{\phi,t,f}}{C_f + C_r}
```

φ then follows from the first equation, and
$\Delta F_{z,i} = (G_i + E_i + U_i)/t_i$. Rigid tires recover the §3.3 split.

When an axle's suspension is much stiffer than its tires, $C_i \to 1/K_{\phi,t,i}$
and extra ARB on that axle barely moves the balance. The tires set the
distribution. The gravity term adds $m_s g h'\phi$ to $M_e$; it is under 1% for
this car (§6).

## 4. Pitch

### 4.1 Pitch rate: heave–pitch stiffness

Take body heave $z$ (up) and pitch θ (nose down) at the sprung CG, with the front
axle $a_s$ ahead and the rear axle $b_s$ behind. With per-wheel rates $k_f$, $k_r$
(wheel rates for the suspension alone, ride rates to include tires):

```math
U = \tfrac12\,\mathbf q^{\mathsf T}\mathbf K\,\mathbf q, \qquad
\mathbf q = \begin{bmatrix} z \\ \theta \end{bmatrix}, \qquad
\mathbf K = 2\begin{bmatrix}
  k_f + k_r & k_r b_s - k_f a_s \\
  k_r b_s - k_f a_s & k_f a_s^2 + k_r b_s^2
\end{bmatrix}
```

- **Pitch rate about the CG:** $K_{\theta,CG} = 2(k_f a_s^2 + k_r b_s^2)$.
- **Heave–pitch coupling:** the off-diagonal term. It vanishes when
  $k_f a_s = k_r b_s$.
- **Elastic centre:** a pure pitch moment rotates the body about the point
  $e = (k_f a_s - k_r b_s)/(k_f + k_r)$ ahead of the CG. The pitch rate there is
  the lowest about any point:

```math
K_{\theta,e} = K_{\theta,CG} - 2(k_f + k_r)\,e^2 = \frac{2\,k_f\,k_r\,l^2}{k_f + k_r}
```

$K_{\theta,e}$ depends only on the axle rates and the wheelbase, not on where the
CG sits. It is set by the softer axle. Springs and motion ratios enter only
through $k_w = k_s/MR^2$. The ARB does not appear, because pitch moves both
wheels of an axle together.

### 4.2 Longitudinal load transfer and anti geometry

Longitudinal load transfer at each axle is set by statics alone:

```math
\Delta W = \frac{m\,a_x\,h}{l}
```

Anti percentage is the fraction of $\Delta W$ at an axle that goes through the
links instead of the springs. It depends on where the longitudinal force is
reacted:

- At the **contact patch** for outboard brakes ($z_P = 0$).
- At the **wheel centre** for torque reacted by the chassis: inboard brakes,
  halfshaft drive, and regen ($z_P = R$, the loaded radius).

Let the axle's side-view instant centre sit at height $e_i$ above ground, a
horizontal distance $d_i$ from the wheel towards the middle of the car. Then:

```math
\text{anti}_i = \frac{l}{h}\sum_k s_{i,k}\,\frac{e_i - z_{P,k}}{d_i}
```

Here $s_{i,k}$ is the share of total longitudinal force carried at axle $i$ by
mechanism $k$.

| Case | Formula |
| --- | --- |
| Front anti-dive, outboard brakes | $AD_f = \beta_f\,\dfrac{l}{h}\,\dfrac{e_f}{d_f}$ |
| Rear anti-lift, outboard friction + halfshaft regen | $AL_r = \dfrac{l}{h}\left[s_{fric}\,\dfrac{e_r}{d_r} + s_{regen}\,\dfrac{e_r - R}{d_r}\right]$, where $s_{fric} + s_{regen} = 1 - \beta_f$ |
| Rear anti-squat, RWD through halfshafts | $AS_r = \dfrac{l}{h}\,\dfrac{e_r - R}{d_r}$ |

The regen row is specific to this EV. Rear anti-lift changes with the regen split
even with the hardpoints fixed.

### 4.3 Pitch gradient

The springs see $(1-\text{anti})\,\Delta W$ and the tires see all of it. With axle
rates $K_i = 2k_{w,i}$ and $K_{t,i} = 2k_{t,i}$, braking gives a front drop and
a rear rise of:

```math
\delta_f = \Delta W\left(\frac{1 - AD_f}{K_f} + \frac{1}{K_{t,f}}\right), \qquad
\delta_r = \Delta W\left(\frac{1 - AL_r}{K_r} + \frac{1}{K_{t,r}}\right), \qquad
\theta = \frac{\delta_f + \delta_r}{l}
```

So the pitch gradient and effective pitch rate are:

```math
\frac{\theta}{a_x/g} = \frac{m g h}{l^2}
\left[\frac{1 - AD_f}{K_f} + \frac{1}{K_{t,f}} + \frac{1 - AL_r}{K_r} + \frac{1}{K_{t,r}}\right],
\qquad
K_{\theta,\mathrm{eff}} \equiv \frac{m\,a_x\,h}{\theta} = \frac{l^2}{[\,\cdots]}
```

- **Zero anti:** $K_{\theta,\mathrm{eff}}$ equals $K_{\theta,e}$ computed with ride rates.
  Longitudinal load transfer is a pure pitch couple, so the body pitches about
  the elastic centre.
- **Anti** multiplies each axle's spring compliance by $(1-\text{anti})$. At 100%
  on both axles only the tires deflect.
- **RWD acceleration:** the front carries no longitudinal force, so the front
  rises by $\delta_f = \Delta W\,(1/K_f + 1/K_{t,f})$. The rear squats by
  $\delta_r = \Delta W\,\big((1 - AS_r)/K_r + 1/K_{t,r}\big)$.
- **DS-010 inversion:** solving $\delta_f$ for $AD_f$ gives
  $AD_f = 1 - K_f\,(\delta_f/\Delta W - 1/K_{t,f})$. This is the inversion in
  [README.md](README.md) when its `K_w,f` and `K_t,f` are **axle** rates
  ($2k_w$, $2k_t$), because its `dW` is the whole-axle transfer. If per-wheel
  rates are plugged in, the spring term is off by 2×. The rear follows the same
  pattern.

## 5. Sensitivities

### 5.1 Rates

Each entry is the elasticity $\partial\ln Y / \partial\ln X$: percent change in
the rate per percent change in the lever. For $K_{\theta,e}$, $k_{other}$ is
the opposite axle's wheel rate.

| Lever $X$ | $K_{\phi,spr}$ | $K_{\phi,arb}$ | $K_{\theta,e}$ |
| --- | --- | --- | --- |
| Spring rate $k_s$ | 1 | — | $k_{other}/(k_f + k_r)$ |
| Motion ratio $MR$ (wheel/spring) | −2 | — | $-2\,k_{other}/(k_f + k_r)$ |
| Bar rate $k_{bar}$ | — | 1 | 0 |
| Drop-link ratio $IR_d$ (bellcrank $\lambda_d/\lambda_{rod}$) | — | 2 | 0 |
| ARB arm lever $\lambda_a$ | — | −2 | 0 |
| Track $t$ | 2 | 2 | 0 |
| Wheelbase $l$ | 0 | 0 | 2 |

In absolute terms, 1 N·m/rad of bar rate adds $r_{arb}^2$ N·m/rad of roll rate:
$\partial K_{\phi,arb}/\partial k_{bar} = r_{arb}^2$.

### 5.2 Balance

The front share of suspension roll rate is
$\chi = K_{\phi,f}/(K_{\phi,f} + K_{\phi,r})$, with:

```math
\frac{\partial\chi}{\partial K_{\phi,f}} = \frac{1 - \chi}{K_{\phi,f} + K_{\phi,r}}, \qquad
\frac{\partial\chi}{\partial K_{\phi,r}} = -\frac{\chi}{K_{\phi,f} + K_{\phi,r}}
```

Roll balance saturates as $\chi$ approaches 0 or 1. It saturates sooner when
tire or chassis compliance is significant (§3.2, §3.4).

### 5.3 Moment arms

These levers change the moment the rates carry, not the rates themselves.

- **CG height:** roll gradient ∝ $h'$. Pitch gradient ∝ $h/l^2$ at fixed anti
  percentage.
- **Roll-centre heights:** raising $z_{rc,i}$ lowers $h'$. It also moves
  load transfer at that axle into the instant geometric path.
- **Anti:**

```math
\frac{\partial}{\partial AD_f}\!\left(\frac{\theta}{a_x/g}\right) = -\frac{m g h}{l^2 K_f}, \qquad
\frac{\partial \delta_f}{\partial AD_f} = -\frac{\Delta W}{K_f}
```

  The same holds for $AL_r$ and $AS_r$ with the rear rate.
- **Anti with fixed hardpoints:** anti ∝ $l/h$, so the spring-borne pitch moment
  is $h\,(1 - AD_f) = h - \beta_f\,l\,e_f/d_f$. The anti geometry removes a
  fixed height from the pitch moment arm, and raising the CG adds dive
  one-for-one.

## 6. Worked example: 2027 WIP vehicle

**These numbers are illustrative only.** Inputs come from
[`vehicle_wip_2027_frontv19_rearv35.yml`](vehicle_wip_2027_frontv19_rearv35.yml)
and the FourPostEval baseline in `work/baseline/diag_metrics.csv` (gitignored).
Many of that file's values are placeholder carryovers. Its rear z datum is
unresolved, so every z-dependent number below ($h$, $h'$, roll-centre height,
gradients) is not a design conclusion.

| Input | Front | Rear |
| --- | --- | --- |
| Track $t$ (m) | 1.2446 | 1.1938 |
| Spring $k_s$ (N/m) | 26,269 | 43,782 |
| $MR$, FourPostEval `avg_motion_ratio_*` | 1.0944 | 1.7222 |
| $k_{bar}$ (N·m/rad) | 258.94 | 535.36 |
| $1/r_{arb}$, `avg_stabar_motion_ratio_*` | 0.07196 | 0.05653 |
| $k_t$ (N/m) | 98,947 | 98,947 |
| $z_{rc}$ (m), `avg_fbrc_equivalent_height_*` | 0.0285 | 0.0372 |

Mass properties: $m$ = 261.07 kg, $h$ = 0.2796 m, $a$ = 0.8003 m, $l$ = 1.5494 m.
The sprung mass including the driver is $m_s$ = 226.41 kg, $h_s$ = 0.2923 m,
$a_s$ = 0.8101 m. $K_c$ = 300,000 N·m/rad.

### Roll

Values are in N·m/rad unless marked. FourPostEval's values are in brackets.

| Result | Front | Rear |
| --- | --- | --- |
| Wheel rate $k_w$ (N/m) | 21,934 | 14,761 |
| $K_{\phi,spr}$ | 16,988 [16,988] | 10,519 [10,519] |
| $r_{arb}$ (rad/rad) | 13.90 | 17.69 |
| $K_{\phi,arb}$ | 50,004 [50,004] | 167,527 [167,540] |
| $K_{\phi,s}$ | 66,992 (1,169 N·m/deg) | 178,046 (3,107 N·m/deg) |
| $K_{\phi,t}$ | 76,636 | 70,508 |
| Series $K_{\phi,i}$ | 35,745 | 50,507 |

- **§3.1 reproduces FourPostEval.** The spring roll rates match exactly. The ARB
  rates match to 0.01%; FourPostEval averages $k_{bar}/MR_{arb}^2$ over the roll
  sweep instead of using the mean ratio.
- **Front share of roll rate:** 27.3% for the suspension alone, 30.2% with the
  flexible chassis (§3.2), and 41.4% with the tires in series.
- **Roll gradient:** 0.135 deg/g with rigid tires, where the gravity term changes
  it by 0.2%. With tires (§3.4) it is 0.439 deg/g, and front lateral load
  transfer rises from 30.2% to 42.0%. The tire roll rates are the same size as
  the front suspension's and well below the rear's. So the tires, not the bars,
  set this car's roll balance.
- **The rear ARB number is an artifact of placeholder geometry.** It supplies 94%
  of the rear suspension roll rate. The rear `stabar` pickup was carried over
  from the old bellcrank in `vehicles/current/vehicle.yml`, whose pivot was
  115 mm further forward and 256 mm lower. The pickup now sits 261 mm from the
  new pivot, against 76 mm for the rod and 74 mm for the shock. The
  perpendicular lever arms are 235, 66, and 58 mm. BobSim's `shark_import`
  rejects a carried-over stabar pickup more than 2× the longest other arm, for
  exactly this reason. Treat the
  rear ARB and every roll-balance figure above as placeholders until the pickup
  is defined in the new bellcrank's geometry.

### Pitch

FourPostEval reports essentially zero anti for this car: `avg_anti_dive_pct` =
0.12, `avg_anti_squat_pct` = −0.29.

| Result | Value |
| --- | --- |
| $K_{\theta,CG}$, wheel rates | 44,926 N·m/rad (784 N·m/deg) |
| Elastic centre | 0.187 m ahead of sprung CG |
| $K_{\theta,e}$, wheel rates | 42,363 N·m/rad (739 N·m/deg) |
| $K_{\theta,e}$, ride rates = $K_{\theta,\mathrm{eff}}$ at zero anti | 35,951 N·m/rad (627 N·m/deg) |
| $\Delta W$ per g | 462.2 N |
| Braking, zero anti: front drop / rear rise | 12.9 / 18.0 mm/g, pitch gradient 1.14 deg/g |
| Braking, $AD_f = AL_r = 30\%$ | 9.7 / 13.3 mm/g, 0.85 deg/g, $K_{\theta,\mathrm{eff}}$ = 842 N·m/deg |
| RWD acceleration, $AS_r = 30\%$ | front rise 12.9 mm/g, rear squat 13.3 mm/g, 0.97 deg/g |

Each 10% of front anti-dive removes $\Delta W \times 0.1/K_f$ ≈ 1.05 mm/g of
front dive.

## 7. Limits of the linear model

- **Rising rate.** $k_w = k_s/MR^2$ drops the $F_s\,\partial IR_s/\partial z_w$
  term (§2.1). FourPostEval also uses a motion ratio averaged over heave for the
  spring roll rate.
- **ARB compliance.** $k_{bar}$ should be the effective rate at the arms,
  including arm bending and drop-link or bearing compliance. Drop-link angle
  changes with travel.
- **Tires.** $k_t$ varies with load, pressure, and camber. Unsprung-mass load
  transfer bypasses the springs, and §4 folds it into $\Delta W$.
- **Kinematics.** Roll-centre heights, anti percentages, and installation ratios
  all move with travel. Jacking forces add heave that is not included here. Use
  FourPostSim across the travel range, and treat this note as the local
  linearisation.
- **Aero.** Downforce changes the static heave loads, not the rates. The current
  aero map has no roll dependence (see [README.md](README.md), Known Issues).
- **Dynamics.** The rates set the natural frequencies, and dampers set the
  transient angular rates. For roll,
  $f_\phi \approx \frac{1}{2\pi}\sqrt{K_\phi/(I_{xx,s} + m_s h'^2)}$. For heave
  and pitch, solve $\det(\mathbf K - \omega^2\,\mathrm{diag}(m_s, I_{yy,s})) = 0$
  with $\mathbf K$ from §4.1 using ride rates.

## References

- W. F. Milliken and D. L. Milliken, *Race Car Vehicle Dynamics*, SAE, 1995:
  ch. 16 (ride and roll rates), ch. 17 (suspension geometry and anti features),
  ch. 18 (wheel loads).
- BobSim `_3_StandardSim/FourPostEval/four_post_eval_sim.py`: motion-ratio fits
  and roll-stiffness metrics.
- BobSim `_0_Utils/dyn_py/parameters.py` and `models.py`: wheel rate, ARB force
  law, and roll-stiffness distribution in the reduced-order models.
