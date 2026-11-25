# Bouncy-Ball 3.0 — C++20 & SFML 2D Physics Lab

An evolution of the September 2025 `Bouncy-Ball-2.0` C++ experiment into a real-time, SI-unit **2D rigid-disk physics sandbox** built with **C++20** and **SFML 3**.

The original 91-line Raylib experiment is preserved in [`legacy/main_2025_raylib.cpp`](legacy/main_2025_raylib.cpp) for side-by-side comparison.

## What Improved Over 2.0

1. **Exact Frame-Rate-Independent SI Physics (`meters`, `seconds`, `kg`, `Joules`)**
   - Fixes missing `dt` in position integration (`pos.y += velocity` in 2.0 was frame-rate dependent and mixed meters with pixels).
   - Separates physical `mass` ($\text{kg}$, derived from areal density $\rho \pi R^2$) from geometric `radius` ($R$ in meters) and rotational inertia ($I = \frac{1}{2} m R^2$).
   - Uses a $+y$-up world coordinate system (`1.0 m = 100 px`) so gravitational potential energy $U_g = m g h$ decreases as the ball falls instead of increasing downward.
2. **Sub-Stepped Velocity Verlet + Within-Step Time-of-Impact (TOI) Restitution**
   - Solves exact contact fraction $\alpha \in [0, 1]$ during floor impacts so energy is conserved to $< 0.2\%$ across dozens of bounces at $e = 1.0$.
   - Completes the original `// TODO GET FINAL HEIGHT` with live measured `Last Bounce Apex`, `Initial Drop h0`, and `dApex` guides.
3. **2D Multi-Ball Rigid-Disk Collisions, Spin, Aerodynamics & Custom Ramps**
   - Pairwise impulse solver with Coulomb tangential friction (transferring spin $\omega$ on oblique bounces) and positional slop correction.
   - Quadratic aerodynamic drag ($\mathbf{F}_d = -\frac{1}{2}\rho C_d A \|\mathbf{v}\|\mathbf{v}$) and **Magnus spin lift** ($\mathbf{F}_M \propto \omega \times \mathbf{v}$).
   - Drawable capsule ramps/platforms, spring-damper mouse grabbing, and slingshot trajectory prediction.
4. **Live Stacked Energy Oscilloscope, Squash-and-Stretch & Procedural Audio**
   - Real-time stacked plot of $U_g$, $K_{\text{trans}}$, $K_{\text{rot}}$, and $E_{\text{total}}$.
   - Area-preserving impact squash-and-stretch, velocity trails, collision sparks, and zero-asset procedural stereo bounce synthesis.

## Build & Run (macOS + Homebrew SFML 3)

```bash
make        # Builds ./bouncy_ball
make test   # Runs headless C++20 physics unit tests
./bouncy_ball
```

## Controls

| Input | Action |
|---|---|
| **Left-Drag (empty space)** | Slingshot-launch a new ball with live trajectory preview |
| **Mouse Wheel / `Q` `E`** | Adjust spawn radius / initial spin $\omega$ |
| **Left-Drag (on ball)** | Grab & fling ball with spring-damper cursor |
| **Right-Drag / Right-Click** | Draw bouncy ramp / delete ball or ramp under cursor |
| **`1` – `6`** | Select material (`Lab SkyBlue`, `Superball`, `Steel`, `Tennis`, `Basketball`, `Heavy Lead`) |
| **`Up` / `Down`** | Adjust global restitution $e \in [0.00, 1.00]$ |
| **`[` / `]`** | Cycle gravity (`Earth`, `Moon`, `Mars`, `Jupiter`, `Zero-G`) |
| **`D`** | Toggle aerodynamic drag & Magnus spin effect |
| **`P`** | Pure conservation mode ($e = 1.0$, zero drag, zero friction) |
| **`Space` / `S`** | Pause & resume / single-step one frame |
| **`B` / `R` / `C` / `X`** | Chaos burst / Reset classic 2.0 drop / Clear balls / Clear ramps |
| **`G` / `T` / `V` / `A` / `M` / `H`** | Toggle Grid, Trails, Vectors, Apex lines, Mute, Help |
