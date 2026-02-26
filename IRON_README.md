# ⊕ IRON COMMAND — Battle Tank Engineering Suite

> Multi-file tank engineering & live HUD project. Your phone is the tank.

![C++17](https://img.shields.io/badge/C%2B%2B-17-orange) ![JavaScript](https://img.shields.io/badge/JavaScript-ES2022-yellow) ![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-Ready-green)

---

## 📁 Project Structure

```
iron-command/
├── index.html           ← Hub launcher (start here)
├── tank-live.html       ← Mobile HUD — your device IS the tank
├── commander.html       ← Desktop commander station
├── physics.js           ← Shared physics engine (JS)
├── ballistics.h         ← C++ physics header (single-include)
├── main_ballistics.cpp  ← C++ CLI ballistics calculator
└── README.md
```

---

## 🌐 GitHub Pages Deploy (30 seconds)

1. Create a new GitHub repository
2. Upload **all 7 files** to the repo root
3. Go to **Settings → Pages → Branch: main → Save**
4. Your URL: `https://yourusername.github.io/repo-name/`
5. Open `tank-live.html` **on your phone** (HTTPS required for sensors)

---

## 📱 tank-live.html — Your Device IS the Tank

| Device sensor | Tank function |
|---|---|
| **Gyroscope** | Gun elevation (tilt forward/back) |
| **Compass** | Turret bearing (real heading tape) |
| **GPS** | Live battlefield coordinates + minimap trail |
| **Accelerometer** | Speed estimate + **shake = impact hit** |
| **Battery API** | Fuel gauge (low battery = critical fuel) |
| **Vibration API** | Haptic fire kick + hit feedback |

### Sensor Requirements
- **HTTPS required** — sensors don't work on `file://` or `http://`
- **iOS 13+** — tap "Activate Systems" to grant permissions
- **Android Chrome** — permissions auto-granted on HTTPS
- **GPS** — allow location when browser asks

---

## 🔬 C++ Ballistics Engine

### Build

```bash
# Linux / macOS
g++ -O2 -std=c++17 -o ironballistics main_ballistics.cpp

# Windows (MSVC)
cl /O2 /std:c++17 main_ballistics.cpp /Fe:ironballistics.exe

# MinGW (Windows)
g++ -O2 -std=c++17 -o ironballistics.exe main_ballistics.cpp
```

### Quick mode
```bash
./ironballistics --quick 120 1650 apfsds 2000
# apfsds 120mm @ 1650m/s:
#   Range:  2000.00 m
#   ToF:    1.23 s
#   Vimp:   1541.20 m/s
#   Drop:   74.12 mm
```

### Interactive menu
```bash
./ironballistics
# 1. Ballistics Calculator     — trajectory + pen vs range table
# 2. Armor Penetration         — LOS, slope effect, armor package
# 3. Engine Performance        — power/weight, range, ground pressure
# 4. Quick Fire Control        — full FCS solution with lead
# 5. Preset Scenarios          — WWII through modern
# 6. Atmosphere Model          — ISA density, sound speed table
```

---

## ⚡ physics.js API Reference

```html
<!-- Include in any HTML file -->
<script src="physics.js"></script>
```

```javascript
// 1. Trajectory (RK4 integration)
const traj = IronPhysics.ballistics.trajectory({
  v0: 1650,           // muzzle velocity m/s
  mass: 4.8,          // projectile mass kg
  caliber: 0.120,     // caliber m
  elevDeg: 3.5,       // gun elevation degrees
  type: 'apfsds',     // 'apfsds'|'heat'|'he'|'hesh'
  maxRange: 4000,     // max range to solve to
  dt: 0.005,          // time step (smaller = more accurate)
});
// → { points[], maxRange, timeOfFlight, impactVelocity, drop }

// 2. APFSDS penetration (Alekseevski-Tate)
const pen = IronPhysics.ballistics.apfsdsPenetration({
  velocity: 1541,         // impact velocity m/s
  rodLength: 0.64,        // rod length m (L/D≈22 for 120mm)
  rodDiameter: 0.027,     // rod diameter m
  rodDensity: 17500,      // WHA kg/m³ (default)
});
// → { penetration_mm, hydro_limit_mm, LD_ratio }

// 3. HEAT penetration (Munroe effect)
const heat = IronPhysics.ballistics.heatPenetration({
  caliber: 0.120,         // caliber m
  liner: 'copper',        // 'copper'|'steel'|'tantalum'
  standoff: 0.48,         // standoff distance m (optional)
});
// → { penetration_mm, liner_factor, standoff_factor }

// 4. Slope effect
const slope = IronPhysics.ballistics.slopeEffect(250, 68);
// → { los_mm, norm_mult, tate_factor, effective_mm }

// 5. Engine performance
const perf = IronPhysics.engine.performance({
  power_hp: 1500,
  mass_tonnes: 68,
  fuelCap_l: 1909,
  trackWidth_mm: 635,
  contactLen_mm: 4000,
  type: 'turbine',
});
// → { topSpeed_kmh, range_km, ptw_hp_tonne, groundPressure_kg_cm2, ... }

// 6. Sensor fusion (for mobile HUD)
const heading = IronPhysics.sensors.compassEma(prev, raw, 0.12);
const gun = IronPhysics.sensors.orientationToGun(alpha, beta, gamma);
const impact = IronPhysics.sensors.detectImpact(ax, ay, az, prevG, 1.8);

// 7. Fire control
const fcs = IronPhysics.combat.fireControl({
  elevDeg: 3.5,
  ammoType: 'apfsds',
  targetSpeed: 10,
  targetBearing: 90,
});
// → { range_m, timeOfFlight_s, impactVelocity, lead_m }
```

### Node.js
```javascript
const IronPhysics = require('./physics.js');
const result = IronPhysics.ballistics.trajectory({ v0:1650, ... });
```

---

## 🧮 Physics Algorithms

### Trajectory — 4th-Order Runge-Kutta
The trajectory is numerically integrated using RK4 with the drag equation:

```
Fd = ½ρCd·A·v²
```

Where `ρ` is computed from the ISA barometric formula, and `Cd` varies by Mach number using a piecewise approximation fitted to ballistic data.

### APFSDS Penetration — Alekseevski-Tate
Based on the Alekseevski-Tate model with Tate strength corrections:

```
P = L · √(ρP/ρT) · f(v)
```

Where `f(v)` corrects for the subhydrodynamic regime at lower velocities.

### Slope Effect — Tate Obliquity Factor
```
LOS = t / cos(θ)
Tate factor = 1 + 0.015·(θ - 55°)^1.5  [for θ > 55°]
```

### Atmosphere — ISA Barometric Formula
```
ρ(h) = p(h) / (R·T(h))
T(h) = 288.15 - 0.0065·h
p(h) = 101325 · (T/T0)^(g/L·R)
```

---

## 📊 Browser Sensor Compatibility

| Feature | Chrome Android | Safari iOS | Firefox | Desktop |
|---------|---------------|------------|---------|---------|
| DeviceOrientation | ✅ | ✅ (perm) | ✅ | ⚠ limited |
| DeviceMotion | ✅ | ✅ (perm) | ✅ | ⚠ limited |
| Geolocation | ✅ | ✅ | ✅ | ✅ |
| Battery API | ✅ | ❌ | ❌ | ✅ Chrome |
| Vibration | ✅ | ❌ | ✅ | ❌ |
| Wake Lock | ✅ | ✅ | ✅ | ✅ |

> **iOS note:** Tap "Activate Systems" button to trigger the permission dialog.
> DeviceOrientation/Motion require user gesture in iOS 13+.

---

## 🎯 Architecture

```
physics.js (shared math core)
    ↑           ↑
commander.html  tank-live.html
(desktop FCS)   (mobile HUD)
    ↕               ↕
[Chart.js]      [Device Sensors]
                [GPS/Gyro/Accel/Battery]

ballistics.h (C++ mirror of physics.js)
    ↑
main_ballistics.cpp (CLI tool)
```

---

*IRON COMMAND v2.0 — Battle Tank Engineering Suite*
