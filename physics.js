/**
 * ═══════════════════════════════════════════════════════════════
 * IRON COMMAND — Shared Physics Engine
 * physics.js  v2.0
 *
 * Ballistics, armor penetration, engine performance, and sensor
 * math. All algorithms mirror the C++ implementation in
 * ballistics.cpp / ballistics.h for portability.
 *
 * Usage:  <script src="physics.js"></script>
 *         const result = IronPhysics.ballistics.apfsds({ ... });
 * ═══════════════════════════════════════════════════════════════
 */

'use strict';

const IronPhysics = (() => {

  /* ════════════════════════════════════════════════════════════
     CONSTANTS
  ════════════════════════════════════════════════════════════ */
  const G          = 9.80665;      // standard gravity m/s²
  const RHO_SEA    = 1.225;        // air density kg/m³ at sea level
  const R_AIR      = 287.058;      // specific gas constant J/(kg·K)

  const MATERIALS = {
    steel:     { density: 7850,  hardness: 200  },
    tungsten:  { density: 19300, hardness: 2600 },
    DU:        { density: 19050, hardness: 200  }, // depleted uranium
    wha:       { density: 17500, hardness: 400  }, // WHA tungsten heavy alloy
    lead:      { density: 11340, hardness: 5    },
    aluminum:  { density: 2700,  hardness: 100  },
  };

  const ARMOR_MULT = {
    rha:       1.00,  // rolled homogeneous armor baseline
    cha:       0.94,  // cast, slightly lower
    hha:       1.30,  // high-hardness armor
    composite: 2.40,  // advanced composite vs KE
    chobham:   3.00,  // Chobham/Dorchester vs KE
    du_comp:   3.50,  // DU composite (M1A2 SEPv3)
    nera:      1.25,  // non-explosive reactive vs KE
    era_ke:    0.40,  // ERA effectiveness vs KE (additive bonus)
    era_heat:  1.80,  // ERA effectiveness vs HEAT (multiply by blocked)
  };

  /* ════════════════════════════════════════════════════════════
     ATMOSPHERE MODEL
     Simple ISA (International Standard Atmosphere)
  ════════════════════════════════════════════════════════════ */
  const atmosphere = {
    /**
     * Air density at altitude h (metres) using barometric formula.
     * @param {number} h - altitude in metres
     * @returns {number} density in kg/m³
     */
    density(h) {
      const T0 = 288.15;  // sea-level temperature K
      const L  = 0.0065;  // temperature lapse rate K/m
      const T  = T0 - L * h;
      const p  = 101325 * Math.pow(T / T0, G / (L * R_AIR));
      return p / (R_AIR * T);
    },

    /**
     * Speed of sound at altitude h.
     * @param {number} h - altitude in metres
     * @returns {number} speed of sound m/s
     */
    soundSpeed(h) {
      const T = 288.15 - 0.0065 * h;
      return Math.sqrt(1.4 * R_AIR * T);
    },

    /**
     * Mach number of a projectile.
     * @param {number} v - velocity m/s
     * @param {number} h - altitude m
     * @returns {number} Mach number
     */
    mach(v, h = 0) {
      return v / this.soundSpeed(h);
    }
  };

  /* ════════════════════════════════════════════════════════════
     BALLISTICS MODULE
  ════════════════════════════════════════════════════════════ */
  const ballistics = {

    /**
     * Compute drag coefficient as function of Mach number.
     * Uses piecewise approximation of experimental Cd-Mach curves.
     * @param {number} mach - Mach number
     * @param {string} type - 'apfsds'|'heat'|'he'|'ap'|'atgm'
     * @returns {number} drag coefficient
     */
    dragCoeff(mach, type = 'apfsds') {
      if (type === 'apfsds') {
        // Long rod penetrator: very low drag, transonic spike
        if (mach < 0.8)  return 0.020;
        if (mach < 1.0)  return 0.020 + 0.030 * ((mach - 0.8) / 0.2);
        if (mach < 1.2)  return 0.050 - 0.020 * ((mach - 1.0) / 0.2);
        if (mach < 3.0)  return 0.030 - 0.010 * ((mach - 1.2) / 1.8);
        return 0.020;
      }
      if (type === 'heat' || type === 'atgm') {
        // Fin-stabilised HEAT: higher drag body
        if (mach < 0.8)  return 0.15;
        if (mach < 1.0)  return 0.15 + 0.20 * ((mach - 0.8) / 0.2);
        if (mach < 1.2)  return 0.35 - 0.15 * ((mach - 1.0) / 0.2);
        return 0.20;
      }
      if (type === 'he' || type === 'hesh') {
        // Blunt nose: higher base drag
        return mach < 1 ? 0.30 : 0.35;
      }
      // Generic AP
      return mach < 1 ? 0.25 : 0.30;
    },

    /**
     * Numerically integrate projectile trajectory using RK4.
     * Returns array of time-stepped positions, velocities, and derived values.
     *
     * @param {Object} p - projectile parameters
     *   p.v0       {number} muzzle velocity m/s
     *   p.mass     {number} projectile mass kg
     *   p.caliber  {number} caliber m (e.g. 0.120)
     *   p.elevDeg  {number} elevation angle degrees
     *   p.type     {string} 'apfsds'|'heat'|'he'|'ap'
     *   p.altitude {number} firing altitude m (optional, default 0)
     *   p.maxRange {number} max range to integrate to m
     *   p.dt       {number} time step s (optional, default 0.001)
     * @returns {Object} { points, maxRange, timeOfFlight, impactVelocity, impactAngle }
     */
    trajectory(p) {
      const dt   = p.dt       || 0.001;
      const h0   = p.altitude || 0;
      const elev = p.elevDeg  * Math.PI / 180;
      const A    = Math.PI * (p.caliber / 2) ** 2;  // cross-sectional area

      // Initial conditions
      let x  = 0,  y  = h0;
      let vx = p.v0 * Math.cos(elev);
      let vy = p.v0 * Math.sin(elev);
      let t  = 0;

      const points = [{x, y, vx, vy, t, v: p.v0}];

      const deriv = (x, y, vx, vy) => {
        const v   = Math.sqrt(vx*vx + vy*vy);
        const alt = Math.max(0, y);
        const rho = atmosphere.density(alt);
        const mac = atmosphere.mach(v, alt);
        const Cd  = this.dragCoeff(mac, p.type);
        const Fd  = 0.5 * rho * Cd * A * v * v;  // drag force N
        const ax  = -(Fd / p.mass) * (vx / v);
        const ay  = -(Fd / p.mass) * (vy / v) - G;
        return {dvx: ax, dvy: ay};
      };

      let step = 0;
      const MAX_STEPS = 200000;

      while (step < MAX_STEPS && x < (p.maxRange || 8000) && y >= 0) {
        // RK4
        const k1 = deriv(x, y, vx, vy);
        const k2 = deriv(x + vx*dt/2, y + vy*dt/2, vx + k1.dvx*dt/2, vy + k1.dvy*dt/2);
        const k3 = deriv(x + vx*dt/2, y + vy*dt/2, vx + k2.dvx*dt/2, vy + k2.dvy*dt/2);
        const k4 = deriv(x + vx*dt, y + vy*dt, vx + k3.dvx*dt, vy + k3.dvy*dt);

        vx += (k1.dvx + 2*k2.dvx + 2*k3.dvx + k4.dvx) * dt / 6;
        vy += (k1.dvy + 2*k2.dvy + 2*k3.dvy + k4.dvy) * dt / 6;
        x  += vx * dt;
        y  += vy * dt;
        t  += dt;
        step++;

        // Store every 10th point for performance
        if (step % 10 === 0) {
          points.push({ x, y, vx, vy, t, v: Math.sqrt(vx*vx + vy*vy) });
        }
      }

      const last  = points[points.length - 1];
      const impV  = Math.sqrt(last.vx**2 + last.vy**2);
      const impAng= Math.atan2(-last.vy, last.vx) * 180 / Math.PI;

      return {
        points,
        maxRange:       x,
        timeOfFlight:   t,
        impactVelocity: impV,
        impactAngle:    impAng,
        drop:           y - h0,   // negative = dropped below firing height
      };
    },

    /**
     * APFSDS long-rod penetration model.
     * Based on Lanz-Odermatt and simplified Alekseevski-Tate.
     *
     * @param {Object} p
     *   p.velocity  {number} impact velocity m/s
     *   p.rodDensity {number} penetrator density kg/m³ (default WHA 17500)
     *   p.rodLength  {number} rod length m
     *   p.rodDiameter {number} rod diameter m
     *   p.targetDensity {number} target density kg/m³ (default RHA 7850)
     * @returns {Object} { penetration_m, penetration_mm }
     */
    apfsdsPenetration(p) {
      const rhoP = p.rodDensity    || 17500;
      const rhoT = p.targetDensity || 7850;
      const L    = p.rodLength;
      const d    = p.rodDiameter;
      const v    = p.velocity;
      const LD   = L / d;

      // Hydrodynamic limit: P = L * sqrt(rhoP/rhoT)
      const P_hydro = L * Math.sqrt(rhoP / rhoT);

      // Velocity correction (Tate correction for finite velocity)
      // At very high velocity → hydrodynamic; at lower → subhydrodynamic
      const Yp    = 1.5e9;  // penetrator strength Pa (WHA)
      const Yt    = 1.0e9;  // target strength Pa (RHA)
      const u     = v / Math.sqrt(2 * Yt / rhoT);  // reduced velocity
      const corr  = Math.max(0.5, Math.min(1.0, u));

      const pen_m = P_hydro * corr;
      return {
        penetration_m:  pen_m,
        penetration_mm: pen_m * 1000,
        LD_ratio:        LD,
        hydroLimit_mm:  P_hydro * 1000,
        velocityCorr:   corr,
      };
    },

    /**
     * Munroe effect HEAT jet penetration.
     * @param {Object} p
     *   p.caliber  {number} caliber m
     *   p.liner    {string} 'copper'|'steel'|'tantalum' (default copper)
     *   p.standoff {number} standoff distance m
     *   p.velocity {number} detonation/jet velocity m/s (optional)
     * @returns {Object}
     */
    heatPenetration(p) {
      const liners = { copper: 0.72, steel: 0.85, tantalum: 0.95 };
      const ef     = liners[p.liner] || 0.72;
      // Empirical: P ≈ 5–7 × caliber with good standoff
      const optStandoff = p.caliber * 4.0;
      const sdFactor = p.standoff
        ? Math.max(0.4, 1 - Math.abs(p.standoff - optStandoff) / (optStandoff * 3))
        : 1.0;
      const pen_m  = p.caliber * 6.5 * ef * sdFactor;
      return {
        penetration_mm: pen_m * 1000,
        liner_factor:   ef,
        standoff_factor: sdFactor,
        optimal_standoff_mm: optStandoff * 1000,
      };
    },

    /**
     * Slope normalisation: LOS thickness and Tate obliquity factor.
     * @param {number} thickness_mm - actual plate thickness
     * @param {number} slope_deg    - slope angle from vertical (0 = flat plate)
     * @returns {Object}
     */
    slopeEffect(thickness_mm, slope_deg) {
      const theta = slope_deg * Math.PI / 180;
      const los   = thickness_mm / Math.cos(theta);      // line-of-sight thickness
      const norm  = 1 / Math.cos(theta);                 // normalisation multiplier

      // Tate obliquity factor (additional resistance at high angles)
      // Empirically significant above ~55°
      const tateFactor = slope_deg > 55
        ? 1 + 0.015 * (slope_deg - 55) ** 1.5
        : 1.0;

      return {
        los_mm:      los,
        norm_mult:   norm,
        tate_factor: tateFactor,
        effective_mm: los * tateFactor,
      };
    },

    /**
     * Full armor package effective protection calculator.
     * @param {Object} layers - array of {material, thickness_mm, slope_deg}
     * @param {string} roundType - 'ke'|'heat'|'hesh'
     * @returns {number} effective RHA mm
     */
    armorPackage(layers, roundType = 'ke') {
      return layers.reduce((total, layer) => {
        const mult = ARMOR_MULT[layer.material] || 1.0;
        const slope = this.slopeEffect(layer.thickness_mm, layer.slope_deg || 0);
        const eff  = slope.effective_mm * mult;
        // HEAT only partially defeated by KE-rated layers
        const factor = roundType === 'heat'
          ? (layer.material === 'composite' || layer.material === 'chobham' ? 3.5 : 1.0)
          : 1.0;
        return total + eff * factor;
      }, 0);
    },
  };

  /* ════════════════════════════════════════════════════════════
     ENGINE MODULE
  ════════════════════════════════════════════════════════════ */
  const engine = {

    /**
     * Simple turbine/diesel power curve model.
     * @param {Object} p
     *   p.maxPower_kw  {number} peak power kW
     *   p.maxTorque_nm {number} peak torque Nm
     *   p.redline_rpm  {number} max RPM
     *   p.type         {string} 'turbine'|'diesel'|'hybrid'
     * @returns {Function} rpm → {power_kw, torque_nm, efficiency}
     */
    powerCurve(p) {
      return function(rpm) {
        const r = rpm / p.redline_rpm;
        let pNorm, tNorm;

        if (p.type === 'turbine') {
          // Turbines: flat torque, drops off at very high RPM
          tNorm = r < 0.3 ? r / 0.3 : (r < 0.85 ? 1.0 : 1.0 - (r - 0.85) / 0.15);
          pNorm = r * tNorm;
        } else if (p.type === 'diesel') {
          // Diesel: peak torque at ~1500 RPM, peak power at higher
          tNorm = r < 0.5 ? (0.7 + 0.3 * (r / 0.5)) : (1.0 - 0.5 * ((r - 0.5) / 0.5));
          pNorm = r * tNorm;
        } else {
          // Hybrid: near-flat power
          tNorm = r < 0.2 ? r / 0.2 : (1.0 - 0.3 * r);
          pNorm = r * tNorm;
        }

        const efficiency = p.type === 'turbine' ? 0.22
          : p.type === 'diesel' ? 0.42
          : 0.55;

        return {
          power_kw:   p.maxPower_kw  * Math.max(0, pNorm),
          torque_nm:  p.maxTorque_nm * Math.max(0, tNorm),
          efficiency,
          rpm,
        };
      };
    },

    /**
     * Compute vehicle performance stats from engine + vehicle params.
     * @param {Object} p
     *   p.power_hp       {number} peak power hp
     *   p.mass_tonnes    {number} combat mass tonnes
     *   p.fuelCap_l      {number} fuel capacity litres
     *   p.trackWidth_mm  {number} single track width mm
     *   p.contactLen_mm  {number} track contact length mm
     *   p.type           {string} engine type
     * @returns {Object}
     */
    performance(p) {
      const kw    = p.power_hp * 0.7457;
      const ptw   = p.power_hp / p.mass_tonnes;  // hp/tonne

      // Speed: empirical from PTW (good fit to real data)
      // v_max ≈ 5.5 × PTW^0.55 for wheeled, 4.5 × PTW^0.55 for tracked
      const topSpeed_kmh  = Math.min(90, 4.5 * Math.pow(ptw, 0.55));

      // Fuel consumption L/km (varies by type)
      const consump = { turbine: 3.8, diesel: 2.6, hybrid: 1.9 }[p.type] || 2.8;

      // Range
      const range_km = p.fuelCap_l / consump;

      // Ground pressure
      const area_cm2 = 2 * (p.trackWidth_mm / 10) * (p.contactLen_mm / 10);
      const gp_kg_cm2 = (p.mass_tonnes * 1000) / area_cm2;

      // 0→32km/h time (estimate)
      const force_N = kw * 1000 / (topSpeed_kmh / 3.6);
      const accel = force_N / (p.mass_tonnes * 1000);
      const time032 = (32 / 3.6) / accel;

      // Thermal output W (waste heat)
      const eta    = { turbine: 0.22, diesel: 0.42, hybrid: 0.55 }[p.type] || 0.35;
      const waste_kw = kw * (1 - eta);

      return {
        power_kw:       kw,
        ptw_hp_tonne:   ptw,
        topSpeed_kmh,
        range_km,
        groundPressure_kg_cm2: gp_kg_cm2,
        time_0_32_s:    time032,
        wasteHeat_kw:   waste_kw,
        fuelConsump_l_km: consump,
      };
    }
  };

  /* ════════════════════════════════════════════════════════════
     SENSOR FUSION MODULE
     Used by tank-live.html for real device sensor processing
  ════════════════════════════════════════════════════════════ */
  const sensors = {

    /**
     * Exponential moving average filter.
     * @param {number} prev  - previous filtered value
     * @param {number} raw   - new raw measurement
     * @param {number} alpha - smoothing factor 0–1 (lower = smoother)
     * @returns {number}
     */
    ema(prev, raw, alpha = 0.15) {
      return prev + alpha * (raw - prev);
    },

    /**
     * Wrap-aware EMA for compass angles (handles 0/360 boundary).
     * @param {number} prev  - previous filtered heading degrees
     * @param {number} raw   - new raw heading degrees
     * @param {number} alpha - smoothing factor
     * @returns {number}
     */
    compassEma(prev, raw, alpha = 0.12) {
      let delta = raw - prev;
      if (delta >  180) delta -= 360;
      if (delta < -180) delta += 360;
      let out = prev + alpha * delta;
      if (out <   0) out += 360;
      if (out >= 360) out -= 360;
      return out;
    },

    /**
     * Convert device orientation to gun elevation and turret bearing.
     * Assumes device is held portrait, pointed at target.
     *
     * @param {number} alpha - compass bearing 0–360°
     * @param {number} beta  - device pitch -180 to +180°
     * @param {number} gamma - device roll -90 to +90°
     * @returns {Object} { heading, elevation_deg, roll_deg, stabilised_roll }
     */
    orientationToGun(alpha, beta, gamma) {
      const heading   = alpha;                 // compass heading = turret bearing
      const elevation = -beta;                 // forward tilt = depression, back = elevation
      const roll      = gamma;                 // device roll = tank hull roll

      // Stabilised reticle (FCS correction): counter-rotate by half roll
      const stabRoll  = -roll * 0.5;

      return { heading, elevation_deg: elevation, roll_deg: roll, stab_roll: stabRoll };
    },

    /**
     * Detect sudden impact from accelerometer data.
     * Uses a high-pass filter to detect transients above threshold.
     *
     * @param {number} ax, ay, az - current acceleration m/s²
     * @param {number} prevG      - previous G reading
     * @param {number} threshold  - impact threshold G (default 1.8)
     * @returns {Object} { isImpact, gForce, delta }
     */
    detectImpact(ax, ay, az, prevG, threshold = 1.8) {
      const mag    = Math.sqrt(ax**2 + ay**2 + az**2);
      const gForce = mag / G;
      const delta  = Math.abs(gForce - prevG);
      return {
        isImpact: delta > threshold,
        gForce,
        delta,
        mag,
      };
    },

    /**
     * Estimate vehicle speed from accelerometer when GPS is unavailable.
     * Simple dead-reckoning via lateral acceleration integration.
     *
     * @param {number} currentSpeed_ms - current speed m/s
     * @param {number} ax, ay          - horizontal accelerations
     * @param {number} dt              - time step s
     * @returns {number} new speed estimate m/s
     */
    accelSpeed(currentSpeed_ms, ax, ay, dt) {
      const lateral = Math.sqrt(ax**2 + ay**2);
      const bias    = 0.08;  // typical sensor bias m/s²
      let   speed   = currentSpeed_ms;
      if (lateral > bias) {
        speed += (lateral - bias) * dt;
      } else {
        speed = Math.max(0, speed - 0.3 * dt);  // deceleration
      }
      return Math.min(20, speed);  // cap at 20 m/s ≈ 72 km/h
    },

    /**
     * Cardinal direction name from bearing.
     * @param {number} heading - degrees 0–360
     * @returns {string}
     */
    cardinalDir(heading) {
      const dirs = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
                    'S','SSW','SW','WSW','W','WNW','NW','NNW'];
      return dirs[Math.round(heading / 22.5) % 16];
    },
  };

  /* ════════════════════════════════════════════════════════════
     COMBAT MODULE
     Fire-control system computations
  ════════════════════════════════════════════════════════════ */
  const combat = {

    /**
     * Fire control solution:
     * Given current elevation and ammo type, return estimated
     * range, time of flight, and required lead for moving target.
     *
     * @param {Object} p
     *   p.elevDeg       {number} gun elevation degrees
     *   p.ammoType      {string} 'apfsds'|'heat'|'he'
     *   p.targetSpeed   {number} target speed m/s (optional)
     *   p.targetBearing {number} angle between gun and target motion (optional)
     * @returns {Object}
     */
    fireControl(p) {
      const ammoParams = {
        apfsds:   { v0: 1650, mass: 4.8,   caliber: 0.120, type: 'apfsds' },
        heat:     { v0: 1050, mass: 10.5,  caliber: 0.120, type: 'heat'   },
        he:       { v0: 850,  mass: 14.2,  caliber: 0.120, type: 'he'     },
        hesh:     { v0: 760,  mass: 15.0,  caliber: 0.120, type: 'hesh'   },
      };
      const ammo = ammoParams[p.ammoType] || ammoParams.apfsds;
      const traj = ballistics.trajectory({
        ...ammo,
        elevDeg: p.elevDeg,
        maxRange: 6000,
        dt: 0.005,
      });

      // Lead calculation: how far ahead to aim for a moving target
      let lead_m = 0;
      if (p.targetSpeed && p.targetBearing !== undefined) {
        const perpSpeed = p.targetSpeed * Math.sin(p.targetBearing * Math.PI / 180);
        lead_m = perpSpeed * traj.timeOfFlight;
      }

      return {
        range_m:         traj.maxRange,
        timeOfFlight_s:  traj.timeOfFlight,
        impactVelocity:  traj.impactVelocity,
        drop_m:          traj.drop,
        lead_m,
        ammo:            p.ammoType,
      };
    },

    /**
     * Evaluate whether a shot will penetrate a given armor package.
     * @param {number} pen_mm      - penetrator capability mm RHA
     * @param {Object} armorSetup  - { material, thickness_mm, slope_deg, era }
     * @returns {Object} { penetrates, margin_mm, verdict }
     */
    penetrationCheck(pen_mm, armorSetup) {
      const base  = ballistics.slopeEffect(armorSetup.thickness_mm, armorSetup.slope_deg || 0);
      const mult  = ARMOR_MULT[armorSetup.material] || 1.0;
      const eraBonus = armorSetup.era || 0;
      const effective = base.effective_mm * mult + eraBonus;
      const margin    = effective - pen_mm;
      return {
        penetrates:   margin < 0,
        margin_mm:    margin,
        effective_mm: effective,
        verdict: margin < 0 ? 'PENETRATION' : 'NO PENETRATION',
      };
    },
  };

  /* ════════════════════════════════════════════════════════════
     PUBLIC API
  ════════════════════════════════════════════════════════════ */
  return {
    atmosphere,
    ballistics,
    engine,
    sensors,
    combat,
    MATERIALS,
    ARMOR_MULT,
    version: '2.0.0',
  };

})();

// CommonJS / Node.js compatibility
if (typeof module !== 'undefined' && module.exports) {
  module.exports = IronPhysics;
}
