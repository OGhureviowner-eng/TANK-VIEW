/**
 * ═══════════════════════════════════════════════════════════════
 * IRON COMMAND — C++ CLI Ballistics Calculator
 * main_ballistics.cpp
 *
 * Interactive command-line interface for the ballistics engine.
 *
 * Build:
 *   g++ -O2 -std=c++17 -o ironballistics main_ballistics.cpp
 *
 * Run:
 *   ./ironballistics
 *   ./ironballistics --quick 120 1650 apfsds 2000
 *   ./ironballistics --batch scenario.csv
 * ═══════════════════════════════════════════════════════════════
 */

#include "ballistics.h"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>

using namespace IronPhysics;

// ─────────────────────────────────────────────
// Print progress bar in terminal
// ─────────────────────────────────────────────
static void printBar(const std::string& label, double pct, int width = 40) {
  int filled = static_cast<int>(pct / 100.0 * width);
  std::cout << "  " << label;
  std::cout << " [";
  for (int i = 0; i < width; i++)
    std::cout << (i < filled ? '█' : '░');
  std::cout << "] " << std::fixed << std::setprecision(1) << pct << "%\n";
}

// ─────────────────────────────────────────────
// Scenario runner
// ─────────────────────────────────────────────
static void runScenario(const std::string& ammoType, double caliber_m,
                        double v0, double mass_kg, double elev_deg,
                        double armorThick_mm, double armorSlope_deg,
                        double era_bonus = 0) {
  std::cout << "\n  ══════════════════════════════════════════════\n";
  std::cout << "  SCENARIO: " << ammoType << " | Cal: " << caliber_m*1000 << "mm"
            << " | V0: " << v0 << "m/s | Elev: " << elev_deg << "°\n";
  std::cout << "  ══════════════════════════════════════════════\n";

  // Trajectory
  Traj::Params tp;
  tp.v0        = v0;
  tp.mass_kg   = mass_kg;
  tp.caliber_m = caliber_m;
  tp.elev_deg  = elev_deg;
  tp.type      = ammoType;
  tp.dt        = 0.002;

  auto t0 = std::chrono::high_resolution_clock::now();
  auto traj = Traj::solve(tp);
  auto t1 = std::chrono::high_resolution_clock::now();
  double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

  Reporter::printTrajectory(traj, ammoType + " trajectory");
  std::cout << "    (computed in " << elapsed << " ms)\n\n";

  // Penetration
  double pen_mm = 0;
  if (ammoType == "apfsds") {
    double len = caliber_m * 22;  // L/D ≈ 22
    double dia = caliber_m * 0.22;
    auto pres = Pen::apfsds(traj.impactVelocity_ms, len, dia);
    Reporter::printPenetration(pres);
    pen_mm = pres.penetration_mm;
  } else if (ammoType == "heat") {
    pen_mm = Pen::heat_mm(caliber_m);
    std::cout << "  ── PENETRATION ──\n";
    std::cout << "    HEAT Jet (Munroe): " << pen_mm << " mm RHA\n\n";
  }

  // Armor
  std::vector<Armor::Layer> layers = {{
    Armor::Material::DU_COMPOSITE, armorThick_mm, armorSlope_deg
  }};
  auto armRes = Armor::evaluate(layers, pen_mm, era_bonus,
                                ammoType == "heat");
  Reporter::printArmor(armRes);

  // Visual verdict
  if (armRes.penetrated) {
    std::cout << "  ╔══════════════════════════════╗\n";
    std::cout << "  ║  !! TARGET PENETRATED !!     ║\n";
    std::cout << "  ╚══════════════════════════════╝\n";
  } else {
    std::cout << "  ╔══════════════════════════════╗\n";
    std::cout << "  ║  ⛨  ARMOR HELD  +" << std::setw(6) << armRes.margin_mm << "mm  ║\n";
    std::cout << "  ╚══════════════════════════════╝\n";
  }
  std::cout << "\n";
}

// ─────────────────────────────────────────────
// Interactive menu
// ─────────────────────────────────────────────
static void interactiveMenu() {
  Reporter::printHeader();

  while (true) {
    std::cout << "  ┌─────────────────────────────────────┐\n";
    std::cout << "  │  IRON COMMAND  |  Main Menu          │\n";
    std::cout << "  ├─────────────────────────────────────┤\n";
    std::cout << "  │  1. Ballistics Calculator            │\n";
    std::cout << "  │  2. Armor Penetration Analyzer       │\n";
    std::cout << "  │  3. Engine Performance               │\n";
    std::cout << "  │  4. Quick Fire Control               │\n";
    std::cout << "  │  5. Run Preset Scenarios             │\n";
    std::cout << "  │  6. Atmosphere Model                 │\n";
    std::cout << "  │  0. Exit                             │\n";
    std::cout << "  └─────────────────────────────────────┘\n";
    std::cout << "  >> ";

    int choice;
    std::cin >> choice;

    if (choice == 0) break;

    else if (choice == 1) {
      std::cout << "\n  === BALLISTICS CALCULATOR ===\n";
      double v0, mass, cal, elev;
      std::string type;
      std::cout << "  Ammo type (apfsds/heat/he/hesh): ";  std::cin >> type;
      std::cout << "  Muzzle velocity m/s [1650]: ";       std::cin >> v0;
      std::cout << "  Projectile mass kg [4.8]: ";         std::cin >> mass;
      std::cout << "  Caliber mm [120]: ";                 std::cin >> cal;
      std::cout << "  Elevation degrees [0]: ";            std::cin >> elev;

      Traj::Params tp { v0, mass, cal/1000.0, elev, 0, 8000, 0.002, type };
      auto r = Traj::solve(tp);
      Reporter::printTrajectory(r, type);

      // Penetration table by range
      std::cout << "  ── PENETRATION VS RANGE ──\n";
      std::cout << "  " << std::setw(8) << "Range" << std::setw(12) << "Velocity"
                << std::setw(14) << "Penetration" << "\n";
      std::cout << "  " << std::setw(8) << "(m)" << std::setw(12) << "(m/s)"
                << std::setw(14) << "(mm RHA)" << "\n";
      std::cout << "  " << std::string(34, '-') << "\n";

      for (const auto& pt : r.points) {
        if (static_cast<int>(pt.x) % 500 == 0 && pt.x > 0) {
          double pen = 0;
          if (type == "apfsds") {
            auto pr = Pen::apfsds(pt.v, cal/1000.0*22, cal/1000.0*0.22);
            pen = pr.penetration_mm;
          } else {
            pen = Pen::heat_mm(cal/1000.0);
          }
          std::cout << "  " << std::setw(8) << std::fixed << std::setprecision(0) << pt.x
                    << std::setw(12) << std::setprecision(1) << pt.v
                    << std::setw(14) << std::setprecision(1) << pen << "\n";
        }
      }
      std::cout << "\n";
    }

    else if (choice == 2) {
      std::cout << "\n  === ARMOR ANALYZER ===\n";
      double thick, slope, pen, era;
      std::string mat, round;
      std::cout << "  Plate thickness mm [250]: ";         std::cin >> thick;
      std::cout << "  Slope angle ° from vertical [68]: "; std::cin >> slope;
      std::cout << "  Material (rha/cha/hha/composite/chobham/du_composite): ";
      std::cin >> mat;
      std::cout << "  ERA bonus mm RHA [0]: ";             std::cin >> era;
      std::cout << "  Incoming penetration mm [700]: ";    std::cin >> pen;

      Armor::Material matEnum = Armor::Material::RHA;
      if (mat == "cha")          matEnum = Armor::Material::CHA;
      if (mat == "hha")          matEnum = Armor::Material::HHA;
      if (mat == "composite")    matEnum = Armor::Material::COMPOSITE;
      if (mat == "chobham")      matEnum = Armor::Material::CHOBHAM;
      if (mat == "du_composite") matEnum = Armor::Material::DU_COMPOSITE;

      std::vector<Armor::Layer> layers = {{matEnum, thick, slope}};
      auto r = Armor::evaluate(layers, pen, era);
      Reporter::printArmor(r);

      // Slope table
      std::cout << "  ── SLOPE EFFECT TABLE ──\n";
      std::cout << "  " << std::setw(8) << "Angle°" << std::setw(12) << "Multiplier"
                << std::setw(12) << "Tate" << std::setw(14) << "Effective\n";
      for (double a : {0.0, 20.0, 40.0, 55.0, 60.0, 65.0, 68.0, 70.0, 75.0}) {
        auto se = Armor::slopeEffect(thick, a);
        std::cout << "  " << std::setw(8) << a
                  << std::setw(12) << std::setprecision(3) << se.norm_mult
                  << std::setw(12) << se.tate_factor
                  << std::setw(12) << se.effective_mm << " mm\n";
      }
      std::cout << "\n";
    }

    else if (choice == 3) {
      std::cout << "\n  === ENGINE PERFORMANCE ===\n";
      double hp, mass, fuel, trackW, contactL;
      std::string type;
      std::cout << "  Engine type (turbine/diesel/hybrid): "; std::cin >> type;
      std::cout << "  Max power hp [1500]: ";                  std::cin >> hp;
      std::cout << "  Combat mass tonnes [68]: ";              std::cin >> mass;
      std::cout << "  Fuel capacity litres [1909]: ";          std::cin >> fuel;
      std::cout << "  Track width mm [635]: ";                 std::cin >> trackW;
      std::cout << "  Contact length mm [4000]: ";             std::cin >> contactL;

      Engine::Type etype = Engine::Type::DIESEL;
      if (type == "turbine") etype = Engine::Type::TURBINE;
      if (type == "hybrid")  etype = Engine::Type::HYBRID;

      Engine::Params ep { hp, mass, fuel, trackW, contactL, etype };
      auto perf = Engine::compute(ep);
      Reporter::printEngine(perf);

      printBar("Power/Weight", std::min(100.0, perf.ptw_hp_tonne / 30.0 * 100));
      printBar("Top Speed   ", perf.topSpeed_kmh / 90.0 * 100);
      printBar("Range       ", std::min(100.0, perf.range_km / 800.0 * 100));
      printBar("Efficiency  ", perf.thermal_efficiency * 100);
      std::cout << "\n";
    }

    else if (choice == 4) {
      std::cout << "\n  === QUICK FIRE CONTROL ===\n";
      double elev, targetSpd, armorThick;
      std::string ammo;
      std::cout << "  Gun elevation ° [3.5]: ";     std::cin >> elev;
      std::cout << "  Ammo type [apfsds]: ";         std::cin >> ammo;
      std::cout << "  Target speed m/s [10]: ";      std::cin >> targetSpd;
      std::cout << "  Target armor mm [600]: ";      std::cin >> armorThick;

      auto sol = FCS::solve(elev, ammo, targetSpd, armorThick, 60.0);
      std::cout << "\n  ── FIRE CONTROL SOLUTION ──\n";
      std::cout << "    Estimated Range:    " << sol.range_m << " m\n";
      std::cout << "    Time of Flight:     " << sol.timeOfFlight_s << " s\n";
      std::cout << "    Impact Velocity:    " << sol.impactVelocity_ms << " m/s\n";
      std::cout << "    Penetration:        " << sol.penetration_mm << " mm RHA\n";
      std::cout << "    Lateral Lead:       " << sol.lead_m << " m\n";
      std::cout << "    Sight Correction:   " << sol.correction_mil << " mil\n";
      std::cout << "    Will Penetrate:     " << (sol.willPenetrate ? "YES" : "NO") << "\n\n";
    }

    else if (choice == 5) {
      std::cout << "\n  Running preset combat scenarios...\n\n";

      // WWII classic: 88mm vs Sherman
      runScenario("apfsds", 0.088, 930, 7.3, 2.5, 50, 30);

      // Cold War: 105mm HEAT vs T-55
      runScenario("heat", 0.105, 1050, 10.5, 0, 200, 45);

      // Modern: 120mm L/55 APFSDS vs ERA-protected T-90
      runScenario("apfsds", 0.120, 1750, 4.2, 1.5, 400, 68, 300);

      // Next-gen: 130mm vs future heavy armor
      runScenario("apfsds", 0.130, 1850, 5.8, 0.5, 600, 70, 400);
    }

    else if (choice == 6) {
      std::cout << "\n  === ISA ATMOSPHERE MODEL ===\n";
      std::cout << "  " << std::setw(10) << "Alt (m)"
                << std::setw(14) << "Density kg/m³"
                << std::setw(14) << "Sound m/s"
                << std::setw(10) << "Temp °C\n";
      std::cout << "  " << std::string(48, '-') << "\n";
      for (double alt : {0,500,1000,2000,3000,5000,8000,11000}) {
        double T = 288.15 - 0.0065 * std::min(alt, 11000.0) - 273.15;
        std::cout << "  " << std::setw(10) << alt
                  << std::setw(14) << std::fixed << std::setprecision(4) << Atmosphere::density(alt)
                  << std::setw(14) << std::setprecision(2) << Atmosphere::soundSpeed(alt)
                  << std::setw(10) << std::setprecision(1) << T << "\n";
      }
      std::cout << "\n";
    }
  }

  std::cout << "\n  [IRON COMMAND] Shutdown. Goodbye, Commander.\n\n";
}

// ─────────────────────────────────────────────
// Quick mode: ./ironballistics --quick CAL V0 TYPE RANGE
// ─────────────────────────────────────────────
static void quickMode(int argc, char* argv[]) {
  if (argc < 5) {
    std::cerr << "Usage: --quick <caliber_mm> <velocity_ms> <type> <max_range_m>\n";
    return;
  }
  double cal   = std::stod(argv[2]) / 1000.0;
  double v0    = std::stod(argv[3]);
  std::string type = argv[4];
  double range = std::stod(argv[5]);
  double mass  = cal > 0.1 ? 4.8 : 3.0;

  Traj::Params tp { v0, mass, cal, 3.0, 0, range, 0.002, type };
  auto r = Traj::solve(tp);

  std::cout << std::fixed << std::setprecision(2);
  std::cout << type << " " << cal*1000 << "mm @ " << v0 << "m/s:\n";
  std::cout << "  Range:  " << r.maxRange_m << " m\n";
  std::cout << "  ToF:    " << r.timeOfFlight_s << " s\n";
  std::cout << "  Vimp:   " << r.impactVelocity_ms << " m/s\n";
  std::cout << "  Drop:   " << std::abs(r.totalDrop_m * 1000) << " mm\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main(int argc, char* argv[]) {
  if (argc >= 2 && std::string(argv[1]) == "--quick") {
    quickMode(argc, argv);
    return 0;
  }

  if (argc >= 2 && std::string(argv[1]) == "--scenarios") {
    Reporter::printHeader();
    runScenario("apfsds", 0.120, 1650, 4.8, 2.0, 400, 68, 300);
    runScenario("heat",   0.120, 1050, 10.5, 0.0, 400, 50, 100);
    return 0;
  }

  interactiveMenu();
  return 0;
}
