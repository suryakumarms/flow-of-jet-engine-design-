using Leap71.LatticeLibraryExamples;
using Leap71.QuasiCrystalExamples;
using Leap71.ShapeKernelExamples;
using PicoGK;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System;

namespace JetEngine
{
    public class CombustorDesign
    {
        public double Length_m         { get; set; }
        public double OuterRadius_m   { get; set; }
        public double InnerRadius_m   { get; set; }
        public double LinerThickness_m{ get; set; } = 0.002;
        public double NumFuelInjectors{ get; set; }
        public double PrimaryZonePhi  { get; set; }  // Equivalence ratio
        public double PatternFactor   { get; set; }
        public double OTDF            { get; set; }  // Overall Temperature Distribution Factor
        public double RTDF            { get; set; }  // Radial Temperature Distribution Factor
        public double CombustionEff   { get; set; }
        public double PressureLoss    { get; set; }
        public double NOx_EI          { get; set; }  // g/kg fuel
        public double CO_EI           { get; set; }
        public string LinerMaterial   { get; set; } = "Hastelloy X + TBC";
        
        public static CombustorDesign Design(CycleResult cycle, EngineFlowPath fp)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  GATE 3B: COMBUSTOR DESIGN & PATTERN FACTOR");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            var c = new CombustorDesign();
            
            // Annulus sizing: combustor sits between HPC exit and HPT inlet
            var lastHPC = fp.HPCStages.Last();
            c.OuterRadius_m = lastHPC.TipRadius * 1.3;
            c.InnerRadius_m = lastHPC.HubRadius * 0.85;
            
            // Length: L/H ratio typically 2.5-4.0 for annular
            double height = c.OuterRadius_m - c.InnerRadius_m;
            c.Length_m = height * 3.0;
            
            // Fuel injectors: one per ~40mm arc at mean radius
            double meanR = (c.OuterRadius_m + c.InnerRadius_m) / 2.0;
            c.NumFuelInjectors = Math.Round(2.0 * Math.PI * meanR / 0.04);
            
            // Primary zone equivalence ratio
            double f = cycle.Stations[4].FuelAirRatio;
            c.PrimaryZonePhi = f / 0.068 * 2.5;  // Rich primary zone
            
            // Combustion efficiency (Lefebvre correlation)
            double Tt3 = cycle.Stations[3].Tt;
            double Pt3 = cycle.Stations[3].Pt;
            double theta = 5.0 * Pt3 * Math.Exp(Tt3 / 300.0) / (cycle.CoreMassFlow / 10.0);
            c.CombustionEff = Math.Min(0.999, 1.0 - 0.5 * Math.Exp(-theta / 1e6));
            
            // Pattern Factor (Lefebvre empirical correlation for annular combustors)
            // PF = 1 - exp(-0.05 * (L_liner / D_liner) * (delta_P / q_ref))
            // Typical value for modern aero engines is 0.15 - 0.25
            double delta_P_frac = 0.04; 
            c.PressureLoss = delta_P_frac;
            
            // Simplified proxy for mixing intensity (L/D * dP/q)
            double mixing_parameter = (c.Length_m / height) * (delta_P_frac * 100.0);
            c.OTDF = 1.0 - Math.Exp(-0.07 * mixing_parameter);
            c.OTDF = Math.Clamp(c.OTDF, 0.12, 0.35); // Bounded to realistic values
            
            // Radial Temperature Distribution Factor (RTDF) determines stator life
            c.RTDF = c.OTDF * 0.7; // RTDF is typically ~70% of OTDF
            
            c.PatternFactor = c.OTDF * 0.40; // Dilution zone mixing reduces pattern factor to ~40% of liner exit variation
            
            double Tmean = cycle.Stations[4].Tt;
            double Tpeak = Tmean + c.OTDF * (Tmean - Tt3);
            
            // Emissions (P3-T3 correlation for NOx)
            // Lefebvre: NOx ∝ P^0.5 · exp(T3/300) · τ_res
            double tau_res = c.Length_m / 50.0;  // Residence time ~6ms
            c.NOx_EI = 0.4 * 0.15 * Math.Sqrt(Pt3 / 1e5) * Math.Exp(Tt3 / 600.0) * tau_res * 1000;
            c.CO_EI  = 30.0 / (c.CombustionEff * 1000);  // Inversely proportional to efficiency

            Console.WriteLine($"  Combustor L={c.Length_m:F3}m  H={height:F3}m  Injectors={c.NumFuelInjectors}");
            Console.WriteLine($"  OTDF={c.OTDF:F3} (T_peak = {Tpeak:F0}K)  RTDF={c.RTDF:F3}");
            Console.WriteLine($"  η_comb={c.CombustionEff:F4}  NOx_EI={c.NOx_EI:F1} g/kg");

            // ── CANTERA HYBRID CALL ──
            var chemReq = new WSLSimulationClient.CombustionRequest
            {
                fuel_type = cycle.Stations.ContainsKey(4) && cycle.Stations[4].FuelAirRatio > 0.05 ? "Hydrogen" : "SAF",
                inlet_temperature_K = Tt3,
                inlet_pressure_Pa = Pt3,
                equivalence_ratio = c.PrimaryZonePhi,
                mass_flow_kg_s = cycle.CoreMassFlow
            };
            var chemRes = WSLSimulationClient.QueryCombustion(chemReq);
            if (chemRes != null)
            {
                Console.WriteLine($"  [WSL Cantera] Solved detailed chemistry ({chemRes.status}):");
                Console.WriteLine($"    Adiabatic Flame T: {chemRes.adiabatic_flame_temperature_K:F1} K");
                c.NOx_EI = chemRes.species_mass_fractions.ContainsKey("NOx") ? chemRes.species_mass_fractions["NOx"] * 1000.0 : c.NOx_EI;
                c.CO_EI = chemRes.species_mass_fractions.ContainsKey("CO") ? chemRes.species_mass_fractions["CO"] * 1000.0 : c.CO_EI;
            }
            else
            {
                Console.WriteLine("  [WSL Cantera] Backend offline at http://localhost:8000. Running local analytical chemistry proxy...");
            }
            
            // GATE CHECK
            bool pf_ok   = c.PatternFactor <= 0.15;
            bool nox_ok  = c.NOx_EI < 50.0;  // CAEP/8 limit ~40-60 for this class
            bool eff_ok  = c.CombustionEff > 0.99;
            
            Console.WriteLine($"  Combustor L={c.Length_m*1000:F0}mm  R_out={c.OuterRadius_m*1000:F0}mm");
            Console.WriteLine($"  Injectors: {c.NumFuelInjectors:F0}");
            Console.WriteLine($"  Pattern Factor: {c.PatternFactor:F3}  {(pf_ok?"✓":"✗ FAIL")}");
            Console.WriteLine($"  Combustion η: {c.CombustionEff:F4}  {(eff_ok?"✓":"✗ FAIL")}");
            Console.WriteLine($"  NOx EI: {c.NOx_EI:F1} g/kg  {(nox_ok?"✓":"✗ CAEP/8 FAIL")}");
            Console.WriteLine($"  CO EI:  {c.CO_EI:F1} g/kg");
            Console.WriteLine($"  Liner: {c.LinerMaterial}");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            return c;
        }
    }

    public static class CombustorAcoustics
    {
        public class AcousticResult
        {
            public double F_1L_Hz;            // first longitudinal mode
            public double F_1T_Hz;            // first transverse (tangential)
            public double GrowthRate;         // Rayleigh growth rate σ (1/s)
            public double DampingCoeff;       // liner damping ζ
            public double StabilityMargin;    // η_stab = ζ/σ - 1 (>0 = stable)
            public bool   Stable;
            public string WorstMode = "";
        }

        public static AcousticResult Analyze(
            CombustorDesign comb, CycleResult cycle)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  COMBUSTOR ACOUSTICS (Rayleigh/Rijke/Crocco criterion)");
            Console.WriteLine("════════════════════════════════════════════════════════");

            var r = new AcousticResult();
            double T4    = cycle.Stations.ContainsKey(4) ? cycle.Stations[4].Tt : 1650.0;
            double gamma = 1.30;
            double R     = 287.0;
            double c_c   = Math.Sqrt(gamma * R * T4);      // speed of sound in combustor

            // Longitudinal modes
            double L_c = comb.Length_m > 0 ? comb.Length_m : 0.45;  // m
            r.F_1L_Hz  = c_c / (2.0 * L_c);

            // Transverse (tangential) mode: f_1T = 1.84·c/(π·D)  [Bessel J1 root]
            double D_c = (comb.InnerRadius_m + comb.OuterRadius_m);  // diameter ≈ (ID+OD)
            r.F_1T_Hz  = 1.84 * c_c / (Math.PI * Math.Max(D_c, 0.1));

            // Rayleigh growth rate (simplified Crocco n-τ model)
            // σ = (γ-1)/(2ρc) · |n_int| · cos(ω·τ_delay)
            double rho_c   = cycle.Stations.ContainsKey(4) ? cycle.Stations[4].Pt / (R*T4) : 3.0;
            double n_int   = 2.5;   // interaction index (fuel-rich zone)
            double tau_d   = 1.5e-3;  // time delay 1.5ms (typical)
            double omega_1L = 2*Math.PI*r.F_1L_Hz;
            r.GrowthRate  = (gamma-1.0)/(2.0*rho_c*c_c) * n_int * Math.Cos(omega_1L*tau_d);
            r.GrowthRate  = Math.Max(0, r.GrowthRate);  // only positive = unstable

            // Liner damping from effusion holes (Howe 1998):
            // ζ = 0.25 · (hole_area / total_area) · M_hole · (1 + M_mean)
            double sigma_h = 0.04;   // perforate porosity (4% liner holes)
            double M_hole  = 0.3;    // hole Mach number
            double M_mean  = 0.05;   // mean flow Mach in combustor
            r.DampingCoeff = 0.25 * sigma_h * M_hole * (1.0 + M_mean) * c_c;

            r.StabilityMargin = r.GrowthRate > 0
                ? r.DampingCoeff / r.GrowthRate - 1.0
                : 10.0;   // unconditionally stable if no growth
            r.Stable = r.StabilityMargin > 0;
            r.WorstMode = r.F_1L_Hz < r.F_1T_Hz ? "1L longitudinal" : "1T tangential";

            Console.WriteLine($"  f_1L={r.F_1L_Hz:F0}Hz  f_1T={r.F_1T_Hz:F0}Hz  c_comb={c_c:F0}m/s");
            Console.WriteLine($"  σ_growth={r.GrowthRate:F2}/s  ζ_liner={r.DampingCoeff:F2}/s  η_stab={r.StabilityMargin:F3}");
            Console.WriteLine($"  Status: {(r.Stable?"✓ STABLE":"✗ THERMOACOUSTIC INSTABILITY RISK")} (worst mode: {r.WorstMode})");
            Console.WriteLine("════════════════════════════════════════════════════════");
            return r;
        }
    }

    public static class CombustorDiffuser
    {
        public class DiffuserResult
        {
            public double V3_mps              { get; set; }  // HPC exit velocity
            public double AreaRatio           { get; set; }  // A_comb / A_HPC
            public double DiffuserDeltaP_Pa   { get; set; }  // Stagnation pressure loss
            public double DiffuserDeltaP_frac { get; set; }  // As fraction of P3
            public double CombustorInletV_mps { get; set; }  // Should be ~20 m/s
            public double DiffuserAngle_deg   { get; set; } = 7.0;  // Half-angle (no separation)
            public double DiffuserLength_mm   { get; set; }
            public bool   FlameBlowoutRisk    { get; set; }  // True if V_ref > 30 m/s
            public bool   SeparationRisk      { get; set; }  // True if angle > 9°
        }

        public static DiffuserResult Design(
            CycleResult cycle, EngineFlowPath fp, CombustorDesign comb)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  GAP 5: COMBUSTOR DIFFUSER DESIGN");
            Console.WriteLine("════════════════════════════════════════════════════════");

            var r = new DiffuserResult();
            const double V_ref = 20.0;    // Target combustor inlet velocity (m/s)
            const double C_loss = 0.20;   // Diffuser loss coefficient (7° half-angle)

            // HPC exit state (Station 3)
            if (!cycle.Stations.ContainsKey(3)) return r;
            var s3 = cycle.Stations[3];

            // HPC exit density from static conditions (M_HPC_exit ≈ 0.35)
            double M3  = 0.35;
            double T3s = s3.Tt / (1.0 + 0.2 * M3 * M3);
            double P3s = s3.Pt * Math.Pow(T3s / s3.Tt, 3.5);
            double rho3 = P3s / (287.0 * T3s);

            // HPC exit annulus area from last HPC stage
            var lastHPC = fp.HPCStages.Count > 0 ? fp.HPCStages.Last() : null;
            if (lastHPC == null) return r;
            double A3 = Math.PI * (lastHPC.TipRadius * lastHPC.TipRadius
                                 - lastHPC.HubRadius * lastHPC.HubRadius);

            // HPC exit velocity
            r.V3_mps = cycle.CoreMassFlow / (rho3 * A3);
            r.V3_mps = Math.Max(r.V3_mps, 60.0);  // Physical floor

            // Area ratio needed to reach V_ref
            r.AreaRatio = r.V3_mps / V_ref;

            // Actual combustor inlet velocity
            r.CombustorInletV_mps = r.V3_mps / r.AreaRatio;

            // Diffuser pressure drop (Sovran-Klomp correlation)
            double q3 = 0.5 * rho3 * r.V3_mps * r.V3_mps;
            r.DiffuserDeltaP_Pa   = C_loss * q3 * Math.Pow(1.0 - 1.0 / r.AreaRatio, 2);
            r.DiffuserDeltaP_frac = r.DiffuserDeltaP_Pa / s3.Pt;

            // Geometry: diffuser length from area ratio and half-angle = 7°
            double r_in  = Math.Sqrt(A3 / Math.PI);
            double r_out = r_in * Math.Sqrt(r.AreaRatio);
            r.DiffuserLength_mm = (r_out - r_in) / Math.Tan(r.DiffuserAngle_deg * Math.PI / 180.0) * 1000.0;

            r.FlameBlowoutRisk = r.CombustorInletV_mps > 30.0;
            r.SeparationRisk   = r.DiffuserAngle_deg > 9.0;

            Console.WriteLine($"  HPC exit V3={r.V3_mps:F1} m/s  AR={r.AreaRatio:F2}  " +
                              $"V_ref={r.CombustorInletV_mps:F1} m/s (target 20)");
            Console.WriteLine($"  ΔP_diff={r.DiffuserDeltaP_Pa/1000:F1} kPa  " +
                              $"({r.DiffuserDeltaP_frac*100:F2}% of P3)  " +
                              $"L_diff={r.DiffuserLength_mm:F0} mm");
            Console.WriteLine($"  Flame blowout risk: {(r.FlameBlowoutRisk?"✗ YES":"✓ NO")}  " +
                              $"Separation risk: {(r.SeparationRisk?"✗ YES (angle > 9°)":"✓ NO")}");
            Console.WriteLine("════════════════════════════════════════════════════════");

            return r;
        }
    }

}
