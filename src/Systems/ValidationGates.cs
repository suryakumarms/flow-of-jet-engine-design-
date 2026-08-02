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
    public static class AeroValidator
    {
        public class AeroCheckResult
        {
            public bool AllPassed { get; set; } = true;
            public List<string> Failures { get; set; } = new();
            public List<string> Warnings { get; set; } = new();
        }

        public static AeroCheckResult ValidateBlades(EngineFlowPath fp, MissionRequirements req)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  GATE 3A: AERODYNAMIC VALIDATION");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            var result = new AeroCheckResult();
            
            foreach (var stage in fp.AllStages())
            {
                var vt = stage.Mean;
                
                // ── GAP 2: Supersonic Tip Mach & Shock Loss ──────────
                // Physics: tip speed U_tip = ω·r_tip increases outward.
                // Relative tip velocity V1r = √(Vz² + (U_tip - Vθ1)²)
                // M1r_tip = V1r / a1  where a1 = √(γ·R·T1_static)
                // If M1r_tip > 1.0 → shock forms → stage efficiency drops:
                //   Δη_shock = 0.08·(M1r - 1.0)^1.5    (Cumpsty correlation)
                // ─────────────────────────────────────────────────────
                if (stage.IsRotor)
                {
                    double omega_s  = stage.RPM * 2.0 * Math.PI / 60.0;
                    double U_tip    = omega_s * stage.TipRadius;
                    double Va_s     = stage.Tip.Va > 0 ? stage.Tip.Va : vt.Va;
                    double Vu1_tip  = stage.Tip.Vu1;

                    // Static temperature at stage inlet from total (M_fan ≈ 0.6, HPC ≈ 0.5)
                    double M_inlet  = stage.Name.Contains("Fan") ? 0.6 : 0.5;
                    double Tt_in_s  = stage.Temperature_In;
                    double gamma_s  = 1.4;
                    double R_s      = 287.0;
                    double T1_stat  = Tt_in_s / (1.0 + (gamma_s - 1.0) / 2.0 * M_inlet * M_inlet);
                    double a1       = Math.Sqrt(gamma_s * R_s * T1_stat);

                    double Wu_tip   = Vu1_tip - U_tip;   // Relative tangential
                    double V1r_tip  = Math.Sqrt(Va_s * Va_s + Wu_tip * Wu_tip);
                    double M1r_tip  = V1r_tip / a1;

                    if (M1r_tip > 1.0)
                    {
                        double delta_eta = 0.08 * Math.Pow(M1r_tip - 1.0, 1.5);
                        string sev = M1r_tip > 1.4 ? "✗ SEVERE" : "⚠ WARN";
                        result.Warnings.Add(
                            $"{stage.Name}: M1r_tip={M1r_tip:F3} > 1.0 → Δη_shock={delta_eta:F4} " +
                            $"({sev})");
                        Console.WriteLine(
                            $"  {stage.Name} TIP SHOCK: U_tip={U_tip:F1}m/s  V1r={V1r_tip:F1}m/s  " +
                            $"M1r={M1r_tip:F3}  Δη={delta_eta:F4}  {sev}");
                        if (M1r_tip > 1.6)
                        {
                            result.Failures.Add($"{stage.Name}: M1r_tip={M1r_tip:F3} > 1.6 → STRONG SHOCK → STALL");
                            result.AllPassed = false;
                        }
                        // ── FIX 2C: feed Δη_shock back into req efficiencies ──────────
                        // This ensures the cycle solver re-runs with degraded efficiency
                        // and produces realistic TSFC / thrust on the next closed-loop iter.
                        double delta_eta_fb = 0.08 * Math.Pow(M1r_tip - 1.0, 1.5);
                        if (stage.Name.Contains("Fan"))
                            req.EtaFan  = Math.Max(0.70, req.EtaFan  - delta_eta_fb);
                        else if (stage.Name.Contains("LPC"))
                            req.EtaLPC  = Math.Max(0.70, req.EtaLPC  - delta_eta_fb);
                        else if (stage.Name.Contains("HPC"))
                            req.EtaHPC  = Math.Max(0.70, req.EtaHPC  - delta_eta_fb);
                        // ──────────────────────────────────────────────────────────────
                    }
                    else
                    {
                        Console.WriteLine(
                            $"  {stage.Name} tip: U={U_tip:F1}m/s  M1r={M1r_tip:F3}  ✓ subsonic tip");
                    }
                }
                
                // De Haller check: W2/W1 > 0.60 (relaxed for highly-loaded multi-stage design)
                if (stage.IsRotor && stage.Name.Contains("C"))  // Compressor
                {
                    if (vt.DeHaller < 0.60)
                    {
                        result.Failures.Add($"{stage.Name}: De Haller = {vt.DeHaller:F3} < 0.60 → FLOW SEPARATION");
                        result.AllPassed = false;
                    }
                }
                
                // Diffusion factor: DF < 0.45 (compressor)
                double df = vt.DiffusionFactor(stage.Solidity);
                if (stage.Name.Contains("C") || stage.Name.Contains("Fan"))
                {
                    if (df > 0.45)
                    {
                        result.Failures.Add($"{stage.Name}: DF = {df:F3} > 0.45 → STALL RISK");
                        result.AllPassed = false;
                    }
                    else if (df > 0.40)
                    {
                        result.Warnings.Add($"{stage.Name}: DF = {df:F3} close to 0.45 limit");
                    }
                }
                
                // Work coefficient check: ψ < 0.45 for compressor, < 2.5 for turbine
                double psi_limit = stage.Name.Contains("T") ? 2.5 : 0.45;
                if (vt.WorkCoefficient > psi_limit)
                {
                    result.Warnings.Add($"{stage.Name}: ψ = {vt.WorkCoefficient:F2} exceeds {psi_limit}");
                }
                
                Console.WriteLine($"  {stage.Name}: DF={df:F3}  DeH={vt.DeHaller:F3}  ψ={vt.WorkCoefficient:F2}  φ={vt.FlowCoefficient:F2}  {(stage.Name.Contains("T") || df<=0.45?"✓":"✗")}");
            }
            
            Console.WriteLine($"  Aero check: {(result.AllPassed ? "ALL PASSED ✓" : "FAILURES FOUND ✗")}");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            return result;
        }
    }

    public static class ManufacturingValidator
    {
        public class MfgCheckResult
        {
            public List<string> Issues { get; set; } = new();
            public bool AllPassed => Issues.Count == 0;
        }

        public static MfgCheckResult Validate(EngineFlowPath fp, CombustorDesign comb)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  GATE 6: DMLS MANUFACTURING VALIDATION");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            var result = new MfgCheckResult();
            
            // Check blade thickness
            foreach (var s in fp.AllStages())
            {
                double minWall = s.Chord * s.MaxThicknessRatio;
                if (minWall < 0.4e-3) // DMLS min wall 0.4mm
                    result.Issues.Add($"{s.Name}: trailing edge {minWall*1000:F2}mm < 0.4mm DMLS limit");
                
                // Check overhang angles (blade lean > 45°)
                double leanAngle = Math.Abs(s.StaggerAngle) * 180.0 / Math.PI;
                if (leanAngle > 45.0)
                    result.Issues.Add($"{s.Name}: stagger {leanAngle:F1}° > 45° needs support structures");
                
                Console.WriteLine($"  {s.Name}: wall={minWall*1000:F2}mm  stagger={leanAngle:F1}°  " +
                                  $"{(minWall >= 0.4e-3 && leanAngle <= 45 ? "✓" : "⚠")}");
            }
            
            // Combustor liner
            if (comb.LinerThickness_m < 1.0e-3)
                result.Issues.Add("Combustor liner < 1mm: difficult for DMLS");
            
            // Cooling channels (if any blade has internal cooling)
            foreach (var s in fp.HPTStages.Concat(fp.LPTStages.Take(1)))
            {
                if (s.Material.Contains("CMSX"))
                {
                    Console.WriteLine($"  {s.Name}: internal cooling channels — needs powder removal validation");
                    if (s.Chord < 0.02)
                        result.Issues.Add($"{s.Name}: chord {s.Chord*1000:F1}mm too small for internal cooling");
                }
            }
            
            Console.WriteLine($"  Manufacturing check: {(result.AllPassed ? "ALL PASSED ✓" : $"{result.Issues.Count} ISSUES ✗")}");
            foreach (var iss in result.Issues)
                Console.WriteLine($"    ⚠ {iss}");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            return result;
        }
    }

}
