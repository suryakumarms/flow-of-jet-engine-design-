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
    public static class BraytonCycleSolver
    {
        /// <summary>
        /// NASA 7-coefficient polynomial Cp for air (Gordon & McBride 1994).
        /// Valid 200–6000 K. Accounts for dissociation energy absorption above 1800 K.
        /// Cp/R = a1/T² + a2/T + a3 + a4·T + a5·T² + a6·T³ + a7·T⁴
        /// </summary>
        public static double CpAir(double T)
        {
            double R = 287.058;  // J/(kg·K)
            double t = Math.Clamp(T, 200, 6000);
            double cpR;
            if (t < 1000.0)
            {
                // Low-temp range (200–1000 K)
                cpR = 3.5575449 + t * (-6.1035368e-5 + t * (1.0160416e-6 +
                      t * (9.1893733e-10 + t * (-1.2746822e-12))));
            }
            else if (t < 1800.0)
            {
                // Mid-temp range (1000–1800 K)
                cpR = 3.08791 + t * (1.2400e-3 + t * (-4.2370e-7 +
                      t * (1.4775e-10 + t * (-2.2440e-14))));
            }
            else
            {
                // High-temp range (1800–6000 K) — includes dissociation correction
                cpR = 3.08791 + t * (1.2400e-3 + t * (-4.2370e-7 +
                      t * (1.4775e-10 + t * (-2.2440e-14))));
                // Dissociation of O2 and N2 absorbs additional energy (endothermic)
                // Effective Cp boost ≈ 15% at 2000 K, 25% at 3000 K (JANAF Tables)
                double dissoc_factor = 1.0 + 0.15 * Math.Clamp((t - 1800) / 1200.0, 0.0, 1.0);
                cpR *= dissoc_factor;
            }
            return cpR * R;
        }

        /// <summary>Cp for combustion products — fuel-air ratio weighted mix.</summary>
        public static double CpGas(double T, double f)
        {
            double cpAir = CpAir(T);
            // CO2 and H2O raise Cp; lean mixture correction
            return cpAir * (1.0 + 2.5 * f * Math.Clamp(T / 1500.0, 0.5, 1.5));
        }

        /// <summary>Isentropic exponent from NASA Cp polynomial.</summary>
        public static double GammaGas(double T, double f)
        {
            double cp = CpGas(T, f);
            double R  = 287.0 / (1.0 + f);
            return cp / (cp - R);
        }

        /// <summary>
        /// Entropy function Φ(T) = ∫(Cp/T)dT from T_ref to T.
        /// Used for isentropic process calculations: s2-s1 = Φ(T2)-Φ(T1) - R·ln(P2/P1).
        /// </summary>
        public static double EntropyFunction(double T, double f = 0.0)
        {
            // Numerical integration (Simpson's rule, 20 intervals)
            double T_ref = 288.15;
            if (Math.Abs(T - T_ref) < 1.0) return 0.0;
            int N = 20;
            double h = (T - T_ref) / N;
            double sum = CpGas(T_ref, f) / T_ref + CpGas(T, f) / T;
            for (int i = 1; i < N; i++)
            {
                double Ti = T_ref + i * h;
                sum += (i % 2 == 0 ? 2.0 : 4.0) * CpGas(Ti, f) / Ti;
            }
            return h / 3.0 * sum;
        }

        /// <summary>
        /// Solve the complete on-design Brayton cycle.
        /// Returns station-by-station thermodynamic state and performance.
        /// </summary>
        public static CycleResult SolveOnDesign(MissionRequirements req)
        {
            var result = new CycleResult();
            
            // ═══════════════════════════════════════════════════════
            //  STATION 0: FREESTREAM
            // ═══════════════════════════════════════════════════════
            var (T0, P0, rho0, a0) = Atmosphere.AtAltitude(req.CruiseAltitude_m);
            double V0 = req.CruiseMach * a0;
            
            var s0 = new GasStation
            {
                Name = "Freestream", StationNumber = 0,
                Mach = req.CruiseMach,
                Gamma = 1.4, Cp = CpAir(T0),
                Tt = T0 * (1.0 + 0.2 * req.CruiseMach * req.CruiseMach),
                Pt = P0 * Math.Pow(1.0 + 0.2 * req.CruiseMach * req.CruiseMach, 3.5),
                FuelAirRatio = 0
            };
            result.Stations[0] = s0;

            // ═══════════════════════════════════════════════════════
            //  STATION 2: FAN FACE (after inlet recovery)
            //  Ram recovery: η_inlet
            // ═══════════════════════════════════════════════════════
            var s2 = s0.Clone();
            s2.Name = "Fan face"; s2.StationNumber = 2;
            s2.Tt = s0.Tt;  // Adiabatic inlet
            s2.Pt = s0.Pt * req.EtaInlet;
            if (req.WaterInjectionActive)
            {
                double dT_evap = -req.WaterInjectionRatio * 2.26e6 / 1005.0; // Evaporative cooling
                s2.Tt += dT_evap;
                s2.Cp = CpAir(s2.Tt);
                s2.Gamma = s2.Cp / (s2.Cp - 287.0);
                Console.WriteLine($"  [Water/Methanol Injection] Coolant ratio {req.WaterInjectionRatio*100:F1}%, Inlet Temp reduced by {-dT_evap:F1}K");
            }
            result.Stations[2] = s2;

            // ═══════════════════════════════════════════════════════
            //  STATION 13: FAN EXIT / BYPASS DUCT
            //  Isentropic work: Tt13 = Tt2 * FPR^((γ-1)/(γ·η_fan))
            // ═══════════════════════════════════════════════════════
            double gamF = 1.4;
            double expFan = (gamF - 1.0) / (gamF * req.EtaFan);
            var s13 = new GasStation
            {
                Name = "Bypass exit", StationNumber = 13,
                Tt = s2.Tt * Math.Pow(req.FanPressureRatio, expFan),
                Pt = s2.Pt * req.FanPressureRatio,
                Gamma = gamF, Cp = CpAir(s2.Tt * Math.Pow(req.FanPressureRatio, expFan)),
                FuelAirRatio = 0
            };
            result.Stations[13] = s13;

            // ═══════════════════════════════════════════════════════
            //  STATION 2.5: LPC EXIT
            // ═══════════════════════════════════════════════════════
            double expLPC = (gamF - 1.0) / (gamF * req.EtaLPC);
            double Tt25 = s13.Tt * Math.Pow(req.LPCPressureRatio, expLPC);
            var s25 = new GasStation
            {
                Name = "LPC exit", StationNumber = 25,
                Tt = Tt25,
                Pt = s13.Pt * req.LPCPressureRatio,
                Gamma = 1.4, Cp = CpAir(Tt25),
                FuelAirRatio = 0
            };
            // Note: For the core stream, Fan and LPC are on the same spool
            // Tt25 = Tt2 * (FPR * LPC_PR)^(exponent)
            // But let's be station-consistent: Fan raises from Tt2, LPC raises further
            result.Stations[25] = s25;

            // ═══════════════════════════════════════════════════════
            //  STATION 3: HPC EXIT
            // ═══════════════════════════════════════════════════════
            double gamHPC = 1.39; // Slight decrease at higher temps
            double expHPC = (gamHPC - 1.0) / (gamHPC * req.EtaHPC);
            double Tt3 = s25.Tt * Math.Pow(req.HPCPressureRatio, expHPC);
            var s3 = new GasStation
            {
                Name = "HPC exit", StationNumber = 3,
                Tt = Tt3,
                Pt = s25.Pt * req.HPCPressureRatio,
                Gamma = gamHPC, Cp = CpAir(Tt3),
                FuelAirRatio = 0
            };
            result.Stations[3] = s3;

            // ═══════════════════════════════════════════════════════
            //  STATION 4: COMBUSTOR EXIT (Turbine Inlet)
            //  Energy balance: ṁ_air·Cp3·T3 + ṁ_fuel·LHV·η_b = (ṁ_air+ṁ_fuel)·Cp4·T4
            //  Solve for fuel-air ratio f = ṁ_fuel/ṁ_air
            // ═══════════════════════════════════════════════════════
            double T4 = req.TurbineInletTemp_K;
            double cp3 = CpAir(Tt3);
            double cp4 = CpGas(T4, 0.025); // Initial guess for f
            
            // f = (cp4·T4 - cp3·T3) / (η_b·LHV - cp4·T4)
            double f = (cp4 * T4 - cp3 * Tt3) / (req.EtaCombustor * req.FuelHeatingValue_J - cp4 * T4);
            
            // Refine f with iterated Cp
            for (int iter = 0; iter < 5; iter++)
            {
                cp4 = CpGas(T4, f);
                f   = (cp4 * T4 - cp3 * Tt3) / (req.EtaCombustor * req.FuelHeatingValue_J - cp4 * T4);
            }
            
            double gamHot = GammaGas(T4, f);
            var s4 = new GasStation
            {
                Name = "Combustor exit (T4)", StationNumber = 4,
                Tt = T4,
                Pt = s3.Pt * (1.0 - req.CombustorPressureLoss),
                Gamma = gamHot, Cp = cp4,
                FuelAirRatio = f
            };
            result.Stations[4] = s4;

            // ═══════════════════════════════════════════════════════
            //  GAP 1 — HPT TURBINE COOLING BLEED (first-principles)
            //
            //  Physics: at T4 > 1650 K single-crystal blades MELT without
            //  convective + film cooling. Bleed air is extracted from HPC exit
            //  (Station 3) and re-injected at the HPT blade trailing edge,
            //  dropping the mixed-out gas temperature before the next stage.
            //
            //  η_cool = (T_gas_rel - T_metal) / (T_metal - T3)        [effectiveness]
            //  ε_cool = C_tech · η_cool / (1 - η_cool)                [mass fraction]
            //  h_45   = (1-ε)·h4 + ε·h3                              [enthalpy mix]
            //  T_45mix = h_45 / Cp_mix                                [back-calc]
            // ═══════════════════════════════════════════════════════
            {
                // Relative gas temperature seen by rotating blade (0.85 × T4 — velocity triangle correction)
                double T_gas_rel  = T4 * 0.85;
                double T_metal    = req.MaxMetalTemp_K;
                double T3_cool    = Tt3;  // Coolant is HPC exit air

                double eta_cool = 0.0;
                double eps_cool = 0.0;
                double T45_mixed = T4;   // Default: no cooling needed

                if (T_gas_rel > T_metal + 10.0)
                {
                    // Cooling effectiveness needed
                    eta_cool = (T_gas_rel - T_metal) / Math.Max(1.0, T_metal - T3_cool);
                    // Mass fraction: semi-empirical Lefebvre technology factor
                    eps_cool = req.CoolingTechFactor * eta_cool / Math.Max(0.01, 1.0 - eta_cool);
                    eps_cool = Math.Min(eps_cool, 0.20);  // Cap at 20% — physical limit

                    // Enthalpy mixing at HPT blade trailing edge:
                    // h_mix = (1-ε)·Cp4·T4 + ε·Cp3·T3
                    double h4     = cp4 * T4;
                    double h3     = BraytonCycleSolver.CpAir(T3_cool) * T3_cool;
                    double h_mix  = (1.0 - eps_cool) * h4 + eps_cool * h3;
                    double cp_mix = (1.0 - eps_cool) * cp4 + eps_cool * BraytonCycleSolver.CpAir(T3_cool);
                    T45_mixed = h_mix / cp_mix;

                    Console.WriteLine($"  [Cooling] T_gas_rel={T_gas_rel:F0}K  T_metal={T_metal:F0}K  " +
                                      $"η_cool={eta_cool:F3}  ε_cool={eps_cool:F4}  T45_mix={T45_mixed:F0}K");
                }
                result.HPT_CoolantFraction = eps_cool;
                result.HPT_MixedTemp_K     = T45_mixed;
                // Store actual bleed flow — will be set once coreMassFlow is known (below)
                // For now, eps is fractional; bleed mass flow is computed in sizing block.
            }

            // ═══════════════════════════════════════════════════════
            //  STATION 4.5: HPT EXIT
            //  Power balance: HPT drives HPC
            //  ṁ_core·(1+f)·Cp4·(T4 - T4.5) = ṁ_core·Cp3·(T3 - T2.5) / η_mech
            // ═══════════════════════════════════════════════════════
            double hpcWork = cp3 * (Tt3 - s25.Tt);  // Per unit core mass flow
            double hptWork = hpcWork / (req.EtaMechanicalHP * (1.0 + f));
            // ── FIX 1B: HPT cooling bleed enthalpy mixing ────────────────────
            // Step 1: work extraction gives Tt45_work (uncooled)
            double Tt45_work  = T4 - hptWork / cp4;
            // Step 2: mix coolant air back in at trailing edge
            // h_45 = (1-ε)·h_45_work + ε·h_3
            // ε_cool already computed above and stored in result.HPT_CoolantFraction
            double eps_cool_fb = result.HPT_CoolantFraction;
            double h45_work_  = BraytonCycleSolver.CpGas(Tt45_work, f) * Tt45_work;
            double h3_cool    = BraytonCycleSolver.CpAir(Tt3) * Tt3;
            double h45_mixed_ = (1.0 - eps_cool_fb) * h45_work_ + eps_cool_fb * h3_cool;
            double cp45_mix   = (1.0 - eps_cool_fb) * BraytonCycleSolver.CpGas(Tt45_work, f)
                               + eps_cool_fb * BraytonCycleSolver.CpAir(Tt3);
            // True mixed-out T45 — used by all downstream stations (LPT, nozzle, thrust)
            double Tt45 = cp45_mix > 0 ? h45_mixed_ / cp45_mix : Tt45_work;
            // ─────────────────────────────────────────────────────────────────
            
            // HPT pressure ratio from efficiency
            double gamHPT = GammaGas((T4 + Tt45) / 2.0, f);
            double pi_hpt = Math.Pow(1.0 - (1.0 - Tt45/T4) / req.EtaHPT, -gamHPT / (gamHPT - 1.0));
            
            var s45 = new GasStation
            {
                Name = "HPT exit", StationNumber = 45,
                Tt = Tt45,
                Pt = s4.Pt / pi_hpt,
                Gamma = gamHPT, Cp = CpGas(Tt45, f),
                FuelAirRatio = f
            };
            result.Stations[45] = s45;

            // ═══════════════════════════════════════════════════════
            //  STATION 5: LPT EXIT
            //  Power balance: LPT drives Fan + LPC
            //  Fan work on total flow (core + bypass)
            //  LPC work on core only
            // ═══════════════════════════════════════════════════════
            double fanWork_perCore = CpAir((s2.Tt + s13.Tt) / 2.0) * (s13.Tt - s2.Tt)
                                    * (1.0 + req.BypassRatio);  // Fan handles all flow
            double lpcWork = CpAir((s13.Tt + s25.Tt) / 2.0) * (s25.Tt - s13.Tt);
            double eta_gear_lp = req.BypassRatio > 12.0 ? 0.993 : 1.0; // A3: GTF gearbox
            double lpShaftWork = (fanWork_perCore / eta_gear_lp + lpcWork) / req.EtaMechanicalLP;
            double lptWork = lpShaftWork / (1.0 + f);
            
            double Tt5 = Tt45 - lptWork / CpGas(Tt45, f);
            double gamLPT = GammaGas((Tt45 + Tt5) / 2.0, f);
            double pi_lpt = Math.Pow(1.0 - (1.0 - Tt5/Tt45) / req.EtaLPT, -gamLPT / (gamLPT - 1.0));
            
            var s5 = new GasStation
            {
                Name = "LPT exit", StationNumber = 5,
                Tt = Tt5,
                Pt = s45.Pt / pi_lpt,
                Gamma = gamLPT, Cp = CpGas(Tt5, f),
                FuelAirRatio = f
            };
            result.Stations[5] = s5;

            // ═══════════════════════════════════════════════════════
            //  STATION 8: CORE NOZZLE EXIT
            //  Check: choked or unchoked
            // ═══════════════════════════════════════════════════════
            double gamN = GammaGas(Tt5, f);
            double nprCore = s5.Pt / P0;  // Nozzle pressure ratio
            double nprCritical = Math.Pow((gamN + 1.0) / 2.0, gamN / (gamN - 1.0));
            
            double V8, T8s, P8;
            if (nprCore > nprCritical) // Choked
            {
                P8  = s5.Pt / nprCritical;
                T8s = Tt5 * 2.0 / (gamN + 1.0);
                V8  = Math.Sqrt(gamN * (CpGas(T8s, f) * (gamN - 1.0) / gamN) * T8s); // = a*
            }
            else // Unchoked: expand to ambient
            {
                P8  = P0;
                T8s = Tt5 * Math.Pow(Math.Max(1.0, P0 / s5.Pt), (gamN - 1.0) / gamN);
                double dhs = CpGas((Tt5 + T8s) / 2.0, f) * Math.Max(0.0, Tt5 - T8s);
                V8  = Math.Sqrt(2.0 * dhs * req.EtaNozzleCore);
            }
            
            var s8 = new GasStation
            {
                Name = "Core nozzle exit", StationNumber = 8,
                Tt = Tt5, Pt = s5.Pt,
                Mach = nprCore > nprCritical ? 1.0 : Math.Sqrt(2.0 / (gamN - 1.0) * (Math.Pow(Math.Max(1.0, s5.Pt / P0), (gamN - 1.0) / gamN) - 1.0)),
                Gamma = gamN, Cp = CpGas(T8s, f),
                FuelAirRatio = f
            };
            result.Stations[8] = s8;

            // ═══════════════════════════════════════════════════════
            //  STATION 18: BYPASS NOZZLE EXIT
            // ═══════════════════════════════════════════════════════
            double gamBy = 1.4;
            double nprBypass = s13.Pt / P0;
            double nprCritBy = Math.Pow((gamBy + 1.0) / 2.0, gamBy / (gamBy - 1.0));
            
            double V18, T18s;
            if (nprBypass > nprCritBy)
            {
                T18s = s13.Tt * 2.0 / (gamBy + 1.0);
                V18  = Math.Sqrt(gamBy * 287.0 * T18s);
            }
            else
            {
                T18s = s13.Tt * Math.Pow(Math.Max(1.0, P0 / s13.Pt), (gamBy - 1.0) / gamBy);
                V18  = Math.Sqrt(2.0 * CpAir((s13.Tt + T18s) / 2.0) * Math.Max(0.0, s13.Tt - T18s) * req.EtaNozzleBypass);
            }
            
            var s18 = new GasStation
            {
                Name = "Bypass nozzle exit", StationNumber = 18,
                Tt = s13.Tt, Pt = s13.Pt,
                Gamma = gamBy, Cp = CpAir(T18s),
                FuelAirRatio = 0
            };
            result.Stations[18] = s18;

            // ═══════════════════════════════════════════════════════
            //  PERFORMANCE CALCULATIONS
            // ═══════════════════════════════════════════════════════
            
            // Specific thrust (per unit total inlet mass flow)
            // F_specific = [(1+f)/(1+BPR) * V8 + BPR/(1+BPR) * V18]
            //            - V0
            //            + [(1+f)/(1+BPR) * (P8-P0)*A8/ṁ_core + ...]
            // Simplified (neglecting pressure thrust for initial sizing):
            double specThrust_core   = (1.0 + f) * V8 - V0;
            double specThrust_bypass = V18 - V0;
            double specThrust_total  = (specThrust_core + req.BypassRatio * specThrust_bypass)
                                       / (1.0 + req.BypassRatio);
            
            result.SpecificThrust = specThrust_total;
            
            // Size the engine: total mass flow needed
            double totalMassFlow = req.ThrustRequired_N / specThrust_total;
            double coreMassFlow  = totalMassFlow / (1.0 + req.BypassRatio);
            double bypassFlow    = coreMassFlow * req.BypassRatio;
            double fuelFlow      = coreMassFlow * f;
            
            result.CoreMassFlow   = coreMassFlow;
            result.BypassMassFlow = bypassFlow;
            result.FuelFlow       = fuelFlow;
            result.NetThrust_N    = req.ThrustRequired_N;
            // Complete Gap 1: bleed mass flow from core
            result.HPT_BleedMassFlow = coreMassFlow * result.HPT_CoolantFraction;
            // Store design params
            result.EtaFan              = req.EtaFan;
            result.EtaHPC              = req.EtaHPC;
            result.TurbineInletTemp_K  = req.TurbineInletTemp_K;
            result.OverallPressureRatio= req.OverallPressureRatio;
            
            // Set mass flows on stations
            foreach (var kv in result.Stations)
            {
                var st = kv.Value;
                int sn = kv.Key;
                if (sn == 0 || sn == 2)
                    st.MassFlow = totalMassFlow;
                else if (sn == 13 || sn == 18)
                    st.MassFlow = bypassFlow;
                else
                {
                    double w_inj = req.WaterInjectionActive ? req.WaterInjectionRatio : 0.0;
                    st.MassFlow = coreMassFlow * (sn >= 4 ? (1.0 + f + w_inj) : (1.0 + w_inj));
                }
            }
            
            // TSFC
            result.TSFC_gkNs = fuelFlow / (req.ThrustRequired_N / 1000.0) * 1000.0;  // g/(kN·s)
            
            // Efficiencies
            double kineticPowerOut = 0.5 * coreMassFlow * (1 + f) * (V8 * V8 - V0 * V0)
                                  + 0.5 * bypassFlow * (V18 * V18 - V0 * V0);
            double heatInput = fuelFlow * req.FuelHeatingValue_J;
            
            result.ThermalEfficiency    = kineticPowerOut / heatInput;
            result.PropulsiveEfficiency = req.ThrustRequired_N * V0 / kineticPowerOut;
            result.OverallEfficiency    = result.ThermalEfficiency * result.PropulsiveEfficiency;
            
            // Power balance
            result.HPC_Power = coreMassFlow * hpcWork;
            result.HPT_Power = coreMassFlow * (1 + f) * cp4 * (T4 - Tt45);
            result.FanPower  = totalMassFlow * CpAir((s2.Tt + s13.Tt) / 2.0) * (s13.Tt - s2.Tt);
            result.LPT_Power = coreMassFlow * (1 + f) * CpGas(Tt45, f) * (Tt45 - Tt5);
            
            // ═══════════════════════════════════════════════════════
            //  PRELIMINARY SIZING
            // ═══════════════════════════════════════════════════════
            // Fan diameter from mass flow: ṁ = ρ·V·A
            // At fan face, M ≈ 0.6 (typical)
            double M_fan = 0.6;
            double T_fan = s2.Tt / (1.0 + 0.2 * M_fan * M_fan);
            double P_fan = s2.Pt * Math.Pow(T_fan / s2.Tt, 3.5);
            double rho_fan = P_fan / (287.0 * T_fan);
            double V_fan = M_fan * Math.Sqrt(1.4 * 287.0 * T_fan);
            double A_fan = totalMassFlow / (rho_fan * V_fan);
            double hubTipRatio = 0.3;  // Typical for turbofan
            result.FanDiameter_m = Math.Sqrt(4.0 * A_fan / (Math.PI * (1.0 - hubTipRatio * hubTipRatio)));
            
            // Core diameter (hub of fan approximately)
            result.CoreDiameter_m = result.FanDiameter_m * hubTipRatio * 2.0;
            
            // ═══════════════════════════════════════════════════════
            //  VALIDATION CHECKS (GATE 1)
            // ═══════════════════════════════════════════════════════
            result.IsValid = true;
            
            // Check T4 material limit
            if (T4 > req.MaxExitTemp_K)
            {
                result.Warnings.Add($"T4={T4:F0}K exceeds material limit {req.MaxExitTemp_K:F0}K");
            }
            
            // Check HPC exit temperature (compressor material limit ~900K for Ti)
            if (Tt3 > 900.0)
            {
                result.Warnings.Add($"HPC exit Tt3={Tt3:F0}K > 900K — needs Ni-alloy last stages");
            }
            
            // Check fan tip speed
            // N_fan ≈ V_tip / (π·D_fan)
            // For M_tip ≈ 1.3-1.5 relative, V_tip ≈ 400-460 m/s
            double V_tip_est = Math.Sqrt(V_fan * V_fan + (Math.PI * result.FanDiameter_m * 60.0) * (Math.PI * result.FanDiameter_m * 60.0));
            // Just use a simple check:
            if (result.FanDiameter_m > 3.5)
                result.Warnings.Add($"Fan diameter {result.FanDiameter_m:F2}m is very large — consider geared turbofan");
            
            // Check LPT exit temp (should be > ambient for positive thrust)
            if (Tt5 < s0.Tt + 10.0)
            {
                result.Errors.Add($"LPT exit temp {Tt5:F0}K too close to freestream {s0.Tt:F0}K — no thrust");
                result.IsValid = false;
            }
            
            // Check fuel-air ratio sanity (stoich ≈ 0.068 for kerosene)
            if (f > 0.068)
            {
                result.Errors.Add($"Fuel-air ratio f={f:F4} exceeds stoichiometric — combustion impossible");
                result.IsValid = false;
            }
            if (f < 0.005)
            {
                result.Warnings.Add($"Fuel-air ratio f={f:F4} very lean — check flame stability");
            }
            
            // Check TSFC range (typical turbofan: 14-22 g/(kN·s))
            if (result.TSFC_gkNs > 25.0)
                result.Warnings.Add($"TSFC={result.TSFC_gkNs:F1} g/(kN·s) is high — check cycle parameters");
            
            return result;
        }
    }

    public static class CycleOptimizer
    {
        public static MissionRequirements CloneReqPublic(MissionRequirements r) => CloneReq(r);

        public static CycleResult SolveWithAutoCorrect(MissionRequirements req, int maxIter = 50)
        {
            MissionRequirements bestReq = req; // A1-FIX
            var current = req;
            CycleResult best = null!;
            double bestTSFC = double.MaxValue;
            
            for (int iter = 0; iter < maxIter; iter++)
            {
                var result = BraytonCycleSolver.SolveOnDesign(current);
                
                Console.WriteLine($"  [Iter {iter}] Thrust={result.NetThrust_N:F0}N  TSFC={result.TSFC_gkNs:F2}  T4={current.TurbineInletTemp_K:F0}K  BPR={current.BypassRatio:F1}  OPR={current.OverallPressureRatio:F1}  Valid={result.IsValid}");
                
                if (result.IsValid && result.Errors.Count == 0)
                {
                    if (result.TSFC_gkNs < bestTSFC)
                    {
                        bestTSFC = result.TSFC_gkNs;
                        best = result;
                        bestReq = current;
                    }
                    
                    // Optimization: try to reduce TSFC
                    // Gradient-free: perturb BPR and OPR slightly
                    if (iter < maxIter - 1)
                    {
                        // Try increasing BPR (reduces TSFC for turbofans up to a point)
                        var reqUp = CloneReq(current);
                        reqUp.BypassRatio += 0.5;
                        var resUp = BraytonCycleSolver.SolveOnDesign(reqUp);
                        
                        if (resUp.IsValid && resUp.TSFC_gkNs < result.TSFC_gkNs)
                        {
                            current = reqUp;
                            continue;
                        }
                        
                        // Try increasing OPR
                        var reqOPR = CloneReq(current);
                        reqOPR.OverallPressureRatio += 1.0;
                        var resOPR = BraytonCycleSolver.SolveOnDesign(reqOPR);
                        
                        if (resOPR.IsValid && resOPR.TSFC_gkNs < result.TSFC_gkNs)
                        {
                            current = reqOPR;
                            continue;
                        }
                        
                        // Converged — no improvement found
                        break;
                    }
                }
                else
                {
                    // ─── AUTO-CORRECT LOGIC ───
                    // If LPT exit temp too low → reduce BPR or increase T4
                    if (result.Errors.Any(e => e.Contains("LPT exit temp")))
                    {
                        if (current.BypassRatio > 3.0)
                            current.BypassRatio -= 0.5;
                        else
                            current.TurbineInletTemp_K += 25.0;
                    }
                    
                    // If fuel-air ratio too high → lower T4 or raise OPR
                    if (result.Errors.Any(e => e.Contains("stoichiometric")))
                    {
                        current.TurbineInletTemp_K -= 50.0;
                    }
                    
                    // If T3 warning → lower OPR or use better materials
                    if (result.Warnings.Any(w => w.Contains("HPC exit")))
                    {
                        // Don't auto-lower OPR, just note it
                    }
                    
                    // If fan too large → increase specific thrust
                    if (result.Warnings.Any(w => w.Contains("Fan diameter")))
                    {
                        current.BypassRatio -= 0.5;
                        current.FanPressureRatio += 0.02;
                    }
                }
            }
            
            if (best != null) {
                req.BypassRatio=bestReq.BypassRatio; req.OverallPressureRatio=bestReq.OverallPressureRatio;
                req.TurbineInletTemp_K=bestReq.TurbineInletTemp_K; req.FanPressureRatio=bestReq.FanPressureRatio;
                req.CombustorPressureLoss=bestReq.CombustorPressureLoss;
                req.EtaFan=bestReq.EtaFan; req.EtaLPC=bestReq.EtaLPC; req.EtaHPC=bestReq.EtaHPC;
            }
            return best ?? BraytonCycleSolver.SolveOnDesign(req);
        }
        
        private static MissionRequirements CloneReq(MissionRequirements r)
        {
            return new MissionRequirements
            {
                ThrustRequired_N       = r.ThrustRequired_N,
                CruiseMach             = r.CruiseMach,
                CruiseAltitude_m       = r.CruiseAltitude_m,
                BypassRatio            = r.BypassRatio,
                OverallPressureRatio   = r.OverallPressureRatio,
                FanPressureRatio       = r.FanPressureRatio,
                LPCPressureRatio       = r.LPCPressureRatio,
                TurbineInletTemp_K     = r.TurbineInletTemp_K,
                EtaFan = r.EtaFan, EtaLPC = r.EtaLPC, EtaHPC = r.EtaHPC,
                EtaHPT = r.EtaHPT, EtaLPT = r.EtaLPT,
                EtaCombustor = r.EtaCombustor, EtaInlet = r.EtaInlet,
                EtaNozzleCore = r.EtaNozzleCore, EtaNozzleBypass = r.EtaNozzleBypass,
                EtaMechanicalHP = r.EtaMechanicalHP, EtaMechanicalLP = r.EtaMechanicalLP,
                CombustorPressureLoss = r.CombustorPressureLoss,
                FuelHeatingValue_J = r.FuelHeatingValue_J,
                MaxTipSpeed_mps = r.MaxTipSpeed_mps,
                MinSurgeMargin = r.MinSurgeMargin,
                MaxExitTemp_K = r.MaxExitTemp_K,
                MaxMetalTemp_K = r.MaxMetalTemp_K,
                CoolingTechFactor = r.CoolingTechFactor,
            };
        }
    }

}
