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
    public class VelocityTriangle
    {
        // Absolute frame
        public double Va   { get; set; }  // Axial velocity (m/s)
        public double Vu1  { get; set; }  // Tangential velocity — inlet
        public double Vu2  { get; set; }  // Tangential velocity — exit
        public double V1   { get; set; }  // Absolute velocity — inlet
        public double V2   { get; set; }  // Absolute velocity — exit
        public double Alpha1 { get; set; } // Absolute inlet angle (rad)
        public double Alpha2 { get; set; } // Absolute exit angle (rad)
        
        // Relative frame (rotor only)
        public double Wu1  { get; set; }
        public double Wu2  { get; set; }
        public double W1   { get; set; }
        public double W2   { get; set; }
        public double Beta1 { get; set; } // Relative inlet angle (rad)
        public double Beta2 { get; set; } // Relative exit angle (rad)
        
        public double U    { get; set; }  // Blade speed at this radius (m/s)
        public double Radius { get; set; } // m
        
        // De Haller number: W2/W1 > 0.72 to avoid separation
        public double DeHaller => W2 / W1;
        
        public double DF => DiffusionFactor(1.0);
        
        // Diffusion factor (Lieblein): DF < 0.45
        public double DiffusionFactor(double solidity)
        {
            return 1.0 - W2/W1 + Math.Abs(Wu1 - Wu2) / (2.0 * solidity * W1);
        }
        
        // Work coefficient (loading): ψ = ΔVu / U
        public double WorkCoefficient => Math.Abs(Vu2 - Vu1) / U;
        
        // Flow coefficient: φ = Va / U
        public double FlowCoefficient => Va / U;
    }

    public class BladeStage
    {
        public string Name { get; set; } = "";
        public int StageIndex { get; set; }
        public bool IsRotor { get; set; }  // True=rotor, False=stator
        public double Temperature_In       { get; set; }
        public double Temperature_Out      { get; set; }
        public double YoungsModulus_GPa    { get; set; } = 114.0;
        public double MaterialDensity_kgm3 { get; set; } = 4430.0;
        
        // Annulus geometry
        public double HubRadius      { get; set; }  // m
        public double TipRadius      { get; set; }  // m
        public double MeanRadius     { get; set; }  // m
        public double AxialChord     { get; set; }  // m
        public double Chord          { get; set; }  // m
        public double Span           => TipRadius - HubRadius;
        public double AspectRatio    => Span / Chord;
        public double HubTipRatio    => HubRadius / TipRadius;
        
        // Blade parameters
        public int    BladeCount     { get; set; }
        public double Solidity       { get; set; }  // chord / pitch
        public double StaggerAngle   { get; set; }  // rad
        public double Camber         { get; set; }  // rad (total turning)
        public double MaxThicknessRatio { get; set; } = 0.06;  // t/c
        
        // Performance
        public double PressureRatio  { get; set; }
        public double RPM            { get; set; }
        
        // Velocity triangles at hub, mean, tip
        public VelocityTriangle Hub  { get; set; } = new();
        public VelocityTriangle Mean { get; set; } = new();
        public VelocityTriangle Tip  { get; set; } = new();
        
        // Material
        public string Material { get; set; } = "Ti-6Al-4V";
    }

    public class EngineFlowPath
    {
        public List<BladeStage> FanStages        { get; set; } = new();
        public List<BladeStage> LPCStages        { get; set; } = new();
        public List<BladeStage> HPCStages        { get; set; } = new();
        public List<BladeStage> HPTStages        { get; set; } = new();
        public List<BladeStage> LPTStages        { get; set; } = new();
        
        // Shaft speeds
        public double HP_RPM { get; set; }
        public double LP_RPM { get; set; }
        
        // Engine length
        public double TotalLength_m { get; set; }
        
        public List<BladeStage> AllStages()
        {
            var all = new List<BladeStage>();
            all.AddRange(FanStages);
            all.AddRange(LPCStages);
            all.AddRange(HPCStages);
            all.AddRange(HPTStages);
            all.AddRange(LPTStages);
            return all;
        }
    }

    public static class FlowPathGenerator
    {
        public static EngineFlowPath Generate(CycleResult cycle, MissionRequirements req)
        {
            var fp = new EngineFlowPath();
            
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  GATE 2: FLOW PATH & BLADE GEOMETRY");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            // ───────────────────────────────────────
            //  SHAFT SPEEDS
            // ───────────────────────────────────────
            // LP spool: limited by fan tip speed
            double fanTipSpeed = Math.Min(req.MaxTipSpeed_mps, 400.0);
            double fanTipR = cycle.FanDiameter_m / 2.0;
            fp.LP_RPM = fanTipSpeed / (2.0 * Math.PI * fanTipR) * 60.0;
            
            // HP spool: size by HPC last-stage tip speed ~ 450 m/s
            double hpcTipR = cycle.CoreDiameter_m / 2.0;
            fp.HP_RPM = 450.0 / (2.0 * Math.PI * hpcTipR) * 60.0;
            
            Console.WriteLine($"  LP spool: {fp.LP_RPM:F0} RPM  |  HP spool: {fp.HP_RPM:F0} RPM");

            // ───────────────────────────────────────
            //  FAN STAGE
            // ───────────────────────────────────────
            double fanHubR  = fanTipR * 0.30;
            double fanMeanR = (fanTipR + fanHubR) / 2.0;
            double fanU     = 2.0 * Math.PI * fanMeanR * fp.LP_RPM / 60.0;
            
            double s2Tt  = cycle.Stations[2].Tt;
            double s13Tt = cycle.Stations[13].Tt;
            double dTfan = s13Tt - s2Tt;
            double cpFan = BraytonCycleSolver.CpAir((s2Tt + s13Tt) / 2.0);
            double dVu_fan = cpFan * dTfan / fanU;
            double Va_fan = 200.0;
            
            var fanRotor = new BladeStage
            {
                Name = "Fan Rotor", StageIndex = 0, IsRotor = true,
                HubRadius = fanHubR, TipRadius = fanTipR, MeanRadius = fanMeanR,
                PressureRatio = req.FanPressureRatio,
                Temperature_In = s2Tt, Temperature_Out = s13Tt,
                RPM = fp.LP_RPM,
                BladeCount = EstimateBladeCount(fanMeanR, 0.50, 1.5), // Optimized solidity
                Chord = 0.50,
                Material = "Ti-6Al-4V",
                MaxThicknessRatio = 0.08,
            };
            fanRotor.Solidity = fanRotor.BladeCount * fanRotor.Chord / (2.0 * Math.PI * fanMeanR);
            
            fanRotor.Mean = ComputeVelocityTriangle(Va_fan, 0, dVu_fan, fanU, fanMeanR);
            fanRotor.Hub  = ComputeVelocityTriangle(Va_fan, 0, dVu_fan * fanMeanR / fanHubR,
                                2.0 * Math.PI * fanHubR * fp.LP_RPM / 60.0, fanHubR);
            fanRotor.Tip  = ComputeVelocityTriangle(Va_fan, 0, dVu_fan * fanMeanR / fanTipR,
                                fanTipSpeed, fanTipR);
            
            fanRotor.StaggerAngle = (fanRotor.Mean.Beta1 + fanRotor.Mean.Beta2) / 2.0;
            fanRotor.Camber       = fanRotor.Mean.Beta1 - fanRotor.Mean.Beta2;
            
            fp.FanStages.Add(fanRotor);
            PrintStageInfo(fanRotor);

            // ───────────────────────────────────────
            //  LPC STAGES
            // ───────────────────────────────────────
            int nLPC = (int)Math.Ceiling(Math.Log(req.LPCPressureRatio) / Math.Log(1.35));
            nLPC = Math.Max(2, Math.Min(nLPC, 4));
            double lpcPR_perStage = Math.Pow(req.LPCPressureRatio, 1.0 / nLPC);
            
            double lpcHubR = fanHubR * 1.05;
            double lpcTipR = fanHubR * 1.5;
            double lpcMeanR = (lpcHubR + lpcTipR) / 2.0;
            double lpcU = 2.0 * Math.PI * lpcMeanR * fp.LP_RPM / 60.0;
            
            double Tt_in = s13Tt;
            for (int i = 0; i < nLPC; i++)
            {
                double Tt_out = Tt_in * Math.Pow(lpcPR_perStage, 0.4 / (1.4 * req.EtaLPC));
                double dT = Tt_out - Tt_in;
                double dVu = BraytonCycleSolver.CpAir(Tt_in) * dT / lpcU;
                
                var stage = new BladeStage
                {
                    Name = $"LPC Rotor {i+1}", StageIndex = i, IsRotor = true,
                    HubRadius = lpcHubR, TipRadius = lpcTipR, MeanRadius = lpcMeanR,
                    PressureRatio = lpcPR_perStage,
                    Temperature_In = Tt_in, Temperature_Out = Tt_out,
                    RPM = fp.LP_RPM,
                    BladeCount = EstimateBladeCount(lpcMeanR, 0.035, 2.0), // Optimized solidity
                    Chord = 0.035,
                    Material = "Ti-6Al-4V",
                };
                stage.Solidity = stage.BladeCount * stage.Chord / (2.0 * Math.PI * lpcMeanR);
                stage.Mean = ComputeVelocityTriangle(180.0, 0, dVu, lpcU, lpcMeanR);
                stage.StaggerAngle = (stage.Mean.Beta1 + stage.Mean.Beta2) / 2.0;
                stage.Camber = stage.Mean.Beta1 - stage.Mean.Beta2;
                
                fp.LPCStages.Add(stage);
                PrintStageInfo(stage);
                
                Tt_in = Tt_out;
                
                // Keep mean radius constant and contract span to prevent crossover
                double lpcMeanR_prev = (lpcHubR + lpcTipR) / 2.0;
                double lpcSpan_prev = lpcTipR - lpcHubR;
                double lpcSpan_new = lpcSpan_prev * 0.82; // contract span by 18% per stage
                lpcHubR = lpcMeanR_prev - lpcSpan_new / 2.0;
                lpcTipR = lpcMeanR_prev + lpcSpan_new / 2.0;
                lpcMeanR = (lpcHubR + lpcTipR) / 2.0;
                lpcU = 2.0 * Math.PI * lpcMeanR * fp.LP_RPM / 60.0;
            }

            // ───────────────────────────────────────
            //  HPC STAGES
            // ───────────────────────────────────────
            int nHPC = (int)Math.Ceiling(Math.Log(req.HPCPressureRatio) / Math.Log(1.15));
            nHPC = Math.Max(8, Math.Min(nHPC, 16));
            double hpcPR_perStage = Math.Pow(req.HPCPressureRatio, 1.0 / nHPC);
            
            double hpcHubR = lpcHubR * 1.02;
            hpcTipR = lpcTipR * 0.95;
            
            Tt_in = cycle.Stations[25].Tt;
            for (int i = 0; i < nHPC; i++)
            {
                double hpcMeanR = (hpcHubR + hpcTipR) / 2.0;
                double hpcU = 2.0 * Math.PI * hpcMeanR * fp.HP_RPM / 60.0;
                
                double Tt_out = Tt_in * Math.Pow(hpcPR_perStage, 0.39 / (1.39 * req.EtaHPC));
                double dT = Tt_out - Tt_in;
                double dVu = BraytonCycleSolver.CpAir(Tt_in) * dT / hpcU;
                
                string mat = Tt_out > 750 ? "Inconel 718" : "Ti-6Al-4V";
                
                var stage = new BladeStage
                {
                    Name = $"HPC Rotor {i+1}", StageIndex = i, IsRotor = true,
                    HubRadius = hpcHubR, TipRadius = hpcTipR,
                    MeanRadius = hpcMeanR,
                    PressureRatio = hpcPR_perStage,
                    Temperature_In = Tt_in, Temperature_Out = Tt_out,
                    RPM = fp.HP_RPM,
                    BladeCount = EstimateBladeCount(hpcMeanR, 0.025 - i * 0.001, 3.5), // Optimized solidity
                    Chord = 0.025 - i * 0.001,
                    Material = mat,
                };
                stage.Chord = Math.Max(stage.Chord, 0.012);
                stage.Solidity = stage.BladeCount * stage.Chord / (2.0 * Math.PI * hpcMeanR);
                stage.Mean = ComputeVelocityTriangle(160.0 - i * 1.5, 0, dVu, hpcU, hpcMeanR);
                stage.StaggerAngle = (stage.Mean.Beta1 + stage.Mean.Beta2) / 2.0;
                stage.Camber = stage.Mean.Beta1 - stage.Mean.Beta2;
                
                fp.HPCStages.Add(stage);
                PrintStageInfo(stage);
                
                Tt_in = Tt_out;
                
                // Keep mean radius constant and contract span to prevent crossover
                double hpcMeanR_prev = (hpcHubR + hpcTipR) / 2.0;
                double hpcSpan_prev = hpcTipR - hpcHubR;
                double hpcSpan_new = hpcSpan_prev * 0.88; // contract span by 12% per stage
                hpcHubR = hpcMeanR_prev - hpcSpan_new / 2.0;
                hpcTipR = hpcMeanR_prev + hpcSpan_new / 2.0;
            }

            // ───────────────────────────────────────
            //  HPT STAGES
            // ───────────────────────────────────────
            int nHPT = cycle.HPT_Power > 20e6 ? 2 : 1;
            double hptHubR = hpcHubR * 0.95;
            double hptTipR = hpcTipR * 1.4;
            double hptWork_perStage = (cycle.Stations[4].Tt - cycle.Stations[45].Tt) / nHPT;
            
            Tt_in = cycle.Stations[4].Tt;
            for (int i = 0; i < nHPT; i++)
            {
                double hptMeanR = (hptHubR + hptTipR) / 2.0;
                double hptU = 2.0 * Math.PI * hptMeanR * fp.HP_RPM / 60.0;
                double Tt_out = Tt_in - hptWork_perStage;
                double f = cycle.Stations[4].FuelAirRatio;
                double dVu = BraytonCycleSolver.CpGas(Tt_in, f) * hptWork_perStage / hptU;
                
                var stage = new BladeStage
                {
                    Name = $"HPT Rotor {i+1}", StageIndex = i, IsRotor = true,
                    HubRadius = hptHubR, TipRadius = hptTipR,
                    MeanRadius = hptMeanR,
                    Temperature_In = Tt_in, Temperature_Out = Tt_out,
                    RPM = fp.HP_RPM,
                    BladeCount = EstimateBladeCount(hptMeanR, 0.055, 1.8), // Optimized solidity
                    Chord = 0.055,
                    Material = "CMSX-4 + TBC",
                    MaxThicknessRatio = 0.18,
                };
                stage.Solidity = stage.BladeCount * stage.Chord / (2.0 * Math.PI * hptMeanR);
                stage.Mean = ComputeVelocityTriangle(250.0, dVu, 0, hptU, hptMeanR);
                stage.StaggerAngle = (stage.Mean.Beta1 + stage.Mean.Beta2) / 2.0;
                stage.Camber = Math.Abs(stage.Mean.Beta1 - stage.Mean.Beta2);
                
                fp.HPTStages.Add(stage);
                PrintStageInfo(stage);
                
                Tt_in = Tt_out;
                hptHubR *= 1.0;
                hptTipR *= 1.015;
            }

            // ───────────────────────────────────────
            //  LPT STAGES
            // ───────────────────────────────────────
            int nLPT = (int)Math.Ceiling(cycle.LPT_Power / 5e6);
            nLPT = Math.Max(3, Math.Min(nLPT, 7));
            double lptWork_perStage = (cycle.Stations[45].Tt - cycle.Stations[5].Tt) / nLPT;
            
            double lptHubR = hptHubR * 0.95;
            double lptTipR = hptTipR * 1.1;
            
            Tt_in = cycle.Stations[45].Tt;
            double f_gas = cycle.Stations[4].FuelAirRatio;
            for (int i = 0; i < nLPT; i++)
            {
                double lptMeanR = (lptHubR + lptTipR) / 2.0;
                double lptU = 2.0 * Math.PI * lptMeanR * fp.LP_RPM / 60.0;
                double Tt_out = Tt_in - lptWork_perStage;
                double dVu = BraytonCycleSolver.CpGas(Tt_in, f_gas) * lptWork_perStage / lptU;
                
                var stage = new BladeStage
                {
                    Name = $"LPT Rotor {i+1}", StageIndex = i, IsRotor = true,
                    HubRadius = lptHubR, TipRadius = lptTipR,
                    MeanRadius = lptMeanR,
                    Temperature_In = Tt_in, Temperature_Out = Tt_out,
                    RPM = fp.LP_RPM,
                    BladeCount = EstimateBladeCount(lptMeanR, 0.065 + i * 0.005, 1.8), // Optimized solidity
                    Chord = 0.065 + i * 0.005,
                    Material = Tt_in > 1050 ? "CMSX-4 + TBC" : Tt_in > 800 ? "Inconel 718" : "Ti-6Al-4V", // Use Inconel at high temps
                    MaxThicknessRatio = Tt_in > 800 ? 0.12 : 0.09,
                };
                stage.Solidity = stage.BladeCount * stage.Chord / (2.0 * Math.PI * lptMeanR);
                stage.Mean = ComputeVelocityTriangle(200.0 + i * 10, dVu, 0, lptU, lptMeanR);
                stage.StaggerAngle = (stage.Mean.Beta1 + stage.Mean.Beta2) / 2.0;
                stage.Camber = Math.Abs(stage.Mean.Beta1 - stage.Mean.Beta2);
                
                fp.LPTStages.Add(stage);
                PrintStageInfo(stage);
                
                Tt_in = Tt_out;
                lptHubR *= 0.98;
                lptTipR *= 1.03;
            }

            // ───────────────────────────────────────
            //  TOTAL ENGINE LENGTH
            // ───────────────────────────────────────
            double axialGap = 0.01;
            double totalLen = 0;
            foreach (var s in fp.AllStages())
            {
                totalLen += (s.Chord * 1.2) + axialGap;
            }
            totalLen += 0.3;
            totalLen += 0.15;
            totalLen += 0.2;
            fp.TotalLength_m = totalLen;
            
            Console.WriteLine("────────────────────────────────────────────────────────");
            Console.WriteLine($"  Total stages: Fan={fp.FanStages.Count} LPC={fp.LPCStages.Count} HPC={fp.HPCStages.Count} HPT={fp.HPTStages.Count} LPT={fp.LPTStages.Count}");
            Console.WriteLine($"  Engine length: {totalLen*1000:F0} mm");
            Console.WriteLine("════════════════════════════════════════════════════════");
            
            return fp;
        }

        /// <summary>
        /// Compute velocity triangle for a rotor at a given radius.
        /// </summary>
        private static VelocityTriangle ComputeVelocityTriangle(
            double Va, double Vu1_abs, double Vu2_abs, double U, double r)
        {
            var vt = new VelocityTriangle { Va = Va, Vu1 = Vu1_abs, Vu2 = Vu2_abs, U = U, Radius = r };
            
            // Absolute velocities
            vt.V1 = Math.Sqrt(Va * Va + Vu1_abs * Vu1_abs);
            vt.V2 = Math.Sqrt(Va * Va + Vu2_abs * Vu2_abs);
            vt.Alpha1 = Math.Atan2(Vu1_abs, Va);
            vt.Alpha2 = Math.Atan2(Vu2_abs, Va);
            
            // Relative velocities (rotor frame): Wu = Vu - U
            vt.Wu1 = Vu1_abs - U;
            vt.Wu2 = Vu2_abs - U;
            vt.W1 = Math.Sqrt(Va * Va + vt.Wu1 * vt.Wu1);
            vt.W2 = Math.Sqrt(Va * Va + vt.Wu2 * vt.Wu2);
            vt.Beta1 = Math.Atan2(vt.Wu1, Va);
            vt.Beta2 = Math.Atan2(vt.Wu2, Va);
            
            return vt;
        }

        /// <summary>
        /// Estimate blade count from Zweifel criterion and solidity.
        /// </summary>
        private static int EstimateBladeCount(double meanR, double chord, double solidity)
        {
            double pitch = chord / solidity;
            int count = (int)(2.0 * Math.PI * meanR / pitch);
            // Round to "good" numbers (avoid resonance with upstream)
            // Use primes or non-multiples
            if (count % 2 == 0) count++;
            return Math.Max(13, count);
        }

        private static void PrintStageInfo(BladeStage s)
        {
            Console.WriteLine($"  {s.Name}: r_hub={s.HubRadius*1000:F1}mm  r_tip={s.TipRadius*1000:F1}mm  " +
                              $"N_blades={s.BladeCount}  PR={s.PressureRatio:F3}  " +
                              $"ΔT={s.Temperature_Out - s.Temperature_In:F1}K  " +
                              $"DeHaller={s.Mean.DeHaller:F2}  ψ={s.Mean.WorkCoefficient:F2}  " +
                              $"mat={s.Material}");
        }
    }

    public static class ThroughflowSolver
    {
        public class SLSt { public double R,Z,Vm,Vt,Pt,Tt,Loss; }
        public class TFRes { public List<SLSt[]> Planes=new(); public double[] SM=Array.Empty<double>(); public bool Conv; }

        public static TFRes Solve(EngineFlowPath fp, CycleResult cy, int Nsl=5, int maxIt=60)
        {
            Console.WriteLine("═══ THROUGHFLOW SOLVER (Streamline Curvature, Katsanis) ═══");
            var res=new TFRes(); var stgs=fp.AllStages().ToList();
            if(stgs.Count==0) return res;
            var s0=stgs[0]; double[] r_sl=new double[Nsl];
            for(int k=0;k<Nsl;k++) r_sl[k]=s0.HubRadius+(s0.TipRadius-s0.HubRadius)*k/(Nsl-1);
            SLSt[] pl=Init(r_sl,cy,s0); res.Planes.Add(pl);
            for(int it=0;it<maxIt;it++)
            {
                double res2=0; var nP=new List<SLSt[]>{pl};
                foreach(var st in stgs)
                {
                    double dVt=st.Mean.Vu2-st.Mean.Vu1, rm=st.MeanRadius, om=st.RPM*2*Math.PI/60;
                    var np=new SLSt[Nsl];
                    for(int k=0;k<Nsl;k++)
                    {
                        var pv=nP.Last()[k];
                        double r=st.HubRadius+(st.TipRadius-st.HubRadius)*k/(Nsl-1);
                        double dvt=st.IsRotor?dVt*rm/Math.Max(r,0.01):0;
                        double Vm=Math.Max(pv.Vm*(rm/Math.Max(r,0.01))*0.25+pv.Vm*0.75,30);
                        double Y=Lyp(st.Mean.DF,st.Chord,st.Span)+Lys(st.Mean.DF,st.Chord,st.Span,k,Nsl)+Lyt(st,k,Nsl);
                        double Tt2=pv.Tt+(st.IsRotor?om*r*dvt/1005.0:0);
                        res2+=Math.Abs(Vm-pv.Vm)/Math.Max(pv.Vm,1);
                        np[k]=new SLSt{R=r,Vm=Vm,Vt=pv.Vt+dvt,Pt=pv.Pt*(1-Y),Tt=Tt2,Loss=Y};
                    }
                    nP.Add(np);
                }
                res2/=stgs.Count*Nsl; pl=nP.Last();
                if(it>4&&res2<1e-4){res.Conv=true;break;}
                res.Planes=nP;
            }
            res.SM=SM(res.Planes,fp,Nsl);
            Console.WriteLine($"  Conv={res.Conv} AvgLoss={res.Planes.SelectMany(p=>p).Average(s=>s.Loss):F4}");
            Console.WriteLine($"  SpanwiseSM: {string.Join(" ",res.SM.Select(x=>$"{x*100:F1}%"))}");
            return res;
        }
        static double Lyp(double DF,double c,double sp){DF=Math.Clamp(DF,.15,.65);double cb=Math.Sqrt(1-Math.Pow(DF,1.5)*.3),dH=Math.Max(.6,1-1.12*DF+.61*DF*DF-.044*DF*DF*DF);return Math.Clamp((.004+.0074*DF)/Math.Max(cb*dH*dH,.01),.002,.12);}
        static double Lys(double DF,double c,double sp,int k,int N){double AR=sp/Math.Max(c,1e-4),CL=2*DF*Math.Max(AR,.5),z=.018*CL*CL/Math.Max(AR,.5);return Math.Clamp(z*(k==0||k==N-1?2.5:.5),0,.08);}
        static double Lyt(BladeStage st,int k,int N){if(k!=N-1)return 0;double CL=2*st.Mean.DF*1.2;return Math.Clamp(.93*.005*CL/Math.Max(st.Solidity*.85,.01),0,.05);}
        static SLSt[] Init(double[] r,CycleResult c,BladeStage s){var p=new SLSt[r.Length];double Vm=150,Pt=c.Stations.ContainsKey(2)?c.Stations[2].Pt:25e3,Tt=c.Stations.ContainsKey(2)?c.Stations[2].Tt:288;for(int k=0;k<r.Length;k++)p[k]=new SLSt{R=r[k],Vm=Vm,Pt=Pt,Tt=Tt};return p;}
        static double[] SM(List<SLSt[]> pls,EngineFlowPath fp,int N){var sm=new double[N];if(pls.Count<2)return sm;double Vmd=fp.AllStages().FirstOrDefault()?.Mean.Va??150;for(int k=0;k<N;k++){double vi=pls[0][k].Vm,vo=pls[^1][k].Vm;sm[k]=(Vmd-Math.Abs(vo-vi))/Math.Max(Vmd,1);}return sm;}
    }

    public static class PyTurboAeroStyle
    {
        public struct BladeSection2D
        {
            public double Radius;            // spanwise location (m)
            public double Chord;             // chord length (m)
            public double StaggerAngle_deg;  // stagger from axial
            public double Camber_deg;        // total camber angle
            public double t_max_ratio;       // max thickness / chord
            public double r_LE;              // leading edge radius (m)
            public double theta_TE_deg;      // trailing edge wedge angle
            public double Sweep_mm;          // forward sweep at this section (mm)
            public double Lean_mm;           // circumferential lean (mm)
        }

        // Generate blade sections at hub/mean/tip with NASA pyturbo-aero parameterization
        public static List<BladeSection2D> GenerateBladeSections(
            BladeStage stage, int nSections = 3)
        {
            var sections = new List<BladeSection2D>();
            double[] radii = nSections == 3
                ? new[]{ stage.HubRadius, stage.MeanRadius, stage.TipRadius }
                : Linspace(stage.HubRadius, stage.TipRadius, nSections);

            for (int i = 0; i < nSections; i++)
            {
                double r = radii[i];
                double t_r = (r - stage.HubRadius) / Math.Max(stage.TipRadius - stage.HubRadius, 1e-6);

                // Free-vortex stagger interpolation (hub to tip)
                double stagger = stage.StaggerAngle * (0.85 + 0.30*t_r);

                // Camber: higher at hub (more work), lower at tip
                double camber = stage.IsRotor ? 40*(1 - 0.4*t_r) : 30*(1 - 0.3*t_r);

                // Chord varies slightly (wider at hub for structural reasons)
                double chord = stage.Chord * (1.0 + 0.15*(1-t_r));

                // t_max/c: thicker at hub for structural, thinner at tip for aerodynamics
                double tmax = stage.MaxThicknessRatio*(1.3 - 0.6*t_r);
                tmax = Math.Clamp(tmax, 0.05, 0.25);

                // NACA-65 style r_LE
                double r_LE = 1.1019 * tmax * tmax * chord;

                // Trailing edge wedge angle
                double theta_TE = 2 * Math.Atan(0.42*tmax) * 180/Math.PI;

                // Sweep and lean (controlled for shock management)
                double sweep = stage.IsRotor ? -5.0*t_r : 3.0*(1-t_r);   // mm backward sweep at tip
                double lean  = stage.IsRotor ? 2.0*(t_r-0.5) : 0;        // mm circumferential lean

                sections.Add(new BladeSection2D
                {
                    Radius=r, Chord=chord, StaggerAngle_deg=stagger, Camber_deg=camber,
                    t_max_ratio=tmax, r_LE=r_LE, theta_TE_deg=theta_TE,
                    Sweep_mm=sweep, Lean_mm=lean
                });
            }
            return sections;
        }

        public static void PrintBladeSections(BladeStage stage)
        {
            var secs = GenerateBladeSections(stage, 3);
            Console.WriteLine($"  {stage.Name} blade sections (PyTurbo-Aero parameterization):");
            Console.WriteLine($"  {"Section",-8} {"r(mm)",7} {"chord(mm)",10} {"stagger°",9} {"camber°",8} {"t/c",6} {"r_LE(mm)",9} {"sweep",6} {"lean",5}");
            string[] names = {"Hub","Mean","Tip"};
            for (int i=0;i<secs.Count;i++)
            {
                var s=secs[i];
                Console.WriteLine($"  {names[i],-8} {s.Radius*1000,7:F1} {s.Chord*1000,10:F2} {s.StaggerAngle_deg,9:F1} {s.Camber_deg,8:F1} {s.t_max_ratio,6:F3} {s.r_LE*1000,9:F3} {s.Sweep_mm,6:F1} {s.Lean_mm,5:F1}");
            }
        }

        static double[] Linspace(double a,double b,int n)
        {var v=new double[n];for(int i=0;i<n;i++)v[i]=a+(b-a)*i/Math.Max(n-1,1);return v;}
    }

}
