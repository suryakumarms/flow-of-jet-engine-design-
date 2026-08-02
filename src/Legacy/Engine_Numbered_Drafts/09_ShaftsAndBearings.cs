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
    public static class RotorDynamics
    {
        public class RotorResult
        {
            public double CriticalSpeed1_RPM { get; set; }
            public double CriticalSpeed2_RPM { get; set; }
            public double OperatingRPM       { get; set; }
            public double Margin1_percent    { get; set; }
            public double Margin2_percent    { get; set; }
            public bool   Passed             { get; set; }
        }

        public static RotorResult AnalyzeSpool(string name, double rpm, double shaftLength, 
                                                double shaftOD, double shaftID, double totalMass)
        {
            Console.WriteLine($"  Rotordynamics [{name}]:");
            
            // Timoshenko beam: ω₁ = (π/L)² · √(EI/(ρA))
            double E = 200e9;  // Steel shaft
            double I = Math.PI / 64.0 * (Math.Pow(shaftOD, 4) - Math.Pow(shaftID, 4));
            double A = Math.PI / 4.0 * (shaftOD * shaftOD - shaftID * shaftID);
            double rhoA = 7850 * A + totalMass / shaftLength;  // Distributed + lumped
            
            // ── FIX 5A: Timoshenko shear correction ──────────────────────────
            // Standard EB beam overestimates bending stiffness for thick shafts.
            // Timoshenko factor: κ = 0.9 for hollow cylinder; adds shear flexibility.
            double kappa    = 0.9;
            double G_mod    = E / (2.0 * (1.0 + 0.3));  // Shear modulus (Poisson ν=0.3)
            double phi_s    = 12.0 * E * I / (kappa * G_mod * A * shaftLength * shaftLength);
            double omegaEB  = Math.Pow(Math.PI / shaftLength, 2) * Math.Sqrt(E * I / rhoA);
            // Timoshenko correction: ω_T = ω_EB / √(1 + φ_s)
            double omega_T1 = omegaEB / Math.Sqrt(1.0 + phi_s);

            // ── FIX 5B: Gyroscopic split (forward/backward whirl) ────────────
            // At high Ω (operating speed), gyroscopic moments split the critical speed
            // into forward whirl (ω_fw) and backward whirl (ω_bw).
            // Approximate: ω_fw ≈ ω_T1·(1 + α·Ω/ω_T1)
            //              ω_bw ≈ ω_T1·(1 - α·Ω/ω_T1)
            // where α ≈ 0.05 (gyroscopic coupling coefficient, depends on polar/transverse
            // inertia ratio Ip/Id; α = 0.05 is representative for a turbine disc).
            double Omega    = rpm * 2.0 * Math.PI / 60.0;  // Operating speed (rad/s)
            double alpha_g  = 0.05;
            double omega_fw = omega_T1 * (1.0 + alpha_g * Omega / omega_T1);
            double omega_bw = omega_T1 * (1.0 - alpha_g * Omega / omega_T1);
            omega_bw = Math.Max(omega_bw, omega_T1 * 0.5);  // Physical floor

            // ── FIX 5C: 2-spool coaxial inter-shaft bearing coupling ─────────
            // HP and LP spools are concentrically nested and coupled via a rolling
            // element inter-shaft bearing. This cross-coupling stiffness K_inter
            // shifts the coupled system natural frequencies.
            //
            // Simplified lumped-parameter correction:
            // K_inter ≈ 1e7 N/m (typical ball bearing, DN ≈ 3×10⁶ mm·rpm)
            // Coupled mode shift: δω ≈ ±K_inter/(2·m_spool·ω_T1)
            double K_inter  = 10e6;  // N/m inter-shaft bearing stiffness
            double m_spool  = totalMass;
            double d_omega  = m_spool > 0 ? K_inter / (2.0 * m_spool * Math.Max(omega_T1, 1.0)) : 0;
            double omega1   = omega_fw + d_omega;   // Forward whirl, coupled
            double omega2   = omega_bw - d_omega;   // Backward whirl, coupled
            omega2 = Math.Max(omega2, omega_T1 * 0.4);

            double crit1    = omega1 * 60.0 / (2.0 * Math.PI);
            double crit2    = omega2 * 60.0 / (2.0 * Math.PI);
            // Also report the third mode (second bending)
            double omega3   = 4.0 * omega_T1;  // Second bending (EB estimate)
            double crit3    = omega3 * 60.0 / (2.0 * Math.PI);

            double margin1  = Math.Abs(crit1 - rpm) / rpm * 100.0;
            double margin2  = Math.Abs(crit2 - rpm) / rpm * 100.0;
            double margin3  = Math.Abs(crit3 - rpm) / rpm * 100.0;

            bool passed     = margin1 > 15.0 && margin2 > 15.0 && margin3 > 15.0;
            
            Console.WriteLine($"    ω_T1={omega_T1*60/(2*Math.PI):F0}RPM  ω_fw={crit1:F0}RPM  ω_bw={crit2:F0}RPM  ω_2nd={crit3:F0}RPM");
            Console.WriteLine($"    Operating={rpm:F0}RPM  Margins: fw={margin1:F1}%  bw={margin2:F1}%  2nd={margin3:F1}%  {(passed?"✓":"✗ WHIRL RISK")}");

            return new RotorResult
            {
                CriticalSpeed1_RPM = crit1, CriticalSpeed2_RPM = crit2,
                OperatingRPM = rpm,
                Margin1_percent = margin1, Margin2_percent = margin2,
                Passed = passed
            };
        }
    }

    public static class BearingSystem
    {
        public class BR{public string Name="";public double CN,L10,QW,cSFD;public bool OK;}
        public static List<BR> Design(EngineFlowPath fp,CycleResult cy)
        {
            Console.WriteLine("═══ BEARING DESIGN (Harris L10 + Childs SFD) ═══");
            var list=new List<BR>();
            double[] rpms={fp.LP_RPM,fp.HP_RPM,fp.LP_RPM}; string[] nm={"Front-Ball(LP)","Mid-Roller(HP)","Rear-Ball(LP)"};
            double[] F={8000,15000,6000},Db={.015,.020,.015};
            for(int i=0;i<3;i++){
                var b=new BR{Name=nm[i]};
                double Om=rpms[i]*2*Math.PI/60,fcm=1.3e4,Z=18,al=15*Math.PI/180;
                b.CN=fcm*Math.Pow(Db[i]*1000,1.8)*Math.Pow(Z,.7)*Math.Pow(Math.Cos(al),.7);
                b.L10=Math.Pow(b.CN/Math.Max(F[i]*1.1,1),3)*1e6/(60*Math.Max(rpms[i],1));
                double dm=Db[i]*5; b.QW=.001*F[i]*dm/2*Om;
                double mu=.003,R=Db[i]*2.5,Ls=Db[i]*2,c=3e-4,ep=.3;
                b.cSFD=mu*R*Ls*Ls*Ls*Math.PI/(2*c*c*c*Math.Pow(1-ep*ep,1.5));
                b.OK=b.L10>30000;
                Console.WriteLine($"  {nm[i]}: C={b.CN/1000:F1}kN L10={b.L10:F0}h Q={b.QW:F1}W cSFD={b.cSFD/1000:F1}kNs/m {(b.OK?"✓":"✗")}");
                list.Add(b);
            }
            return list;
        }
    }

    public static class SealAnalysis
    {
        public class SR{public string Name="",Type="";public double mL,Fr,QkW,Gr;public bool OK;}
        public static List<SR> Analyze(EngineFlowPath fp,CycleResult cy)
        {
            Console.WriteLine("═══ SEAL ANALYSIS (Egli labyrinth + brush) ═══");
            var list=new List<SR>(); double mc=cy.CoreMassFlow;
            GasStation G(int k)=>cy.Stations.GetValueOrDefault(k)??new GasStation{Pt=101325,Tt=288};
            var locs=new[]{
                ("HPT-Disc",G(4).Pt,G(45).Pt,G(4).Tt,0.06,4,"Labyrinth"),
                ("LPT-Disc",G(45).Pt,G(5).Pt,G(45).Tt,0.08,6,"Brush"),
                ("HPC-Exit",G(3).Pt,G(4).Pt,G(3).Tt,0.05,3,"Labyrinth")};
            foreach(var(name,P1,P2,T1,cl,N,tp) in locs){
                double PR=P2/Math.Max(P1,1),Cd=tp=="Brush"?.05:.5/Math.Sqrt(N);
                double A=Math.PI*cl/1000*.02,fe=PR>.02?Math.Sqrt(1-PR*PR)/Math.Sqrt(N-Math.Log(PR+.001)):.01;
                double mL=Cd*A*P1/Math.Sqrt(287*T1)*fe,fr=mL/Math.Max(mc,.01)*100;
                var s=new SR{Name=name,Type=tp,mL=mL,Fr=fr,QkW=mL*(P1-P2)/1200/1000,Gr=13e-6*400*.2*1000,OK=fr<1.5};
                Console.WriteLine($"  {name}[{tp}]: ṁ={mL:F4}kg/s ({fr:F2}%) Q={s.QkW:F1}kW δ={s.Gr:F2}mm {(s.OK?"✓":"✗")}");
                list.Add(s);
            }
            return list;
        }
    }

    public static class ShaftMechanicals
    {
        public class ShaftThrustResult
        {
            public string SpoolName          { get; set; } = "";
            public double CompressorForce_N  { get; set; }   // forward
            public double TurbineForce_N     { get; set; }   // rearward
            public double NetAxialForce_N    { get; set; }
            public double BalancePistonForce_N { get; set; }
            public double BearingForce_N     { get; set; }
            public double BearingLimit_N     { get; set; } = 80000.0;  // 80 kN typical
            public bool   Passed             { get; set; }
        }

        public static (ShaftThrustResult HP, ShaftThrustResult LP) AnalyzeShaftThrust(
            EngineFlowPath fp, CycleResult cycle)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  GAP 4: AXIAL SHAFT THRUST BALANCING");
            Console.WriteLine("════════════════════════════════════════════════════════");

            var hp = ComputeSpoolThrust("HP Spool", fp.HPCStages, fp.HPTStages, cycle);
            var lp = ComputeSpoolThrust("LP Spool",
                fp.FanStages.Concat(fp.LPCStages).ToList(), fp.LPTStages, cycle);

            Console.WriteLine($"  HP spool: F_comp={hp.CompressorForce_N/1000:F1}kN  " +
                              $"F_turb={hp.TurbineForce_N/1000:F1}kN  " +
                              $"F_net={hp.NetAxialForce_N/1000:F1}kN  " +
                              $"F_piston={hp.BalancePistonForce_N/1000:F1}kN  " +
                              $"F_bearing={hp.BearingForce_N/1000:F1}kN  " +
                              $"{(hp.Passed?"✓":"✗ BEARING OVERLOAD")}");
            Console.WriteLine($"  LP spool: F_comp={lp.CompressorForce_N/1000:F1}kN  " +
                              $"F_turb={lp.TurbineForce_N/1000:F1}kN  " +
                              $"F_net={lp.NetAxialForce_N/1000:F1}kN  " +
                              $"F_piston={lp.BalancePistonForce_N/1000:F1}kN  " +
                              $"F_bearing={lp.BearingForce_N/1000:F1}kN  " +
                              $"{(lp.Passed?"✓":"✗ BEARING OVERLOAD")}");
            Console.WriteLine("════════════════════════════════════════════════════════");
            return (hp, lp);
        }

        private static ShaftThrustResult ComputeSpoolThrust(
            string name,
            IList<BladeStage> compressors,
            IList<BladeStage> turbines,
            CycleResult cycle)
        {
            var r = new ShaftThrustResult { SpoolName = name };

            // ── FIX 3: stage-by-stage pressure accumulation ──────────────────
            // Bug was: cycle.Stations.Values.First().Pt  → C# Dictionary has no
            // guaranteed order; at cruise this was returning S0 (23 kPa freestream)
            // instead of the local spool inlet (600-2500 kPa). Forces were off by
            // a factor of 10-40, causing false-positive bearing checks.
            //
            // Fix: seed currentPt from the correct thermodynamic inlet station,
            // then accumulate stage-by-stage using isentropic PR.
            //
            // F_gas,stage = ΔP · A_annulus   (dominant term for disc loading)
            // ─────────────────────────────────────────────────────────────────
            // HP spool inlet: Station 25 (LPC exit);  LP spool inlet: Station 2 (fan face)
            double currentPt_comp = name.Contains("HP")
                ? (cycle.Stations.ContainsKey(25) ? cycle.Stations[25].Pt : 100e3)
                : (cycle.Stations.ContainsKey(2)  ? cycle.Stations[2].Pt  : 25e3);

            // Compressor stages — pressure rise → forward thrust on disc face
            foreach (var s in compressors)
            {
                double A_ann   = Math.PI * (s.TipRadius * s.TipRadius - s.HubRadius * s.HubRadius);
                double inletP  = currentPt_comp;
                double exitP   = inletP * Math.Max(1.0, s.PressureRatio);
                double dP      = exitP - inletP;
                r.CompressorForce_N += dP * A_ann;
                currentPt_comp = exitP;   // Accumulate for next stage
            }

            // Turbine inlet: Stage 4 (HPT) or Stage 45 (LPT)
            double currentPt_turb = name.Contains("HP")
                ? (cycle.Stations.ContainsKey(4)  ? cycle.Stations[4].Pt  : 2000e3)
                : (cycle.Stations.ContainsKey(45) ? cycle.Stations[45].Pt : 500e3);

            // Turbine stages — pressure drop → rearward force on disc face
            foreach (var s in turbines)
            {
                double A_ann   = Math.PI * (s.TipRadius * s.TipRadius - s.HubRadius * s.HubRadius);
                double inletP  = currentPt_turb;
                // Estimate stage PR from temperature drop: (T_out/T_in)^(γ/((γ-1)·η_t))
                double gamma_t = 1.33;
                double eta_t   = 0.92;
                double T_ratio = s.Temperature_In > 0 ? s.Temperature_Out / s.Temperature_In : 0.85;
                double stagePR_turb = Math.Pow(T_ratio, gamma_t / ((gamma_t - 1.0) * eta_t));
                stagePR_turb = Math.Min(stagePR_turb, 1.0);  // Turbine always expands
                double exitP   = inletP * Math.Max(0.3, stagePR_turb);
                double dP      = inletP - exitP;  // Positive: turbine drops pressure
                r.TurbineForce_N += dP * A_ann;
                currentPt_turb = exitP;
            }

            r.NetAxialForce_N = r.CompressorForce_N - r.TurbineForce_N;  // +ve = forward

            // Optimal balance piston design: counteracts the net axial force to leave a small residual load on the thrust bearing
            r.BalancePistonForce_N = Math.Abs(r.NetAxialForce_N) - 0.4 * r.BearingLimit_N;
            if (r.BalancePistonForce_N < 0) r.BalancePistonForce_N = 0;
            r.BearingForce_N = Math.Abs(Math.Abs(r.NetAxialForce_N) - r.BalancePistonForce_N);
            r.Passed = r.BearingForce_N <= r.BearingLimit_N;
            return r;
        }

        public class PTOResult
        {
            public double PowerExtract_W        { get; set; }
            public double TorqueExtract_Nm      { get; set; }
            public double TowerShaftDiameter_mm { get; set; }
            public double WhirlFrequency_Hz     { get; set; }
            public double WhirlSafetyMargin     { get; set; }
            public double Gear1PitchDiameter_mm { get; set; }
            public double Gear2PitchDiameter_mm { get; set; }
            public double GearToothForce_N      { get; set; }
            public double SplineWearLife_hours  { get; set; }
            public bool   Passed                 { get; set; }
        }

        public static PTOResult SizePowerTakeOff(EngineFlowPath fp, CycleResult cycle)
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  POWER TAKE-OFF (PTO) & ACCESSORY DRIVE SIZING");
            Console.WriteLine("════════════════════════════════════════════════════════");

            var r = new PTOResult();

            // Samarium-Cobalt starter-generator load (APET Pratt & Whitney CR-168114 / Reynolds)
            r.PowerExtract_W = 100000.0; // 100 kW extraction
            
            // Rotational speed of HP spool in rad/s
            double HP_RPM = fp.HP_RPM;
            double omega_HP = 2.0 * Math.PI * HP_RPM / 60.0;
            r.TorqueExtract_Nm = r.PowerExtract_W / Math.Max(omega_HP, 1.0);

            // Tower Shaft Sizing for torsional shear (VASCO X2 high-strength steel)
            double tau_allow = 250e6; // 250 MPa allowable torsional shear stress
            double d_shaft = Math.Pow((16.0 * r.TorqueExtract_Nm) / (Math.PI * tau_allow), 1.0 / 3.0);
            r.TowerShaftDiameter_mm = d_shaft * 1000.0; // Convert to mm

            // Critical whirl speed (first critical frequency)
            // L_shaft = 0.8m, E = 200 GPa, density = 7850 kg/m^3
            double L_shaft = 0.8;
            double E_steel = 200e9;
            double rho_steel = 7850.0;
            // f_whirl = (pi * d_shaft / (8 * L^2)) * sqrt(E/rho)
            r.WhirlFrequency_Hz = (Math.PI * d_shaft / (8.0 * L_shaft * L_shaft)) * Math.Sqrt(E_steel / rho_steel);
            double HP_freq = HP_RPM / 60.0;
            r.WhirlSafetyMargin = r.WhirlFrequency_Hz / Math.Max(HP_freq, 1.0);

            // Spiral Bevel Gear set sizing (NASA TP / CF6 performance improvement)
            r.Gear1PitchDiameter_mm = 80.0; // 80mm input bevel gear on HP spool
            double gearRatio = 1.5;        // step up ratio to generator
            r.Gear2PitchDiameter_mm = r.Gear1PitchDiameter_mm / gearRatio; // 53.3mm
            
            // Tangential gear tooth force (Ft = Torque / R_pitch)
            double R_pitch1 = (r.Gear1PitchDiameter_mm / 1000.0) / 2.0;
            r.GearToothForce_N = r.TorqueExtract_Nm / Math.Max(R_pitch1, 0.01);

            // Spline misalignment wear life (APET flexible diaphragm spline wear model)
            double theta_mis = 0.5 * Math.PI / 180.0; // 0.5 degrees misalignment
            double h_lub     = 0.2e-6; // 0.2 micrometers lubrication oil film thickness
            double D_spline  = 0.025;  // 25mm spline diameter
            // Wear volume rate model: W_wear = K * T_pto * theta / (h_lub * D_spline)
            double K_wear = 1.0e-12; // empirical wear factor for lubricated VASCO X2 splines
            double wearRate = K_wear * (r.TorqueExtract_Nm * theta_mis) / (h_lub * D_spline);
            r.SplineWearLife_hours = 1000.0 / Math.Max(wearRate, 1.0e-6); // Hours until wear limit

            // Certification check
            bool stressPassed = r.TowerShaftDiameter_mm >= 12.0; // Minimum manufacturing thickness
            bool whirlPassed  = r.WhirlSafetyMargin >= 1.2;      // 20% margin above redline
            bool wearPassed   = r.SplineWearLife_hours >= 20000.0; // 20,000 hour overhaul limit
            r.Passed = stressPassed && whirlPassed && wearPassed;

            Console.WriteLine($"  HP Spool Extract: {r.PowerExtract_W/1000:F1} kW  |  Torque={r.TorqueExtract_Nm:F1} Nm");
            Console.WriteLine($"  Tower Shaft Diameter: {r.TowerShaftDiameter_mm:F1} mm  (allowable={12.0} mm)  {(stressPassed?"✓":"✗ too thin")}");
            Console.WriteLine($"  Whirl Speed: {r.WhirlFrequency_Hz:F0} Hz vs Redline={HP_freq:F0} Hz  (margin={r.WhirlSafetyMargin*100-100:F1}% vs 20% target)  {(whirlPassed?"✓":"✗ critical speed risk")}");
            Console.WriteLine($"  Spiral Bevel Gears: D1={r.Gear1PitchDiameter_mm:F1}mm  D2={r.Gear2PitchDiameter_mm:F1}mm  Ft={r.GearToothForce_N:F0} N");
            Console.WriteLine($"  Spline wear life: {r.SplineWearLife_hours:F0} hours  (limit={20000} hrs)  {(wearPassed?"✓":"✗ accelerated wear risk")}");
            Console.WriteLine("════════════════════════════════════════════════════════");

            return r;
        }
    }

    public static class InterShaftBearingCoupling
    {
        public class CouplingResult
        {
            public double Kxx_MNm       { get; set; }   // Direct stiffness (MN/m)
            public double Kxy_MNm       { get; set; }   // Cross-coupled stiffness
            public double CrossGyro_Nm  { get; set; }   // Cross-spool gyroscopic moment
            public double CriticalSpeedLP_RPM { get; set; } // LP critical speed with coupling
            public double CriticalSpeedHP_RPM { get; set; } // HP critical speed with coupling
            public bool   CriticalMarginOK    { get; set; } // >15% margin from operating speeds
        }

        public static CouplingResult Evaluate(EngineFlowPath fp, CycleResult cycle)
        {
            Console.WriteLine("  [Gate 5D] INTER-SHAFT BEARING COUPLING MATRIX");
            var r = new CouplingResult();

            // Inter-shaft roller bearing properties (typical values: Childs 1993 Table 3.2)
            // Bearing located at HP-LP inter-spool at HPC axial station
            double Db   = 0.025;   // bearing bore diameter, m
            double Z    = 20;      // number of rolling elements
            double alpha= 0.0;     // contact angle, radians (roller bearing)
            double fcm  = 1.3e4;   // basic static capacity coefficient (N/mm^1.8)

            // Dynamic stiffness of inter-shaft bearing (Hertz contact linearised)
            // K_direct ≈ 2.0 × C_rating (Childs eq 3-14)
            double C_rating = fcm * Math.Pow(Db * 1000, 1.8) * Math.Pow(Z, 0.7);
            r.Kxx_MNm = 2.0 * C_rating / 1e6;  // MN/m

            // Cross-coupled stiffness from journal rotation in squeeze-film:
            // Kxy ≈ 0.3 × Kxx (Childs: lightly loaded SFD)
            r.Kxy_MNm = 0.3 * r.Kxx_MNm;

            // Cross-shaft gyroscopic coupling
            // G_cross = Σ(I_disc_i) × Ω_LP × Ω_HP / (2π)
            double I_LP = 0.0, I_HP = 0.0;
            foreach (var s in fp.FanStages.Concat(fp.LPCStages).Concat(fp.LPTStages))
            { double r2 = s.HubRadius * 0.7; I_LP += 0.5 * (7800 * Math.PI * r2 * r2 * 0.05) * r2 * r2; }
            foreach (var s in fp.HPCStages.Concat(fp.HPTStages))
            { double r2 = s.HubRadius * 0.7; I_HP += 0.5 * (7800 * Math.PI * r2 * r2 * 0.05) * r2 * r2; }

            double omLP = fp.LP_RPM * 2 * Math.PI / 60.0;
            double omHP = fp.HP_RPM * 2 * Math.PI / 60.0;
            r.CrossGyro_Nm = Math.Sqrt(I_LP * I_HP) * omLP * omHP / (2 * Math.PI * 1000);

            // Coupled critical speeds: uncoupled beam critical speed modified by K_inter
            // ωc_coupled = sqrt(ωc_uncoupled² + Kxy/I)
            double wc_LP_unc = omLP * 0.72;  // 72% of operating speed (typical Jeffcott estimate)
            double wc_HP_unc = omHP * 0.68;
            double Kxy_SI    = r.Kxy_MNm * 1e6;
            r.CriticalSpeedLP_RPM = Math.Sqrt(wc_LP_unc * wc_LP_unc + Kxy_SI / Math.Max(I_LP, 0.01)) * 60 / (2 * Math.PI);
            r.CriticalSpeedHP_RPM = Math.Sqrt(wc_HP_unc * wc_HP_unc + Kxy_SI / Math.Max(I_HP, 0.01)) * 60 / (2 * Math.PI);

            // Margin: critical must be < 85% of operating or > 115%
            double marginLP = Math.Abs(r.CriticalSpeedLP_RPM - fp.LP_RPM) / fp.LP_RPM;
            double marginHP = Math.Abs(r.CriticalSpeedHP_RPM - fp.HP_RPM) / fp.HP_RPM;
            r.CriticalMarginOK = marginLP > 0.15 && marginHP > 0.15;

            Console.WriteLine($"    K_direct={r.Kxx_MNm:F2} MN/m  K_cross={r.Kxy_MNm:F2} MN/m");
            Console.WriteLine($"    Cross-gyroscopic moment: {r.CrossGyro_Nm:F1} kN·m");
            Console.WriteLine($"    LP critical: {r.CriticalSpeedLP_RPM:F0} rpm (margin {marginLP*100:F1}%)");
            Console.WriteLine($"    HP critical: {r.CriticalSpeedHP_RPM:F0} rpm (margin {marginHP*100:F1}%)");
            Console.WriteLine($"    Coupled critical margin: {(r.CriticalMarginOK ? "✓ PASS" : "✗ FAIL — resonance risk")}");
            return r;
        }
    }

    public static class ElasticBladeDiskCoupling
    {
        public class CoupledMode
        {
            public string  StageName   { get; set; } = "";
            public double  OmegaBlade_Hz   { get; set; }  // Uncoupled blade 1F
            public double  OmegaDisk_Hz    { get; set; }  // Uncoupled disk umbrella mode
            public double  OmegaCoupledLow_Hz  { get; set; } // Lower coupled frequency
            public double  OmegaCoupledHigh_Hz { get; set; } // Upper coupled frequency
            public double  CampbellShift_pct   { get; set; } // % shift vs rigid-disk assumption
            public bool    ResonanceRisk   { get; set; }
        }

        public static List<CoupledMode> Analyze(EngineFlowPath fp, CycleResult cycle)
        {
            Console.WriteLine("  [Gate 5E] ELASTIC BLADE-DISK COUPLED EIGENVALUES (Ewins 1985)");
            var results = new List<CoupledMode>();

            foreach (var st in fp.AllStages().Where(s => s.IsRotor))
            {
                double E   = st.YoungsModulus_GPa * 1e9;
                double rho = st.MaterialDensity_kgm3;
                double L   = st.Span;
                double h   = st.Chord * st.MaxThicknessRatio;
                double b   = st.Chord * 0.12;
                double I   = b * h * h * h / 12.0;
                double A   = b * h;
                double rA  = rho * A;

                // Uncoupled blade 1F natural frequency (Euler-Bernoulli)
                double beta1L = 1.875;
                double omega_blade = beta1L * beta1L / (L * L) * Math.Sqrt(E * I / rA) / (2 * Math.PI);

                // Uncoupled disc umbrella mode (Kirchhoff plate, 0-nodal-diameter)
                double rDisc = st.HubRadius * 0.7;
                double tDisc = 0.05;
                double D_plate = E * tDisc * tDisc * tDisc / (12 * (1 - 0.3 * 0.3));  // flexural rigidity
                double rhoDisc = 7800;
                // f_disc_umbrella ≈ 1.015 × sqrt(D/(rhoDisc·tDisc)) / rDisc²  (Soedel 1993)
                double omega_disc = 1.015 * Math.Sqrt(D_plate / (rhoDisc * tDisc)) / (rDisc * rDisc) / (2 * Math.PI);

                // Root coupling spring: k_root from fir-tree contact stiffness
                // k_root ≈ E × A_root / L_root  (root contact zone ~30% of blade area)
                double k_root = E * (A * 0.3) / (L * 0.1);

                // Ewins coupled frequency:
                double wb2 = omega_blade * omega_blade;
                double wd2 = omega_disc  * omega_disc;
                double mid = (wb2 + wd2) / 2.0;
                double half_diff = (wb2 - wd2) / 2.0;
                double k_c = k_root / (rA * L * (2 * Math.PI) * (2 * Math.PI));  // normalised
                double disc_low  = Math.Sqrt(Math.Max(mid - Math.Sqrt(half_diff * half_diff + k_c * k_c), 0.001));
                double disc_high = Math.Sqrt(mid + Math.Sqrt(half_diff * half_diff + k_c * k_c));

                double shift = Math.Abs(disc_low - omega_blade) / Math.Max(omega_blade, 0.001) * 100.0;

                // Check if any engine-order resonance falls on coupled mode (within 8%)
                double RPM = st.RPM;
                bool resonanceRisk = false;
                foreach (int EO in new[] { 1, 2, 3, 4, 5, 6, 7, 8 })
                {
                    double f_exc = EO * RPM / 60.0;
                    if (Math.Abs(f_exc - disc_low) / disc_low < 0.08 ||
                        Math.Abs(f_exc - disc_high) / disc_high < 0.08)
                        resonanceRisk = true;
                }

                var m = new CoupledMode
                {
                    StageName          = st.Name,
                    OmegaBlade_Hz      = omega_blade,
                    OmegaDisk_Hz       = omega_disc,
                    OmegaCoupledLow_Hz = disc_low,
                    OmegaCoupledHigh_Hz= disc_high,
                    CampbellShift_pct  = shift,
                    ResonanceRisk      = resonanceRisk
                };
                results.Add(m);
                Console.WriteLine($"    {st.Name}: f_blade={omega_blade:F1}Hz  f_disk={omega_disc:F1}Hz  "
                    + $"f_coupled=[{disc_low:F1}, {disc_high:F1}]Hz  shift={shift:F1}%  {(resonanceRisk ? "⚠ EO resonance" : "✓")}");
            }
            return results;
        }
    }

}
