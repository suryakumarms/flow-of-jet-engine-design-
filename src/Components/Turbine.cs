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
    public static class TurbineCooling
    {
        public class CR{public double FilmEta,Hint,Nuimp,OvEta,Twall,Cfrac;public bool TBC;}
        public static CR Analyze(double Tg,double Tc,double Mc=.45,double chord=.05,double pitch=.02,double wk=14,double wt=.003)
        {
            Console.WriteLine("═══ HPT COOLING (Baldauf/Dittus-Boelter/Martin) ═══");
            var r=new CR();
            r.FilmEta=Math.Clamp(1.0/(1+.329*Math.Pow(.71,.4)/Math.Max(Mc,.01)*Math.Pow(15,.8)),.2,.85);
            double Dh=.002,Vc=50,rh=5,mu=3.5e-5,Pr=.73,k=mu*1150/Pr,Re=rh*Vc*Dh/mu;
            r.Hint=.023*Math.Pow(Re,.8)*Math.Pow(Pr,.4)*k/Dh;
            r.Nuimp=.5*Math.Pow(rh*Vc*.001/mu,.7)*Math.Pow(Pr,.42)*Math.Pow(3,-.6);
            double Bi=3500*wt/wk;
            r.OvEta=Math.Clamp(1.0/(1.0/r.FilmEta+Bi),.1,.9);
            r.Twall=Tg-r.OvEta*(Tg-Tc);
            r.Cfrac=Math.Clamp(3500*(Tg-r.Twall)*chord*pitch/(1005*Math.Max(r.Twall-Tc,1)*50),.01,.15);
            r.TBC=r.Twall>1100;
            Console.WriteLine($"  η_f={r.FilmEta:F3} h_int={r.Hint:F0}W/m²K T_wall={r.Twall:F0}K TBC={r.TBC}");
            return r;
        }
    }

    public static class KackerOkapuuLoss
    {
        public class KOResult
        {
            public double Y_profile, Y_secondary, Y_tip, Y_trailing, Y_shock, Y_total;
            public double Eta_tt;       // total-to-total stage efficiency
            public double Re_correction;
        }

        public static KOResult Evaluate(
            double alpha1_deg, double alpha2_deg, double alpha3_deg,
            double beta1_deg, double beta3_deg,
            double h, double c, double s, double tc,
            double M2, double M3rel, double tcl, double r_hub, double r_tip,
            double Re, double gamma = 1.33)
        {
            var r = new KOResult();
            double a1=alpha1_deg*Math.PI/180, a2=alpha2_deg*Math.PI/180, a3=alpha3_deg*Math.PI/180;
            double b1=beta1_deg*Math.PI/180, b3=beta3_deg*Math.PI/180;
            double am = Math.Atan((Math.Tan(a2)+Math.Tan(a3))/2);  // mean flow angle

            double CL = 2*(s/c)*(Math.Tan(a2)+Math.Tan(a3))*Math.Cos(am);
            double t_over_c = tc;
            double beta_ratio = Math.Abs(beta1_deg/Math.Max(alpha2_deg,1.0));
            double Yp0 = 0.914*(0.023 + 0.58*t_over_c);   // nozzle (β1=0)
            double Yp1 = 0.914*(0.008 + 0.66*t_over_c);   // impulse (β1=α2)
            double YpAMDC = (Yp0 + beta_ratio*beta_ratio*(Yp1-Yp0))
                           * Math.Pow(t_over_c/0.2, beta_ratio);

            double Ma_hub = M2 * (1 + 0.1*(r_tip/Math.Max(r_hub,0.01)-1));  // hub Ma higher
            double f_hub = Math.Max(0, Ma_hub - 0.4);
            double P0_ratio_approx = Math.Pow((1+0.5*(gamma-1)*M3rel*M3rel)/(1+0.5*(gamma-1)*M2*M2), gamma/(gamma-1));
            r.Y_shock = f_hub > 0
                ? 0.75*Math.Pow(f_hub,1.75)*(r_hub/Math.Max(r_tip,0.01))*P0_ratio_approx
                : 0.0;

            double Kp = M2 < 0.2 ? 1.0 : Math.Max(0.1, 1.0 - 0.25*Math.Sqrt(M2-0.2));

            r.Y_profile = 0.914*(2.0/3.0*YpAMDC*Kp + r.Y_shock);
            r.Y_profile = Math.Clamp(r.Y_profile, 0.005, 0.25);

            double hoc = h/Math.Max(c,1e-6);
            double fAR = hoc <= 2.0
                ? (1.0 - 0.25*Math.Sqrt(Math.Max(0,2-hoc)))/Math.Max(hoc,0.01)
                : 1.0/Math.Max(hoc,0.01);
            double YsADMC = 0.0334*fAR*Math.Pow(CL/(s/c),2)
                           *(Math.Cos(a2)/Math.Max(Math.Cos(b1),0.01))
                           *(Math.Cos(a2)*Math.Cos(a2)/Math.Max(Math.Pow(Math.Cos(am),3),0.01));
            double Ks = Math.Max(0.1, 1.0 - 0.15*M2);  // compressibility factor
            r.Y_secondary = 1.2*YsADMC*Ks;
            r.Y_secondary = Math.Clamp(r.Y_secondary, 0.003, 0.30);

            double r_mean = (r_hub+r_tip)/2;
            double dEta0 = 0.93*CL*Math.Cos(a2)/(s/Math.Max(c,1e-6));  // efficiency at zero clearance ≈ f(CL,α2)
            r.Y_tip = dEta0*0.93*(tcl/Math.Max(h,0.001))*Math.Cos(a2)*(r_tip/Math.Max(r_mean,0.001));
            r.Y_tip = Math.Clamp(r.Y_tip, 0, 0.12);

            double t_te_over_s = 0.02;
            double dPhi2 = t_te_over_s * (1.0 - 0.5*beta_ratio);
            r.Y_trailing = 1.0/(1.0 - dPhi2) - 1.0;
            r.Y_trailing = Math.Clamp(r.Y_trailing, 0, 0.05);

            r.Re_correction = Re <= 2e5 ? Math.Pow(Re/2e5,-0.4)
                            : Re < 1e6  ? 1.0
                            : Math.Pow(Re/1e6,-0.2);
            r.Y_profile *= r.Re_correction;

            r.Y_total = r.Y_profile + r.Y_secondary + r.Y_tip + r.Y_trailing;

            double Tt_corr = 1.0 + 0.5*(gamma-1)*M3rel*M3rel;
            r.Eta_tt = Math.Max(0.60, 1.0 - r.Y_total*Tt_corr);
            return r;
        }

        public static void EvaluateTurbineStages(EngineFlowPath fp, CycleResult cycle)
        {
            Console.WriteLine("═══ KACKER-OKAPUU LOSS MODEL (1982 ASME J. Eng. Power) ═══");
            foreach (var st in fp.HPTStages.Concat(fp.LPTStages))
            {
                double M2_rot   = 0.30 + 0.15*(st.Temperature_In>1200?1:0);  // HPT M2 higher
                double M3r_rot  = 0.50 + 0.10*(st.IsRotor?1:0);
                double Re   = 5e5;
                double tcl  = 0.0005;  // 0.5mm tip clearance

                var ko_stat = Evaluate(
                    0, 60, 0, 0, 0,
                    st.Span, st.Chord, st.Chord*st.Solidity/Math.Max(st.BladeCount,1)*2*Math.PI*st.MeanRadius/Math.Max(st.BladeCount,1),
                    st.MaxThicknessRatio, 0.25, 0.65, 0.0, st.HubRadius, st.TipRadius, Re);

                var ko_rot = Evaluate(
                    20, 60, -30, 50, -55,
                    st.Span, st.Chord, st.Chord*st.Solidity/Math.Max(st.BladeCount,1)*2*Math.PI*st.MeanRadius/Math.Max(st.BladeCount,1),
                    st.MaxThicknessRatio, M2_rot, M3r_rot, tcl, st.HubRadius, st.TipRadius, Re);

                double eta_stage = 0.5 * (ko_stat.Eta_tt + ko_rot.Eta_tt);

                Console.WriteLine($"  {st.Name} (Stator NGV): Y_p={ko_stat.Y_profile:F4} Y_s={ko_stat.Y_secondary:F4} Y_te={ko_stat.Y_trailing:F4} → Y_tot={ko_stat.Y_total:F4}  η_tt={ko_stat.Eta_tt*100:F2}%");
                Console.WriteLine($"  {st.Name} (Rotor):      Y_p={ko_rot.Y_profile:F4} Y_s={ko_rot.Y_secondary:F4} Y_cl={ko_rot.Y_tip:F4} Y_te={ko_rot.Y_trailing:F4} → Y_tot={ko_rot.Y_total:F4}  η_tt={ko_rot.Eta_tt*100:F2}%");
                Console.WriteLine($"  {st.Name} Combined Stage Mean: η_tt={eta_stage*100:F2}%");
            }
            Console.WriteLine("════════════════════════════════════════════════════════");
        }
    }

    public static class Turbine
    {
        public static void BuildTurbine(EngineFlowPath fp, float sc, BBox3 domain, string outDir, float zHPT, float zLPT, float coreR)
        {
            // ════════════════════════════════════════
            //  4. HPT ASSEMBLY (SEPARATE BLADES & DISCS)
            // ════════════════════════════════════════
            Library.Log("HPT: separate blades with cooling & slotted discs...");
            var vHPTBlades = new Voxels();
            var vHPTDisks = new Voxels();
            float zPos = zHPT;
            foreach (var stage in fp.HPTStages)
            {
                float hR = (float)(stage.HubRadius * sc);
                float tR = (float)(stage.TipRadius * sc);
                float ch = Math.Max((float)(stage.Chord * sc), 12.0f);
                float th = Math.Max(ch * (float)stage.MaxThicknessRatio, 6.0f);

                // Twisted blades with internal cooling cavity, platform, and squealer
                var solid = new Voxels(new SdfTwistedBladeRow(hR, tR, ch, th, (float)stage.StaggerAngle * 0.85f, (float)stage.StaggerAngle * 1.15f, zPos, stage.BladeCount), domain);
                // T1-4: serpentine cooling channels + film holes (replaces simple hollow cavity)
                solid.BoolSubtract(new Voxels(new SdfSerpentineCooling(hR, tR, ch, th, zPos, stage.BladeCount), domain));
                // T1-7: balancing boss pads on disc front face
                var vBossHPT = new Voxels(new SdfBalancingBoss(hR * 0.85f, zPos - ch * 0.4f, 2.5f, 6f, 24), domain);
                vHPTDisks.BoolAdd(vBossHPT);
                solid.BoolAdd(new Voxels(new SdfDisk(hR * 0.85f, hR * 1.06f, zPos - ch*0.3f, ch*0.2f), domain));
                solid.BoolSubtract(new Voxels(new SdfDisk(tR * 0.97f, tR, zPos + ch*0.3f, ch*0.12f), domain));
                
                // Add tenons
                var tenons = new Voxels(new SdfFirTreeRow(hR, zPos, ch, ch * 0.25f, 3.0f, 3, 8f, stage.BladeCount), domain);
                solid.BoolAdd(tenons);
                vHPTBlades.BoolAdd(solid);

                // Disc ring with slots
                var disk = new Voxels(new SdfDisk(hR * 0.65f, hR, zPos, ch * 0.6f), domain);
                var slots = new Voxels(new SdfFirTreeRow(hR + 0.5f, zPos, ch + 2.0f, ch * 0.25f + 0.5f, 3.2f, 3, 8f, stage.BladeCount), domain);
                disk.BoolSubtract(slots);
                vHPTDisks.BoolAdd(disk);

                zPos += ch * 2f;
            }
            JetEngineFabrication.SaveSTL(vHPTBlades, outDir, "Jet_HPT_Blades.stl");
            Library.oViewer().Add(vHPTBlades, 4);
            Library.oViewer().SetGroupMaterial(4, new ColorFloat(1.0f, 0.7f, 0.3f), 0.85f, 0.05f);

            JetEngineFabrication.SaveSTL(vHPTDisks, outDir, "Jet_HPT_Disks.stl");
            Library.oViewer().Add(vHPTDisks, 15);
            Library.oViewer().SetGroupMaterial(15, new ColorFloat(0.5f, 0.5f, 0.55f), 0.7f, 0.05f);

            // ════════════════════════════════════════
            //  5. LPT ASSEMBLY (SEPARATE BLADES & DISCS)
            // ════════════════════════════════════════
            Library.Log("Generating separate LPT blades and discs...");
            var vLPTBlades = new Voxels();
            var vLPTDisks = new Voxels();
            zPos = zLPT;
            foreach (var stage in fp.LPTStages)
            {
                float hR = (float)(stage.HubRadius * sc);
                float tR = (float)(stage.TipRadius * sc);
                float ch = (float)(stage.Chord * sc);
                float th = ch * (float)stage.MaxThicknessRatio;

                var blades = new Voxels(new SdfBladeRow(hR, tR, ch, th, (float)stage.StaggerAngle, zPos, stage.BladeCount), domain);
                var tenons = new Voxels(new SdfFirTreeRow(hR, zPos, ch, ch * 0.22f, 2.5f, 3, 7f, stage.BladeCount), domain);
                blades.BoolAdd(tenons);
                vLPTBlades.BoolAdd(blades);

                var disk = new Voxels(new SdfDisk(hR * 0.65f, hR, zPos, ch * 0.5f), domain);
                var slots = new Voxels(new SdfFirTreeRow(hR + 0.5f, zPos, ch + 2.0f, ch * 0.22f + 0.4f, 2.7f, 3, 7f, stage.BladeCount), domain);
                disk.BoolSubtract(slots);
                vLPTDisks.BoolAdd(disk);

                zPos += ch * 1.8f;
            }
            JetEngineFabrication.SaveSTL(vLPTBlades, outDir, "Jet_LPT_Blades.stl");
            Library.oViewer().Add(vLPTBlades, 5);
            Library.oViewer().SetGroupMaterial(5, new ColorFloat(0.8f, 0.6f, 0.3f), 0.7f, 0.1f);

            JetEngineFabrication.SaveSTL(vLPTDisks, outDir, "Jet_LPT_Disks.stl");
            Library.oViewer().Add(vLPTDisks, 16);
            Library.oViewer().SetGroupMaterial(16, new ColorFloat(0.55f, 0.55f, 0.6f), 0.7f, 0.05f);

            // ════ NEW: TURBINE STATORS (NGVs) (interleaved) ════
            Library.Log("Generating turbine stator guide vanes (NGVs)...");
            var vTurbineStators = new Voxels();
            if (fp.HPTStages.Count > 0)
            {
                var hptSt = fp.HPTStages[0];
                float hS = (float)(hptSt.HubRadius * sc), tS = (float)(hptSt.TipRadius * sc);
                float cS = (float)(hptSt.Chord * sc), thS = cS * 0.12f;
                float zStHPT = zHPT - cS * 0.8f;
                vTurbineStators.BoolAdd(new Voxels(new SdfTwistedBladeRow(hS, tS, cS, thS, -(float)hptSt.StaggerAngle * 0.8f, -(float)hptSt.StaggerAngle * 0.9f, zStHPT, hptSt.BladeCount + 4), domain));
            }
            if (fp.LPTStages.Count > 0)
            {
                var lptSt = fp.LPTStages[0];
                float hS = (float)(lptSt.HubRadius * sc), tS = (float)(lptSt.TipRadius * sc);
                float cS = (float)(lptSt.Chord * sc), thS = cS * 0.12f;
                float zStLPT = zLPT - cS * 0.8f;
                vTurbineStators.BoolAdd(new Voxels(new SdfTwistedBladeRow(hS, tS, cS, thS, -(float)lptSt.StaggerAngle * 0.8f, -(float)lptSt.StaggerAngle * 0.9f, zStLPT, lptSt.BladeCount + 4), domain));
            }
            JetEngineFabrication.SaveSTL(vTurbineStators, outDir, "Jet_Turbine_Stators.stl");
            Library.oViewer().Add(vTurbineStators, 19);
            Library.oViewer().SetGroupMaterial(19, new ColorFloat(0.7f, 0.6f, 0.5f), 0.6f, 0.1f);

            // T1-6: Pre-swirl nozzle slots on HPT inner stator platform
            Library.Log("Generating pre-swirl nozzle slots on HPT inner platform...");
            float psRadius = coreR * 0.55f;  // inner casing at HPT
            var vPreSwirl  = new Voxels(new SdfPreSwirlSlots(
                psRadius, zHPT - 5f, 6f, 4f, 45f, 36), domain);
            JetEngineFabrication.SaveSTL(vPreSwirl, outDir, "Jet_PreSwirl_Slots.stl");
            Library.oViewer().Add(vPreSwirl, 31);
            Library.oViewer().SetGroupMaterial(31, new ColorFloat(0.4f, 0.7f, 0.9f), 0.5f, 0.2f);
        }
    }
}
