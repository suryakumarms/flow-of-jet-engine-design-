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
    public static class CompressorMap
    {
        public class MP{public double Nc_pct,Wc,PR,Eta,SM;public bool Surge;}
        public class MapRes{public List<MP> Pts=new();public double B;public bool SurgeRisk;public double DC60;public double SMLoss;}
        
        public static MapRes Generate(CycleResult cy,EngineFlowPath fp, double DC60_inlet = 0.15)
        {
            Console.WriteLine("═══ COMPRESSOR MAP (Moore-Greitzer & Inlet Distortion) ═══");
            var r=new MapRes();
            r.DC60 = DC60_inlet; // e.g. 0.15 represents severe crosswind or S-duct inlet
            
            // Loss of Surge Margin due to Inlet Distortion:
            // Delta_SM = DC60 * K_theta (where K_theta is sensitivity ~ 0.5 for modern transonics)
            r.SMLoss = r.DC60 * 0.5;
            
            double Tt25=cy.Stations.ContainsKey(25)?cy.Stations[25].Tt:500;
            double Pt25=cy.Stations.ContainsKey(25)?cy.Stations[25].Pt:500e3;
            double Wcd=cy.CoreMassFlow*Math.Sqrt(288.15/Tt25)/(Pt25/101325);
            double PRd=cy.Stations.ContainsKey(3)&&cy.Stations.ContainsKey(25)?cy.Stations[3].Pt/cy.Stations[25].Pt:5;
            
            foreach(int pct in new[]{70,80,90,95,100}){
                double nc=pct/100.0,Wc=Wcd*nc*(1+.1*(1-nc)),PR=1+(PRd-1)*nc*nc,eta=.88-.04*Math.Pow(1-nc,2);
                
                // Surge pressure ratio
                double PRs=1+(PRd-1)*nc*nc*1.15;
                
                // Clean surge margin
                double clean_SM=(PRs-PR)/PR;
                
                // Distorted surge margin
                double SM = clean_SM - r.SMLoss;
                
                r.Pts.Add(new MP{Nc_pct=pct,Wc=Wc,PR=PR,Eta=eta,SM=SM,Surge=SM<.05});
                Console.WriteLine($"  Nc={pct}% Wc={Wc:F2} PR={PR:F2} η={eta:F3} SM(clean)={clean_SM*100:F1}% SM(distorted)={SM*100:F1}%");
            }
            
            // Greitzer B parameter for surge vs rotating stall boundary
            // B = U / (2*a) * sqrt(V_p / (A_c * L_c))
            double U=fp.HP_RPM*2*Math.PI/60*(fp.HPCStages.Count>0?fp.HPCStages[0].MeanRadius:.15);
            double a_sound = Math.Sqrt(1.4 * 287 * Tt25);
            r.B=(U/(2*a_sound))*Math.Sqrt(.2/(Math.PI*.05*.05*.5));
            r.SurgeRisk=r.B>0.8;
            
            Console.WriteLine($"  Inlet DC60={r.DC60:F2} -> SM Loss={r.SMLoss*100:F1}%");
            Console.WriteLine($"  Moore-Greitzer B={r.B:F3} (B>0.8 implies deep surge, B<0.8 rotating stall)");
            Console.WriteLine($"  System Status: {(r.Pts.Any(p=>p.Surge)?"✗ SURGE / STALL DETECTED":"✓ AERODYNAMICALLY STABLE")}");
            return r;
        }

        // Multi-stage stage stacking calculation (Gap 20)
        public static double StackingSolve(EngineFlowPath fp, CycleResult cy, double inlet_massflow)
        {
            // HPC inlet is Station 25 (LPC exit)
            double P_in = cy.Stations.ContainsKey(25) ? cy.Stations[25].Pt : 101325.0;
            double T_in = cy.Stations.ContainsKey(25) ? cy.Stations[25].Tt : 288.15;
            double P_curr = P_in;
            double T_curr = T_in;
            double W = inlet_massflow;

            foreach (var st in fp.HPCStages)
            {
                double rho = P_curr / (287.0 * T_curr);
                double A = Math.PI * (st.TipRadius * st.TipRadius - st.HubRadius * st.HubRadius);
                double Va = W / (Math.Max(rho, 0.01) * Math.Max(A, 0.01));
                double U = st.RPM * 2.0 * Math.PI / 60.0 * st.MeanRadius;
                double phi = Va / Math.Max(U, 1.0);
                
                // Stage pressure and temperature rise from stage map characteristics
                double phi_design = 0.5;
                double dH_design = 0.3 * U * U; // design stage loading
                double loading = dH_design * (1.0 - 2.0 * (phi - phi_design)); // loading decreases with flow
                loading = Math.Max(loading, 0.0);
                
                double eta_stage = 0.89 - 0.2 * Math.Pow(phi - phi_design, 2);
                eta_stage = Math.Clamp(eta_stage, 0.5, 0.95);
                
                T_curr += loading / 1005.0;
                double pr = Math.Pow(1.0 + eta_stage * loading / (1005.0 * T_curr), 1.4 / 0.4);
                P_curr *= pr;
            }
            return P_curr / P_in; // total OPR of stacked HPC stages
        }
    }

}
