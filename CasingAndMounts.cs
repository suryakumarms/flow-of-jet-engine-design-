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
    public static class EngineMountSystem
    {
        public class MountResult
        {
            public double ForwardMountForce_kN { get; set; }
            public double AftMountForce_kN { get; set; }
            public double MountSafetyFactor { get; set; }
            public double PylonDeflection_mm { get; set; }
            public bool MountStructuralPassed { get; set; }
        }

        public static MountResult Solve(double thrust_N, double engineWeight_N, double g_maneuver = 9.0, double gyroMoment_Nm = 15000.0, double mountThickness_mm = 25.0)
        {
            var r = new MountResult();
            
            // 3-link mount system (two forward mounts, one aft mount)
            double F_vertical = engineWeight_N * g_maneuver;
            
            // Moment balance
            double engine_length = 2.5; // m
            r.ForwardMountForce_kN = (0.70 * thrust_N + 0.5 * F_vertical + gyroMoment_Nm / engine_length) / 1000.0;
            r.AftMountForce_kN = (0.30 * thrust_N + 0.5 * F_vertical + gyroMoment_Nm / engine_length) / 1000.0;
            
            // Mount capability scales linearly with thickness (titanium yield limit)
            double allowable_mount_force_kN = 12.0 * mountThickness_mm; 
            r.MountSafetyFactor = allowable_mount_force_kN / Math.Max(r.ForwardMountForce_kN, 1.0);
            
            // Pylon stiffness deflection: K_pylon scales with mount thickness
            double k_pylon = 45e6 * (mountThickness_mm / 15.0);
            r.PylonDeflection_mm = (F_vertical + thrust_N) / k_pylon * 1000.0;
            
            r.MountStructuralPassed = r.MountSafetyFactor >= 1.5 && r.PylonDeflection_mm < 5.0; // 5mm deflection limit
            
            return r;
        }
    }

    public static class CasingAndMounts
    {
        public static void BuildCasingAndMounts(
            EngineFlowPath fp, float sc, BBox3 domain, string outDir,
            float zFan, float zLPC, float zHPC, float zComb, float zHPT, float zLPT, float zNozzle,
            float coreR, float fanTipRs, CombustorDesign comb, float rMax)
        {
            float combIR = (float)(comb.InnerRadius_m * sc);
            float combOR = (float)(comb.OuterRadius_m * sc);
            float fanHubR = (float)(fp.FanStages[0].HubRadius * sc);
            float fanChord = (float)(fp.FanStages[0].Chord * sc);

            Library.Log("Generating split outer casing (upper + lower shells with flanges)...");
            Func<float, float> casingProfile = z =>
            {
                if (z < zFan) return fanTipRs + 5f;
                if (z < zHPC) return fanTipRs + 5f - (z - zFan) / (zHPC - zFan) * (fanTipRs - coreR - 20f);
                if (z < zComb) return coreR + 25f;
                if (z < zHPT) return combOR + 10f;
                if (z < zNozzle) return combOR + 10f - (z - zHPT) / (zNozzle - zHPT) * (combOR - coreR);
                return coreR + 5f;
            };

            var vCasingShell = new Voxels(new SdfRevolution(casingProfile, 0f, 3f, -50f, zNozzle + 50f), domain);
            var vGyroid     = new Voxels(new SdfGyroid(25f, 0f), domain);
            var vCasingLat  = new Voxels(vCasingShell);
            vCasingLat.BoolIntersect(vGyroid);
            var vInnerSkin  = new Voxels(new SdfRevolution(casingProfile, 0.0f, 5.0f, -50f, zNozzle + 50f), domain);
            var vOuterSkin  = new Voxels(new SdfRevolution(casingProfile, 20.0f, 5.0f, -50f, zNozzle + 50f), domain);
            var vCasingFull = new Voxels();
            vCasingFull.BoolAdd(vCasingLat);
            vCasingFull.BoolAdd(vInnerSkin);
            vCasingFull.BoolAdd(vOuterSkin);

            // Forward Mount (at zFan, on top centerline)
            float rCasingFan = casingProfile(zFan) + 20f;
            var vForwardMount = new Voxels(new SdfCylinder(new Vector3(0, rCasingFan, zFan), new Vector3(0, rCasingFan + 40f, zFan), 15f), domain);
            vForwardMount.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, rCasingFan + 25f, zFan - 10f), new Vector3(0, rCasingFan + 25f, zFan + 10f), 8f), domain));
            vCasingFull.BoolAdd(vForwardMount);

            // Aft Mount (at zLPT, on top centerline)
            float rCasingLPT = casingProfile(zLPT) + 20f;
            var vAftMount = new Voxels(new SdfCylinder(new Vector3(0, rCasingLPT, zLPT), new Vector3(0, rCasingLPT + 40f, zLPT), 15f), domain);
            vAftMount.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, rCasingLPT + 25f, zLPT - 10f), new Vector3(0, rCasingLPT + 25f, zLPT + 10f), 8f), domain));
            vCasingFull.BoolAdd(vAftMount);

            // Apply bypass nozzle chevrons (20 teeth, 35mm depth) at the casing exit
            var vBypassChevrons = new Voxels(new SdfChevronCut(zNozzle - 35f, zNozzle + 15f, 0f, rMax * 2.5f, 20), domain);
            vCasingFull.BoolSubtract(vBypassChevrons);

            BBox3 upperDomain = new BBox3(new Vector3(-rMax, 0, -100), new Vector3(rMax, rMax, zNozzle + 100));
            BBox3 lowerDomain = new BBox3(new Vector3(-rMax, -rMax, -100), new Vector3(rMax, 0, zNozzle + 100));

            // Upper Casing half with flange
            var vCasingUpper = new Voxels(vCasingFull);
            vCasingUpper.BoolIntersect(new Voxels(new SdfDisk(0f, rMax * 2f, zNozzle / 2f, zNozzle + 200f), upperDomain));
            var vUpperFlange = new Voxels(new SdfAnnulus(z => casingProfile(z) + 15f, z => casingProfile(z) + 35f, zNozzle / 2f, zNozzle + 200f), upperDomain);
            vUpperFlange.BoolIntersect(new Voxels(new SdfDisk(0f, rMax * 2f, zNozzle / 2f, 10f), upperDomain));
            vCasingUpper.BoolAdd(vUpperFlange);
            JetEngineFabrication.SaveSTL(vCasingUpper, outDir, "Jet_Casing_Upper.stl");
            Library.oViewer().Add(vCasingUpper, 6);
            Library.oViewer().SetGroupMaterial(6, new ColorFloat(0.5f, 0.5f, 0.55f), 0.4f, 0.2f);

            // Lower Casing half with flange
            var vCasingLower = new Voxels(vCasingFull);
            vCasingLower.BoolIntersect(new Voxels(new SdfDisk(0f, rMax * 2f, zNozzle / 2f, zNozzle + 200f), lowerDomain));
            var vLowerFlange = new Voxels(new SdfAnnulus(z => casingProfile(z) + 15f, z => casingProfile(z) + 35f, zNozzle / 2f, zNozzle + 200f), lowerDomain);
            vLowerFlange.BoolIntersect(new Voxels(new SdfDisk(0f, rMax * 2f, zNozzle / 2f, 10f), lowerDomain));
            vCasingLower.BoolAdd(vLowerFlange);
            JetEngineFabrication.SaveSTL(vCasingLower, outDir, "Jet_Casing_Lower.stl");
            Library.oViewer().Add(vCasingLower, 18);
            Library.oViewer().SetGroupMaterial(18, new ColorFloat(0.45f, 0.45f, 0.5f), 0.4f, 0.2f);

            // Inner core casing (cowl)
            Library.Log("Generating inner core casing (cowl)...");
            Func<float, float> innerCasingProfile = z =>
            {
                if (z < zFan) return fanHubR;
                if (z < zHPC) return fanHubR + (z - zFan) / (zHPC - zFan) * (coreR - fanHubR);
                if (z < zComb) return coreR + 10f;
                if (z < zHPT) return combIR - 10f;
                if (z < zNozzle) return coreR + 10f;
                return coreR;
            };
            var vInnerCasing = new Voxels(new SdfRevolution(innerCasingProfile, 0f, 4.0f, zFan, zNozzle), domain);
            JetEngineFabrication.SaveSTL(vInnerCasing, outDir, "Jet_Inner_Casing.stl");
            Library.oViewer().Add(vInnerCasing, 23);
            Library.oViewer().SetGroupMaterial(23, new ColorFloat(0.5f, 0.5f, 0.5f), 0.7f, 0.15f);

            // Accessory Gearbox (AGB) & Tower Shaft Casing
            Library.Log("Generating Accessory Gearbox (AGB) & Tower Shaft Casing...");
            var vAGB = new Voxels();
            float zMidHPC = (zHPC + zComb) / 2.0f;
            float rInnerAGB = coreR;
            float rOuterAGB = casingProfile(zMidHPC) + 30f;
            vAGB.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, -rInnerAGB, zMidHPC), new Vector3(0, -rOuterAGB, zMidHPC), 12f), domain));
            vAGB.BoolSubtract(new Voxels(new SdfCylinder(new Vector3(0, -rInnerAGB - 5f, zMidHPC), new Vector3(0, -rOuterAGB + 5f, zMidHPC), 8f), domain));
            vAGB.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, -rOuterAGB - 20f, zMidHPC - 15f), new Vector3(0, -rOuterAGB - 20f, zMidHPC + 15f), 35f), domain));
            JetEngineFabrication.SaveSTL(vAGB, outDir, "Jet_AGB_Gearbox.stl");
            Library.oViewer().Add(vAGB, 26);
            Library.oViewer().SetGroupMaterial(26, new ColorFloat(0.55f, 0.55f, 0.6f), 0.8f, 0.1f);

            // Oil Cooler Blocks
            Library.Log("Generating Oil Cooler Blocks (FCOC/ACOC)...");
            var vOilCoolers = new Voxels();
            float rBypassMid = (coreR + fanTipRs) / 2.0f;
            vOilCoolers.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, rBypassMid - 20f, zMidHPC), new Vector3(0, rBypassMid + 20f, zMidHPC), 25f), domain));
            vOilCoolers.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, coreR + 40f, zHPT), new Vector3(0, coreR + 40f, zHPT + 40f), 20f), domain));
            JetEngineFabrication.SaveSTL(vOilCoolers, outDir, "Jet_Oil_Coolers.stl");
            Library.oViewer().Add(vOilCoolers, 27);
            Library.oViewer().SetGroupMaterial(27, new ColorFloat(0.7f, 0.55f, 0.55f), 0.7f, 0.1f);
        }
    }
}
