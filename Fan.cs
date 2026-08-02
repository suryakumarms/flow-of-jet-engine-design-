using PicoGK;
using System;
using System.IO;
using System.Numerics;

namespace JetEngine
{
    public static class Fan
    {
        public static void BuildFan(EngineFlowPath fp, float sc, BBox3 domain, string outDir, float zFan, float zLPC, float zHPC, float fanTipRs, float coreR, float combOR_early)
        {
            Library.Log("Generating separate fan blades and disc...");
            var fanStage = fp.FanStages[0];
            float fanHubR  = (float)(fanStage.HubRadius * sc);
            float fanTipR2 = (float)(fanStage.TipRadius * sc);
            float fanChord = (float)(fanStage.Chord * sc);
            float fanThick = fanChord * (float)fanStage.MaxThicknessRatio;

            // Generate blades with tenons
            var vFanBlades = new Voxels(new SdfBladeRow(fanHubR, fanTipR2, fanChord, fanThick, (float)fanStage.StaggerAngle, zFan, fanStage.BladeCount), domain);
            var vFanTenons = new Voxels(new SdfFirTreeRow(fanHubR, zFan, fanChord, fanChord * 0.22f, 2.5f, 3, 8f, fanStage.BladeCount), domain);
            vFanBlades.BoolAdd(vFanTenons);
            JetEngineFabrication.SaveSTL(vFanBlades, outDir, "Jet_Fan_Blades.stl");
            Library.oViewer().Add(vFanBlades, 1);
            Library.oViewer().SetGroupMaterial(1, new ColorFloat(0.85f, 0.85f, 0.90f), 0.7f, 0.1f);

            // Generate disc with slots and aerodynamic nose cone spinner (parabolic)
            var vFanDisk = new Voxels(new SdfDisk(fanHubR * 0.4f, fanHubR, zFan, 40f), domain);
            var vSpinner = new Voxels(new SdfSpinner(zFan - 120f, zFan - 20f, fanHubR), domain);
            vFanDisk.BoolAdd(vSpinner);

            // Spinner back-plate (closes hollow behind spinner)
            var vSpinnerBack = new Voxels(new SdfSpinnerBackPlate(fanHubR, zFan - 20f, 8f), domain);
            vFanDisk.BoolAdd(vSpinnerBack);

            // Hub drum — rotating barrel connecting spinner/fan disc to LPC first stage
            var vHubDrum = new Voxels(new SdfHubDrum(fanHubR, zFan, zLPC, 6f), domain);
            vFanDisk.BoolAdd(vHubDrum);

            var vFanSlots = new Voxels(new SdfFirTreeRow(fanHubR + 0.5f, zFan, fanChord + 2.0f, fanChord * 0.22f + 0.5f, 2.7f, 3, 8f, fanStage.BladeCount), domain);
            vFanDisk.BoolSubtract(vFanSlots);
            JetEngineFabrication.SaveSTL(vFanDisk, outDir, "Jet_Fan_Disk.stl");
            Library.oViewer().Add(vFanDisk, 13);
            Library.oViewer().SetGroupMaterial(13, new ColorFloat(0.7f, 0.7f, 0.75f), 0.8f, 0.05f);

            // Nacelle inlet bellmouth cowling (rounded intake lip)
            float inletWall = 8f;
            float inletLipR = 18f;
            var vNacelleInlet = new Voxels(new SdfNacelleInlet(fanTipRs + inletWall, zFan, inletWall, inletLipR), domain);
            JetEngineFabrication.SaveSTL(vNacelleInlet, outDir, "Jet_Nacelle_Inlet.stl");
            Library.oViewer().Add(vNacelleInlet, 20);
            Library.oViewer().SetGroupMaterial(20, new ColorFloat(0.8f, 0.85f, 0.9f), 0.4f, 0.2f);

            // Bypass duct annular shell (structural tube from fan to bypass nozzle)
            Func<float, float> bypassInnerR = z => {
                if (z < zHPC) return fanHubR + (z - zFan) / Math.Max(zHPC - zFan, 1f) * (coreR - fanHubR) + 8f;
                return coreR + 18f;
            };
            Func<float, float> bypassOuterR = z => {
                float r;
                if (z < zFan)      r = fanTipRs + 5f;
                else if (z < zHPC) r = fanTipRs + 5f - (z - zFan) / (zHPC - zFan) * (fanTipRs - coreR - 20f);
                else if (z < 650f) r = coreR + 25f; // zComb is 650f
                else if (z < 900f)  r = combOR_early + 10f; // zHPT is 900f
                else if (z < 1400f) r = combOR_early + 10f - (z - 900f) / (1400f - 900f) * combOR_early; // zNozzle is 1400f
                else r = coreR + 5f;
                return r - 8f;
            };
            var vBypassDuct = new Voxels(new SdfBypassDuct(bypassInnerR, bypassOuterR, zFan + fanChord + 12f, 1400f - 40f, 5f), domain);
            JetEngineFabrication.SaveSTL(vBypassDuct, outDir, "Jet_Bypass_Duct.stl");
            Library.oViewer().Add(vBypassDuct, 21);
            Library.oViewer().SetGroupMaterial(21, new ColorFloat(0.6f, 0.7f, 0.8f), 0.3f, 0.15f);

            // Fan Outlet Guide Vanes (FOGVs)
            Library.Log("Generating Fan Outlet Guide Vanes (FOGVs)...");
            var vFOGVs = new Voxels();
            float fogvHub = fanHubR;
            float fogvTip = fanTipRs;
            float fogvChord = fanChord * 0.4f;
            float fogvThick = fogvChord * 0.08f;
            float zFOGV = zFan + fanChord + 10f;
            vFOGVs.BoolAdd(new Voxels(new SdfBladeRow(fogvHub, fogvTip, fogvChord, fogvThick, 15f, zFOGV, 24), domain));
            JetEngineFabrication.SaveSTL(vFOGVs, outDir, "Jet_FOGVs.stl");
            Library.oViewer().Add(vFOGVs, 24);
            Library.oViewer().SetGroupMaterial(24, new ColorFloat(0.7f, 0.7f, 0.7f), 0.7f, 0.1f);

            // Bypass splitter
            Library.Log("Generating bypass splitter...");
            var vSpl = new Voxels(new SdfAnnulus(z => (coreR + fanTipRs) / 2f - 3f, z => (coreR + fanTipRs) / 2f + 3f, zFan + 20f, zHPC - 20f), domain);
            JetEngineFabrication.SaveSTL(vSpl, outDir, "Jet_Splitter.stl");
            Library.oViewer().Add(vSpl, 10);
            Library.oViewer().SetGroupMaterial(10, new ColorFloat(0.5f, 0.7f, 0.9f), 0.5f, 0.1f);

            // Balancing bosses
            Library.Log("Generating balancing boss pads...");
            var vFanBoss = new Voxels(new SdfBalancingBoss(fanHubR * 0.82f, zFan - 5f, 2.0f, 5f, 24), domain);
            JetEngineFabrication.SaveSTL(vFanBoss, outDir, "Jet_Balancing_Bosses.stl");
            Library.oViewer().Add(vFanBoss, 32);
            Library.oViewer().SetGroupMaterial(32, new ColorFloat(0.7f, 0.7f, 0.7f), 0.9f, 0.05f);
        }
    }
}
