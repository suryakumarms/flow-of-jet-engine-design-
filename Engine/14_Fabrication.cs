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
public static class JetEngineFabrication
    {
        public static void Task(CycleResult cycle, EngineFlowPath fp, CombustorDesign comb)
        {
            try
            {
                PicoGK.Library.Go(3.5f, () => Generate(cycle, fp, comb));
            }
            catch (Exception e)
            {
                Console.WriteLine($"Fabrication failed: {e.Message}\n{e.StackTrace}");
            }
        }

        static void Generate(CycleResult cycle, EngineFlowPath fp, CombustorDesign comb)
        {
            Library.Log("╔══════════════════════════════════════════════════╗");
            Library.Log("║  JET ENGINE FABRICATION — PicoGK Voxel Build    ║");
            Library.Log("╚══════════════════════════════════════════════════╝");

            float sc = 1000f;  // metres → mm
            string outDir = Path.Combine(Environment.CurrentDirectory, "TestOutput");
            Directory.CreateDirectory(outDir);

            // ── Engine Axial Layout (Z axis = engine axis, mm) ──
            float zFan      = 0;
            float zLPC      = 120;
            float zHPC      = 250;
            float zComb     = 650;
            float zHPT      = 900;
            float zLPT      = 1050;
            float zNozzle   = 1400;

            float fanTipR   = (float)(cycle.FanDiameter_m / 2.0 * sc);
            float coreR     = (float)(cycle.CoreDiameter_m / 2.0 * sc);
            float rMax      = fanTipR + 80f;

            BBox3 domain = new BBox3(
                new Vector3(-rMax, -rMax, -100),
                new Vector3(rMax, rMax, zNozzle + 100));

            // ════════════════════════════════════════
            //  1. FAN ASSEMBLY (SEPARATE BLADES & DISC)
            // ════════════════════════════════════════
            Library.Log("Generating separate fan blades and disc...");
            var fanStage = fp.FanStages[0];
            float fanHubR  = (float)(fanStage.HubRadius * sc);
            float fanTipRs = (float)(fanStage.TipRadius * sc);
            float fanChord = (float)(fanStage.Chord * sc);
            float fanThick = fanChord * (float)fanStage.MaxThicknessRatio;

            // Generate blades with tenons
            var vFanBlades = new Voxels(new SdfBladeRow(fanHubR, fanTipRs, fanChord, fanThick, (float)fanStage.StaggerAngle, zFan, fanStage.BladeCount), domain);
            var vFanTenons = new Voxels(new SdfFirTreeRow(fanHubR, zFan, fanChord, fanChord * 0.22f, 2.5f, 3, 8f, fanStage.BladeCount), domain);
            vFanBlades.BoolAdd(vFanTenons);
            SaveSTL(vFanBlades, outDir, "Jet_Fan_Blades.stl");
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
            SaveSTL(vFanDisk, outDir, "Jet_Fan_Disk.stl");
            Library.oViewer().Add(vFanDisk, 13);
            Library.oViewer().SetGroupMaterial(13, new ColorFloat(0.7f, 0.7f, 0.75f), 0.8f, 0.05f);

            // Nacelle inlet bellmouth cowling (rounded intake lip)
            float inletWall = 8f;
            float inletLipR = 18f;
            var vNacelleInlet = new Voxels(new SdfNacelleInlet(fanTipRs + inletWall, zFan, inletWall, inletLipR), domain);
            SaveSTL(vNacelleInlet, outDir, "Jet_Nacelle_Inlet.stl");
            Library.oViewer().Add(vNacelleInlet, 20);
            Library.oViewer().SetGroupMaterial(20, new ColorFloat(0.8f, 0.85f, 0.9f), 0.4f, 0.2f);

            // Bypass duct annular shell (structural tube from fan to bypass nozzle)
            // bypassOuterR uses the same piecewise formula as casingProfile (declared later in section 6)
            // but inlined here to avoid a forward-reference compile error.
            float combOR_early = coreR + 60f; // approximate outer combustor radius
            Func<float, float> bypassInnerR = z => {
                if (z < zHPC) return fanHubR + (z - zFan) / Math.Max(zHPC - zFan, 1f) * (coreR - fanHubR) + 8f;
                return coreR + 18f;
            };
            Func<float, float> bypassOuterR = z => {
                float r;
                if (z < zFan)      r = fanTipRs + 5f;
                else if (z < zHPC) r = fanTipRs + 5f - (z - zFan) / (zHPC - zFan) * (fanTipRs - coreR - 20f);
                else if (z < zComb) r = coreR + 25f;
                else if (z < zHPT)  r = combOR_early + 10f;
                else if (z < zNozzle) r = combOR_early + 10f - (z - zHPT) / (zNozzle - zHPT) * combOR_early;
                else r = coreR + 5f;
                return r - 8f;
            };
            var vBypassDuct = new Voxels(new SdfBypassDuct(bypassInnerR, bypassOuterR, zFan + fanChord + 12f, zNozzle - 40f, 5f), domain);
            SaveSTL(vBypassDuct, outDir, "Jet_Bypass_Duct.stl");
            Library.oViewer().Add(vBypassDuct, 21);
            Library.oViewer().SetGroupMaterial(21, new ColorFloat(0.6f, 0.7f, 0.8f), 0.3f, 0.15f);

            // ════════════════════════════════════════
            //  2. HPC ASSEMBLY (SEPARATE BLADES & DISCS)
            // ════════════════════════════════════════
            Library.Log("Generating separate HPC blades and disc rings...");
            var vHPCBlades = new Voxels();
            var vHPCDisks = new Voxels();
            float zPos = zHPC;
            foreach (var stage in fp.HPCStages)
            {
                float hR = (float)(stage.HubRadius * sc);
                float tR = (float)(stage.TipRadius * sc);
                float ch = (float)(stage.Chord * sc);
                float th = ch * (float)stage.MaxThicknessRatio;

                // Blade row with tenons
                var blades = new Voxels(new SdfBladeRow(hR, tR, ch, th, (float)stage.StaggerAngle, zPos, stage.BladeCount), domain);
                var tenons = new Voxels(new SdfFirTreeRow(hR, zPos, ch, ch * 0.22f, 2.0f, 3, 6f, stage.BladeCount), domain);
                blades.BoolAdd(tenons);
                vHPCBlades.BoolAdd(blades);

                // Disk ring with slots
                var disk = new Voxels(new SdfDisk(hR * 0.82f, hR, zPos, ch * 0.5f), domain);
                var slots = new Voxels(new SdfFirTreeRow(hR + 0.5f, zPos, ch + 2.0f, ch * 0.22f + 0.4f, 2.2f, 3, 6f, stage.BladeCount), domain);
                disk.BoolSubtract(slots);
                vHPCDisks.BoolAdd(disk);

                zPos += ch * 1.5f;
            }
            SaveSTL(vHPCBlades, outDir, "Jet_HPC_Blades.stl");
            Library.oViewer().Add(vHPCBlades, 2);
            Library.oViewer().SetGroupMaterial(2, new ColorFloat(0.7f, 0.75f, 0.8f), 0.6f, 0.1f);

            SaveSTL(vHPCDisks, outDir, "Jet_HPC_Disks.stl");
            Library.oViewer().Add(vHPCDisks, 14);
            Library.oViewer().SetGroupMaterial(14, new ColorFloat(0.6f, 0.6f, 0.65f), 0.7f, 0.05f);

            // ════════════════════════════════════════
            //  3. COMBUSTOR
            // ════════════════════════════════════════
            Library.Log("Generating combustor...");
            float combIR = (float)(comb.InnerRadius_m * sc);
            float combOR = (float)(comb.OuterRadius_m * sc);
            float combLen = (float)(comb.Length_m * sc);
            float linerT = Math.Max((float)(comb.LinerThickness_m * sc), 6.0f);

            var vCombOuter = new Voxels(new SdfRevolution(z => combOR, 0f, linerT, zComb, zComb + combLen), domain);
            var vCombInner = new Voxels(new SdfRevolution(z => combIR, -linerT, linerT, zComb, zComb + combLen), domain);
            
            // 8x Dilution ports (radial cuts through inner/outer liner)
            var vDilutionPorts = new Voxels();
            float zDilution = zComb + combLen * 0.75f;
            for (int i = 0; i < 8; i++)
            {
                float angle = i * 45f * MathF.PI / 180f;
                float cos = MathF.Cos(angle);
                float sin = MathF.Sin(angle);
                vDilutionPorts.BoolAdd(new Voxels(new SdfCylinder(
                    new Vector3(cos * (combIR - 20f), sin * (combIR - 20f), zDilution),
                    new Vector3(cos * (combOR + 20f), sin * (combOR + 20f), zDilution),
                    12f), domain));
            }
            vCombOuter.BoolSubtract(vDilutionPorts);
            vCombInner.BoolSubtract(vDilutionPorts);

            // Effusion holes (24 holes at zComb + combLen * 0.3f, 24 holes at zComb + combLen * 0.5f)
            var vEffusionPorts = new Voxels();
            for (int i = 0; i < 24; i++)
            {
                float angle = i * 15f * MathF.PI / 180f;
                float cos = MathF.Cos(angle);
                float sin = MathF.Sin(angle);
                vEffusionPorts.BoolAdd(new Voxels(new SdfCylinder(
                    new Vector3(cos * (combIR - 10f), sin * (combIR - 10f), zComb + combLen * 0.3f),
                    new Vector3(cos * (combOR + 10f), sin * (combOR + 10f), zComb + combLen * 0.3f),
                    3f), domain));
                vEffusionPorts.BoolAdd(new Voxels(new SdfCylinder(
                    new Vector3(cos * (combIR - 10f), sin * (combIR - 10f), zComb + combLen * 0.5f),
                    new Vector3(cos * (combOR + 10f), sin * (combOR + 10f), zComb + combLen * 0.5f),
                    3f), domain));
            }
            vCombOuter.BoolSubtract(vEffusionPorts);
            vCombInner.BoolSubtract(vEffusionPorts);

            var vCombDome = new Voxels(new SdfDisk(combIR, combOR, zComb, linerT * 2f), domain);

            var vCombustor = new Voxels();
            vCombustor.BoolAdd(vCombOuter);
            vCombustor.BoolAdd(vCombInner);
            vCombustor.BoolAdd(vCombDome);

            // Swirler rings around the injectors
            var vSwirlers = new Voxels();
            float injR2_local = (combIR + combOR) / 2f;
            for (int i = 0; i < 12; i++)
            {
                float a2 = i * 30f * MathF.PI / 180f;
                float cx = injR2_local * MathF.Cos(a2);
                float cy2 = injR2_local * MathF.Sin(a2);
                var swirler = new Voxels(new SdfCylinder(new Vector3(cx, cy2, zComb - 2f), new Vector3(cx, cy2, zComb + 4f), 12f), domain);
                swirler.BoolSubtract(new Voxels(new SdfCylinder(new Vector3(cx, cy2, zComb - 4f), new Vector3(cx, cy2, zComb + 6f), 8f), domain));
                for (int j = 0; j < 6; j++)
                {
                    float angleVane = j * 60f * MathF.PI / 180f;
                    float vx = MathF.Cos(angleVane);
                    float vy = MathF.Sin(angleVane);
                    swirler.BoolAdd(new Voxels(new SdfCylinder(
                        new Vector3(cx + vx * 7f, cy2 + vy * 7f, zComb + 1f),
                        new Vector3(cx + vx * 13f, cy2 + vy * 13f, zComb + 1f),
                        2f), domain));
                }
                vSwirlers.BoolAdd(swirler);
            }
            vCombustor.BoolAdd(vSwirlers);

            SaveSTL(vCombustor, outDir, "Jet_Combustor.stl");
            Library.oViewer().Add(vCombustor, 3);
            Library.oViewer().SetGroupMaterial(3, new ColorFloat(1.0f, 0.4f, 0.2f), 0.8f, 0.05f);

            // ════════════════════════════════════════
            //  4. HPT ASSEMBLY (SEPARATE BLADES & DISCS)
            // ════════════════════════════════════════
            Library.Log("HPT: separate blades with cooling & slotted discs...");
            var vHPTBlades = new Voxels();
            var vHPTDisks = new Voxels();
            zPos = zHPT;
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
            SaveSTL(vHPTBlades, outDir, "Jet_HPT_Blades.stl");
            Library.oViewer().Add(vHPTBlades, 4);
            Library.oViewer().SetGroupMaterial(4, new ColorFloat(1.0f, 0.7f, 0.3f), 0.85f, 0.05f);

            SaveSTL(vHPTDisks, outDir, "Jet_HPT_Disks.stl");
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
            SaveSTL(vLPTBlades, outDir, "Jet_LPT_Blades.stl");
            Library.oViewer().Add(vLPTBlades, 5);
            Library.oViewer().SetGroupMaterial(5, new ColorFloat(0.8f, 0.6f, 0.3f), 0.7f, 0.1f);

            SaveSTL(vLPTDisks, outDir, "Jet_LPT_Disks.stl");
            Library.oViewer().Add(vLPTDisks, 16);
            Library.oViewer().SetGroupMaterial(16, new ColorFloat(0.55f, 0.55f, 0.6f), 0.7f, 0.05f);

            // ════════════════════════════════════════
            //  6. OUTER SPLIT CASING (UPPER & LOWER WITH FLANGES)
            // ════════════════════════════════════════
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
            SaveSTL(vCasingUpper, outDir, "Jet_Casing_Upper.stl");
            Library.oViewer().Add(vCasingUpper, 6);
            Library.oViewer().SetGroupMaterial(6, new ColorFloat(0.5f, 0.5f, 0.55f), 0.4f, 0.2f);

            // Lower Casing half with flange
            var vCasingLower = new Voxels(vCasingFull);
            vCasingLower.BoolIntersect(new Voxels(new SdfDisk(0f, rMax * 2f, zNozzle / 2f, zNozzle + 200f), lowerDomain));
            var vLowerFlange = new Voxels(new SdfAnnulus(z => casingProfile(z) + 15f, z => casingProfile(z) + 35f, zNozzle / 2f, zNozzle + 200f), lowerDomain);
            vLowerFlange.BoolIntersect(new Voxels(new SdfDisk(0f, rMax * 2f, zNozzle / 2f, 10f), lowerDomain));
            vCasingLower.BoolAdd(vLowerFlange);
            SaveSTL(vCasingLower, outDir, "Jet_Casing_Lower.stl");
            Library.oViewer().Add(vCasingLower, 18);
            Library.oViewer().SetGroupMaterial(18, new ColorFloat(0.45f, 0.45f, 0.5f), 0.4f, 0.2f);

            // ════════════════════════════════════════
            //  NEW: ADDITIONAL MECHANICAL COMPONENTS
            // ════════════════════════════════════════
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
            SaveSTL(vInnerCasing, outDir, "Jet_Inner_Casing.stl");
            Library.oViewer().Add(vInnerCasing, 23);
            Library.oViewer().SetGroupMaterial(23, new ColorFloat(0.5f, 0.5f, 0.5f), 0.7f, 0.15f);

            Library.Log("Generating Fan Outlet Guide Vanes (FOGVs)...");
            var vFOGVs = new Voxels();
            float fogvHub = fanHubR;
            float fogvTip = fanTipRs;
            float fogvChord = fanChord * 0.4f;
            float fogvThick = fogvChord * 0.08f;
            float zFOGV = zFan + fanChord + 10f;
            vFOGVs.BoolAdd(new Voxels(new SdfBladeRow(fogvHub, fogvTip, fogvChord, fogvThick, 15f, zFOGV, 24), domain));
            SaveSTL(vFOGVs, outDir, "Jet_FOGVs.stl");
            Library.oViewer().Add(vFOGVs, 24);
            Library.oViewer().SetGroupMaterial(24, new ColorFloat(0.7f, 0.7f, 0.7f), 0.7f, 0.1f);

            Library.Log("Generating combustor pre-diffuser snout...");
            Func<float, float> snoutProfile = z =>
            {
                float frac = (z - 590f) / (zComb - 590f);
                frac = Math.Clamp(frac, 0f, 1f);
                return coreR * 0.9f + frac * (combIR - coreR * 0.9f);
            };
            var vSnout = new Voxels(new SdfRevolution(snoutProfile, -2f, 4.0f, 590f, zComb), domain);
            SaveSTL(vSnout, outDir, "Jet_Combustor_Snout.stl");
            Library.oViewer().Add(vSnout, 25);
            Library.oViewer().SetGroupMaterial(25, new ColorFloat(0.65f, 0.65f, 0.7f), 0.7f, 0.15f);

            Library.Log("Generating Accessory Gearbox (AGB) & Tower Shaft Casing...");
            var vAGB = new Voxels();
            float zMidHPC = (zHPC + zComb) / 2.0f;
            float rInnerAGB = coreR;
            float rOuterAGB = casingProfile(zMidHPC) + 30f;
            vAGB.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, -rInnerAGB, zMidHPC), new Vector3(0, -rOuterAGB, zMidHPC), 12f), domain));
            vAGB.BoolSubtract(new Voxels(new SdfCylinder(new Vector3(0, -rInnerAGB - 5f, zMidHPC), new Vector3(0, -rOuterAGB + 5f, zMidHPC), 8f), domain));
            vAGB.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, -rOuterAGB - 20f, zMidHPC - 15f), new Vector3(0, -rOuterAGB - 20f, zMidHPC + 15f), 35f), domain));
            SaveSTL(vAGB, outDir, "Jet_AGB_Gearbox.stl");
            Library.oViewer().Add(vAGB, 26);
            Library.oViewer().SetGroupMaterial(26, new ColorFloat(0.55f, 0.55f, 0.6f), 0.8f, 0.1f);

            Library.Log("Generating Oil Cooler Blocks (FCOC/ACOC)...");
            var vOilCoolers = new Voxels();
            float rBypassMid = (coreR + fanTipRs) / 2.0f;
            vOilCoolers.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, rBypassMid - 20f, zMidHPC), new Vector3(0, rBypassMid + 20f, zMidHPC), 25f), domain));
            vOilCoolers.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0, coreR + 40f, zHPT), new Vector3(0, coreR + 40f, zHPT + 40f), 20f), domain));
            SaveSTL(vOilCoolers, outDir, "Jet_Oil_Coolers.stl");
            Library.oViewer().Add(vOilCoolers, 27);
            Library.oViewer().SetGroupMaterial(27, new ColorFloat(0.7f, 0.55f, 0.55f), 0.7f, 0.1f);

            // ════════════════════════════════════════
            //  7. HP + LP SHAFTS (SEPARATE WITH SPLINES)
            // ════════════════════════════════════════
            Library.Log("Generating separate shafts with splines...");
            // LP shaft: inner, runs full length
            var vLPShaft = new Voxels(new SdfCylinder(new Vector3(0, 0, -50), new Vector3(0, 0, zNozzle), 25f), domain);
            vLPShaft.BoolSubtract(new Voxels(new SdfCylinder(new Vector3(0, 0, -60), new Vector3(0, 0, zNozzle + 10), 19f), domain));
            var vLPSpline = new Voxels(new SdfSpline(-20f, 10f, 24f, 25f, 2f, 16), domain);
            vLPShaft.BoolAdd(vLPSpline);
            SaveSTL(vLPShaft, outDir, "Jet_LP_Shaft.stl");
            Library.oViewer().Add(vLPShaft, 7);
            Library.oViewer().SetGroupMaterial(7, new ColorFloat(0.4f, 0.4f, 0.45f), 0.9f, 0.05f);

            // HP shaft: outer, runs from HPC to HPT
            var vHPShaft = new Voxels(new SdfCylinder(new Vector3(0, 0, zHPC - 20), new Vector3(0, 0, zHPT + 50), 40f), domain);
            vHPShaft.BoolSubtract(new Voxels(new SdfCylinder(new Vector3(0, 0, zHPC - 30), new Vector3(0, 0, zHPT + 60), 32f), domain));
            var vHPSpline = new Voxels(new SdfSpline(zHPT, zHPT + 30f, 39f, 40f, 2.5f, 24), domain);
            vHPShaft.BoolAdd(vHPSpline);
            SaveSTL(vHPShaft, outDir, "Jet_HP_Shaft.stl");
            Library.oViewer().Add(vHPShaft, 17);
            Library.oViewer().SetGroupMaterial(17, new ColorFloat(0.35f, 0.35f, 0.4f), 0.9f, 0.05f);

            // ════════════════════════════════════════
            //  8. CORE NOZZLE
            // ════════════════════════════════════════
            // ════ NEW: STATOR VANES (3 interstage rows) ════
            Library.Log("Generating stator vanes...");
            var vStat=new Voxels(); float zSt=zLPC+30f;
            foreach(var st in fp.HPCStages.Take(3)){
                float hS=(float)(st.HubRadius*sc),tS=(float)(st.TipRadius*sc);
                float cS=Math.Max((float)(st.Chord*sc)*.9f,8f),thS=Math.Max(cS*.12f,5f);
                vStat.BoolAdd(new Voxels(new SdfTwistedBladeRow(hS,tS,cS,thS,-(float)st.StaggerAngle*.8f,-(float)st.StaggerAngle*.9f,zSt,st.BladeCount+2),domain));
                zSt+=cS*1.8f;
            }
            SaveSTL(vStat,outDir,"Jet_Stators.stl");
            Library.oViewer().Add(vStat,9); Library.oViewer().SetGroupMaterial(9,new ColorFloat(.6f,.8f,.6f),.6f,.1f);

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
            SaveSTL(vTurbineStators, outDir, "Jet_Turbine_Stators.stl");
            Library.oViewer().Add(vTurbineStators, 19);
            Library.oViewer().SetGroupMaterial(19, new ColorFloat(0.7f, 0.6f, 0.5f), 0.6f, 0.1f);

            // ════ NEW: BYPASS SPLITTER ════
            Library.Log("Generating bypass splitter...");
            var vSpl=new Voxels(new SdfAnnulus(z => (coreR+fanTipRs)/2f - 3f, z => (coreR+fanTipRs)/2f + 3f, zFan+20f, zHPC-20f),domain);
            SaveSTL(vSpl,outDir,"Jet_Splitter.stl");
            Library.oViewer().Add(vSpl,10); Library.oViewer().SetGroupMaterial(10,new ColorFloat(.5f,.7f,.9f),.5f,.1f);

            // ════ NEW: 12× FUEL INJECTORS ════
            Library.Log("Generating fuel injectors...");
            var vInj=new Voxels(); float injR2=(combIR+combOR)/2f;
            for(int i=0;i<12;i++){
                float a2=i*30f*MathF.PI/180f,cx=injR2*MathF.Cos(a2),cy2=injR2*MathF.Sin(a2);
                var st2=new Voxels(new SdfCylinder(new Vector3(cx*1.2f,cy2*1.2f,zComb+10f),new Vector3(cx*.7f,cy2*.7f,zComb+10f),5f),domain);
                st2.BoolAdd(new Voxels(new SdfCylinder(new Vector3(cx*.7f,cy2*.7f,zComb+5f),new Vector3(cx*.7f,cy2*.7f,zComb+15f),8f),domain));
                vInj.BoolAdd(st2);
            }
            SaveSTL(vInj,outDir,"Jet_Injectors.stl");
            Library.oViewer().Add(vInj,11); Library.oViewer().SetGroupMaterial(11,new ColorFloat(.9f,.5f,.2f),.9f,.05f);

            // 2× Igniter Plugs (radial cylinders at zComb + 15mm, at angles 45° and 135°)
            var vIgniters = new Voxels();
            float igniterRad = (combIR + combOR) / 2f;
            float rOuterCasingComb = casingProfile(zComb) + 20f;
            for (int i = 0; i < 2; i++)
            {
                float angle = (45f + i * 90f) * MathF.PI / 180f;
                float cos = MathF.Cos(angle);
                float sin = MathF.Sin(angle);
                // Radial cylinder from outer casing to combustor dome
                var plug = new Voxels(new SdfCylinder(
                    new Vector3(cos * rOuterCasingComb, sin * rOuterCasingComb, zComb + 15f),
                    new Vector3(cos * (igniterRad + 5f), sin * (igniterRad + 5f), zComb + 15f),
                    6f), domain);
                // Internal ceramic insulator core
                plug.BoolAdd(new Voxels(new SdfCylinder(
                    new Vector3(cos * rOuterCasingComb, sin * rOuterCasingComb, zComb + 15f),
                    new Vector3(cos * igniterRad, sin * igniterRad, zComb + 15f),
                    3f), domain));
                vIgniters.BoolAdd(plug);
            }
            SaveSTL(vIgniters, outDir, "Jet_Igniters.stl");
            Library.oViewer().Add(vIgniters, 28);
            Library.oViewer().SetGroupMaterial(28, new ColorFloat(0.9f, 0.9f, 0.95f), 0.9f, 0.05f);

            // ════ NEW: 3× BEARING RINGS ════
            Library.Log("Generating bearing rings + labyrinth seals...");
            var vBr=new Voxels();
            float[] bz2={zFan+10f,zHPC+80f,zLPT+80f};
            float[] bod={coreR*.8f,coreR*.6f,coreR*.7f};
            for(int i=0;i<3;i++){var ring=new Voxels(new SdfDisk(bod[i]-15f,bod[i],bz2[i]-8f,16f),domain);ring.BoolSubtract(new Voxels(new SdfDisk(bod[i]-15f,bod[i]-6f,bz2[i]-4f,8f),domain));vBr.BoolAdd(ring);}
            SaveSTL(vBr,outDir,"Jet_Bearings.stl");
            Library.oViewer().Add(vBr,12); Library.oViewer().SetGroupMaterial(12,new ColorFloat(.4f,.4f,.5f),.9f,.05f);

            // T1-2: Labyrinth seal teeth on HP shaft (8 teeth at HPC-HPT junction)
            Library.Log("Generating labyrinth seal teeth on HP shaft...");
            var vLabyrinth = new Voxels();
            // HP shaft seals between HPC exit and HPT entry
            vLabyrinth.BoolAdd(new Voxels(new SdfLabyrinthSeals(40f, 4f, 1.5f, 12f, zHPC + 60f, zHPT - 20f, 8), domain));
            // LP shaft seals at fan hub and LPT exit
            vLabyrinth.BoolAdd(new Voxels(new SdfLabyrinthSeals(25f, 3.5f, 1.2f, 10f, zFan + 5f, zLPC - 5f, 5), domain));
            vLabyrinth.BoolAdd(new Voxels(new SdfLabyrinthSeals(25f, 3.5f, 1.2f, 10f, zLPT + 20f, zNozzle - 30f, 6), domain));
            SaveSTL(vLabyrinth, outDir, "Jet_Labyrinth_Seals.stl");
            Library.oViewer().Add(vLabyrinth, 29);
            Library.oViewer().SetGroupMaterial(29, new ColorFloat(0.6f, 0.7f, 0.75f), 0.85f, 0.05f);

            // T1-3: Turbine cooling air manifold piping (HPC exit → HPT NGV face)
            Library.Log("Generating cooling air manifold piping...");
            var vCoolPipes = new Voxels();
            // 6 equidistant tubes running axially along inner casing outer surface
            float pipeR = coreR + 8f;
            for (int i = 0; i < 6; i++)
            {
                float ang  = i * 60f * MathF.PI / 180f;
                float px   = pipeR * MathF.Cos(ang);
                float py   = pipeR * MathF.Sin(ang);
                var pipe   = new Voxels(new SdfCylinder(
                    new Vector3(px, py, zComb - 10f),
                    new Vector3(px, py, zHPT + 10f), 4.5f), domain);
                pipe.BoolSubtract(new Voxels(new SdfCylinder(
                    new Vector3(px, py, zComb - 15f),
                    new Vector3(px, py, zHPT + 15f), 2.5f), domain));
                vCoolPipes.BoolAdd(pipe);
            }
            // Annular HPT inlet manifold ring
            vCoolPipes.BoolAdd(new Voxels(new SdfAnnulus(
                z => pipeR - 5f, z => pipeR + 5f, zHPT - 5f, zHPT + 5f), domain));
            SaveSTL(vCoolPipes, outDir, "Jet_CoolingManifold.stl");
            Library.oViewer().Add(vCoolPipes, 30);
            Library.oViewer().SetGroupMaterial(30, new ColorFloat(0.8f, 0.6f, 0.4f), 0.8f, 0.1f);

            // T1-6: Pre-swirl nozzle slots on HPT inner stator platform
            Library.Log("Generating pre-swirl nozzle slots on HPT inner platform...");
            float psRadius = coreR * 0.55f;  // inner casing at HPT
            var vPreSwirl  = new Voxels(new SdfPreSwirlSlots(
                psRadius, zHPT - 5f, 6f, 4f, 45f, 36), domain);
            SaveSTL(vPreSwirl, outDir, "Jet_PreSwirl_Slots.stl");
            Library.oViewer().Add(vPreSwirl, 31);
            Library.oViewer().SetGroupMaterial(31, new ColorFloat(0.4f, 0.7f, 0.9f), 0.5f, 0.2f);

            // T1-7: Balancing boss pads on fan disc front face
            Library.Log("Generating balancing boss pads...");
            var vFanBoss = new Voxels(new SdfBalancingBoss(fanHubR * 0.82f, zFan - 5f, 2.0f, 5f, 24), domain);
            SaveSTL(vFanBoss, outDir, "Jet_Balancing_Bosses.stl");
            Library.oViewer().Add(vFanBoss, 32);
            Library.oViewer().SetGroupMaterial(32, new ColorFloat(0.7f, 0.7f, 0.7f), 0.9f, 0.05f);

            Library.Log("Generating core nozzle + 6 exhaust struts + plug...");
            Func<float, float> nozzleInner = z =>
            {
                float frac = (z - zLPT) / (zNozzle - zLPT);
                frac = Math.Clamp(frac, 0f, 1f);
                return coreR * 0.8f * (1f - 0.3f * frac);
            };

            var vNozzle = new Voxels(new SdfRevolution(nozzleInner, 0f, 6.0f, zLPT, zNozzle), domain);
            for(int i=0;i<6;i++){
                float a6=i*60f*MathF.PI/180f,rx=MathF.Cos(a6),ry=MathF.Sin(a6),zs=zNozzle-80f;
                vNozzle.BoolAdd(new Voxels(new SdfCylinder(new Vector3(rx*15f,ry*15f,zs),new Vector3(rx*coreR*.75f,ry*coreR*.75f,zs+40f),6f),domain));
            }
            vNozzle.BoolAdd(new Voxels(new SdfCylinder(new Vector3(0,0,zNozzle-100f),new Vector3(0,0,zNozzle+20f),18f),domain));

            // Apply Core nozzle chevrons (16 teeth, 30mm depth)
            var vCoreChevrons = new Voxels(new SdfChevronCut(zNozzle - 30f, zNozzle + 5f, 0f, coreR * 1.5f, 16), domain);
            vNozzle.BoolSubtract(vCoreChevrons);
            SaveSTL(vNozzle, outDir, "Jet_Nozzle.stl");
            Library.oViewer().Add(vNozzle, 8);
            Library.oViewer().SetGroupMaterial(8, new ColorFloat(0.6f, 0.6f, 0.65f), 0.5f, 0.15f);

            Library.Log("╔══════════════════════════════════════════════════╗");
            Library.Log("║  FABRICATION COMPLETE — STLs saved to TestOutput║");
            Library.Log("╚══════════════════════════════════════════════════╝");
        }

        static void SaveSTL(Voxels v, string dir, string name)
        {
            string path = Path.Combine(dir, name);
            v.mshAsMesh().SaveToStlFile(path);
            Library.Log($"  Saved: {name}");
        }
    }

}
