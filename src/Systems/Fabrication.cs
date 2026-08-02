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

            var fanStage = fp.FanStages[0];
            float fanTipRs = (float)(fanStage.TipRadius * sc);
            float combOR_early = coreR + 60f;

            // ── 1. Fan Build ──
            Fan.BuildFan(fp, sc, domain, outDir, zFan, zLPC, zHPC, fanTipRs, coreR, combOR_early);

            // ── 2. Compressor Build ──
            Compressor.BuildCompressor(fp, sc, domain, outDir, zLPC, zHPC, zComb, coreR);

            // ── 3. Combustor Build ──
            Combustor.BuildCombustor(comb, sc, domain, outDir, zComb, coreR);

            // ── 4. Turbine Build ──
            Turbine.BuildTurbine(fp, sc, domain, outDir, zHPT, zLPT, coreR);

            // ── 5. Shafts & Bearings Build ──
            ShaftsAndBearings.BuildShaftsAndBearings(fp, sc, domain, outDir, zFan, zLPC, zHPC, zComb, zHPT, zLPT, zNozzle, coreR);

            // ── 6. Casing & Mounts Build ──
            CasingAndMounts.BuildCasingAndMounts(fp, sc, domain, outDir, zFan, zLPC, zHPC, zComb, zHPT, zLPT, zNozzle, coreR, fanTipRs, comb, rMax);

            // ── 7. Core Nozzle ──
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

        public static void SaveSTL(Voxels v, string dir, string name)
        {
            string path = Path.Combine(dir, name);
            v.mshAsMesh().SaveToStlFile(path);
            Library.Log($"  Saved: {name}");
        }
    }

}
