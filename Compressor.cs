using PicoGK;
using System;
using System.IO;
using System.Linq;
using System.Numerics;

namespace JetEngine
{
    public static class Compressor
    {
        public static void BuildCompressor(EngineFlowPath fp, float sc, BBox3 domain, string outDir, float zLPC, float zHPC, float zComb, float coreR)
        {
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
            JetEngineFabrication.SaveSTL(vHPCBlades, outDir, "Jet_HPC_Blades.stl");
            Library.oViewer().Add(vHPCBlades, 2);
            Library.oViewer().SetGroupMaterial(2, new ColorFloat(0.7f, 0.75f, 0.8f), 0.6f, 0.1f);

            JetEngineFabrication.SaveSTL(vHPCDisks, outDir, "Jet_HPC_Disks.stl");
            Library.oViewer().Add(vHPCDisks, 14);
            Library.oViewer().SetGroupMaterial(14, new ColorFloat(0.6f, 0.6f, 0.65f), 0.7f, 0.05f);

            // Stator vanes
            Library.Log("Generating stator vanes...");
            var vStat = new Voxels();
            float zSt = zLPC + 30f;
            foreach (var st in fp.HPCStages.Take(3))
            {
                float hS = (float)(st.HubRadius * sc);
                float tS = (float)(st.TipRadius * sc);
                float cS = Math.Max((float)(st.Chord * sc) * 0.9f, 8f);
                float thS = Math.Max(cS * 0.12f, 5f);
                vStat.BoolAdd(new Voxels(new SdfTwistedBladeRow(hS, tS, cS, thS, -(float)st.StaggerAngle * 0.8f, -(float)st.StaggerAngle * 0.9f, zSt, st.BladeCount + 2), domain));
                zSt += cS * 1.8f;
            }
            JetEngineFabrication.SaveSTL(vStat, outDir, "Jet_Stators.stl");
            Library.oViewer().Add(vStat, 9);
            Library.oViewer().SetGroupMaterial(9, new ColorFloat(0.6f, 0.8f, 0.6f), 0.6f, 0.1f);
        }
    }
}
