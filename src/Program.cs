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
    class Program
    {
        static void Main(string[] args)
        {
            string testName = args.Length > 0 ? args[0].ToLower() : "help";
            Console.WriteLine($"JET ENGINE DESIGN PLATFORM — Running: {testName}");
            
            string outDir = Path.Combine(Environment.CurrentDirectory, "TestOutput");
            Directory.CreateDirectory(outDir);
            
            try
            {
                switch (testName)
                {
                    case "jet_design":
                    case "design":
                    {
                        // Full closed-loop design: Gates 1→2→3→4→6
                        var req = DefaultMission();
                        var (cycle, fp, comb) = ClosedLoopDesigner.DesignEngine(req);
                        break;
                    }
                    
                    case "jet_fabrication":
                    case "fabrication":
                    {
                        // Full pipeline: design → fabrication STL
                        var req = DefaultMission();
                        var (cycle, fp, comb) = ClosedLoopDesigner.DesignEngine(req);
                        JetEngineFabrication.Task(cycle, fp, comb);
                        break;
                    }
                    
                    case "jet_cycle":
                    case "cycle":
                    {
                        // Gate 1 only: Brayton cycle
                        var req = DefaultMission();
                        var result = CycleOptimizer.SolveWithAutoCorrect(req);
                        result.Print();
                        break;
                    }
                    
                    case "jet_blades":
                    case "blades":
                    {
                        // Gates 1+2: Cycle + blade geometry
                        var req = DefaultMission();
                        var cycle = BraytonCycleSolver.SolveOnDesign(req);
                        cycle.Print();
                        var fp = FlowPathGenerator.Generate(cycle, req);
                        break;
                    }
                    
                    case "jet_cfd":
                    {
                        var req_cfd = DefaultMission();
                        var cycle_cfd = CycleOptimizer.SolveWithAutoCorrect(req_cfd);
                        var fp_cfd = FlowPathGenerator.Generate(cycle_cfd, req_cfd);
                        NavierStokesCFD.AnalyzeAllBladeRows(fp_cfd, cycle_cfd);
                        break;
                    }
                    case "jet_fea":
                    {
                        var req_fea = DefaultMission();
                        var cycle_fea = CycleOptimizer.SolveWithAutoCorrect(req_fea);
                        var fp_fea = FlowPathGenerator.Generate(cycle_fea, req_fea);
                        FiniteElementAnalysis.AnalyzeAllStages3D(fp_fea, cycle_fea);
                        LCFandTMF.EvaluateHotSection(fp_fea, cycle_fea);
                        break;
                    }
                    case "jet_cert":
                    {
                        var req_cert = DefaultMission();
                        var cycle_cert = CycleOptimizer.SolveWithAutoCorrect(req_cert);
                        var fp_cert = FlowPathGenerator.Generate(cycle_cert, req_cert);
                        var comb_cert = CombustorDesign.Design(cycle_cert, fp_cert);
                        CertificationPhysics.RunAll(fp_cert, cycle_cert);
                        CombustorAcoustics.Analyze(comb_cert, cycle_cert);
                        EngineAcoustics.Evaluate(fp_cert, cycle_cert, req_cert);
                        break;
                    }
                    case "jet_pinn":
                    {
                        var req_pinn = DefaultMission();
                        var cycle_pinn = CycleOptimizer.SolveWithAutoCorrect(req_pinn);
                        var fp_pinn = FlowPathGenerator.Generate(cycle_pinn, req_pinn);
                        PhysicsNeMoClient.GenerateTrainingDataset(fp_pinn, cycle_pinn, nSamples:50);
                        PhysicsNeMoClient.AdjointOptimize(req_pinn);
                        break;
                    }
                    case "jet_twin":
                    {
                        var req_tw = DefaultMission();
                        var (cycle_tw, fp_tw, _) = ClosedLoopDesigner.DesignEngine(req_tw);
                        DigitalTwin.SimulateFleetAging(cycle_tw, fp_tw);
                        break;
                    }
                    
                    case "jet_envelope":
                    {
                        var req_env = DefaultMission();
                        var cycle_env = CycleOptimizer.SolveWithAutoCorrect(req_env);
                        NPSSComponentMatching.SweepEnvelope(cycle_env, req_env);
                        NacelleInstallation.Evaluate(cycle_env, req_env);
                        break;
                    }
                    
                    case "jet_validate":
                    case "validate":
                    {
                        var req = DefaultMission();
                        var (cycle, fp, comb) = ClosedLoopDesigner.DesignEngine(req, maxGlobalIter: 20);
                        if (comb == null) comb = CombustorDesign.Design(cycle, fp);
                        var aero = AeroValidator.ValidateBlades(fp, req);
                        var stress = ThermoStructural.AnalyzeAllStages(fp, cycle);
                        RotorDynamics.AnalyzeSpool("HP", fp.HP_RPM, fp.TotalLength_m*0.4, 0.12, 0.08, 150);
                        RotorDynamics.AnalyzeSpool("LP", fp.LP_RPM, fp.TotalLength_m*0.8, 0.08, 0.05, 200);
                        ManufacturingValidator.Validate(fp, comb);
                        ShaftMechanicals.AnalyzeShaftThrust(fp, cycle);
                        ShaftMechanicals.SizePowerTakeOff(fp, cycle);
                        CombustorDiffuser.Design(cycle, fp, comb);
                        AntiIcingBleed.Evaluate(cycle, req.CruiseAltitude_m, 216.65);
                        GearboxOilThermal.Evaluate(cycle, req.BypassRatio);
                        SpoolTransient.Analyze(fp, cycle, "HP Spool");
                        SpoolTransient.Analyze(fp, cycle, "LP Spool");
                        ThrustReverser.Evaluate(cycle);
                        HighFidelityAudits.RunAllAudits(fp, cycle, comb);


                        Console.WriteLine();
                        Console.WriteLine("════════════════════════════════════════════════════════");
                        Console.WriteLine("  GRID & MESH GENERATION (Polyhedral Dual Contouring)");
                        Console.WriteLine("  Mapping: PicoGK Voxels → Voronoi Dual → NASA LAVA Mesh");
                        Console.WriteLine("════════════════════════════════════════════════════════");
                        double[,,] dummyVoxels = new double[20, 20, 20];
                        for (int i = 5; i < 15; i++)
                            for (int j = 5; j < 15; j++)
                                for (int k = 5; k < 15; k++)
                                    dummyVoxels[i, j, k] = 1.0;
                        PolyhedralMesher.GenerateMesh(dummyVoxels, 0.1, 0.1, 0.1);
                        Console.WriteLine("════════════════════════════════════════════════════════");
                        Console.WriteLine();

                        NavierStokesCFD.AnalyzeAllBladeRows(fp, cycle);
                        FiniteElementAnalysis.AnalyzeAllStages3D(fp, cycle);
                        LCFandTMF.EvaluateHotSection(fp, cycle, comb.OTDF);
                        CombustorAcoustics.Analyze(comb, cycle);
                        EngineAcoustics.Evaluate(fp, cycle, req);
                        CertificationPhysics.RunAll(fp, cycle);
                        PhysicsNeMoClient.AdjointOptimize(req, maxSteps:8);
                        KackerOkapuuLoss.EvaluateTurbineStages(fp, cycle);
                        foreach (var stb in fp.AllStages().Where(s => s.IsRotor).Take(4))
                            PyTurboAeroStyle.PrintBladeSections(stb);
                        NPSSComponentMatching.SweepEnvelope(cycle, req);
                        NASARotor37Validation.Validate();
                        NacelleInstallation.Evaluate(cycle, req);
                        DigitalTwin.AssessHealth(cycle, fp, 0, 0,
                            cycle.Stations.ContainsKey(45)?cycle.Stations[45].Tt:900,
                            cycle.FuelFlow, fp.LP_RPM, 0.5);

                        Console.WriteLine();
                        Console.WriteLine("════════════════════════════════════════════════════════");
                        Console.WriteLine("  T2 ADDITIONAL SIMULATIONS AND INTEGRATION CHECK");
                        Console.WriteLine("════════════════════════════════════════════════════════");
                        // T2-1: Startup torque simulation
                        var startupRes = SpoolTransient.SimulateStartup(fp, cycle);

                        // T2-2: Inter-shaft bearing coupling
                        var couplingRes = InterShaftBearingCoupling.Evaluate(fp, cycle);

                        // T2-3: Elastic blade-disk coupling
                        var elasticRes = ElasticBladeDiskCoupling.Analyze(fp, cycle);

                        // T2-5: Messinger icing
                        var icingRes = MessingerIcingModel.Evaluate(
                            airspeed_ms: 250.0, LWC_kgm3: 4e-4, MVD_um: 20.0,
                            OAT_K: 263.15, P_inlet: cycle.Stations.ContainsKey(1) ? cycle.Stations[1].Pt : 50000,
                            T_inlet: cycle.Stations.ContainsKey(1) ? cycle.Stations[1].Tt : 263.15,
                            bleedT_K: 600.0, bleedFlow_kgs: cycle.CoreMassFlow * 0.015,
                            inletArea_m2: Math.PI * Math.Pow(cycle.FanDiameter_m / 2.0, 2));
                        Console.WriteLine("════════════════════════════════════════════════════════");
                        Console.WriteLine();

                        break;
                    }
                    
                    case "shape":
                        PicoGK.Library.Go(0.1f,
                            Leap71.ShapeKernelExamples.BaseLensShowCase.Task, outDir);
                        break;
                    
                    case "lattice":
                        PicoGK.Library.Go(0.5f,
                            Leap71.LatticeLibraryExamples.LatticeLibraryShowCase.RegularTask, outDir);
                        break;
                    
                    case "quasi":
                        PicoGK.Library.Go(0.4f,
                            Leap71.QuasiCrystalExamples.PenrosePatternShowCase.Task, outDir);
                        break;
                    
                    default:
                        PrintHelp();
                        break;
                }
            }
            catch (Exception e)
            {
                Console.WriteLine($"Error: {e.Message}");
                Console.WriteLine(e.StackTrace);
            }
        }
        
        static MissionRequirements DefaultMission() => new MissionRequirements
        {
            ThrustRequired_N     = 150000.0,   // ~33,700 lbf (CFM LEAP class)
            CruiseMach           = 0.82,
            CruiseAltitude_m     = 10668.0,     // 35,000 ft
            BypassRatio          = 9.0,
            OverallPressureRatio = 40.0,
            FanPressureRatio     = 1.55,
            LPCPressureRatio     = 2.5,
            TurbineInletTemp_K   = 1750.0,
            WaterInjectionActive = true,
            WaterInjectionRatio  = 0.025,
        };
        
        static void PrintHelp()
        {
            Console.WriteLine("Usage: dotnet run [command]");
            Console.WriteLine("");
            Console.WriteLine("  jet_design      — Full closed-loop design (Gates 1-6, auto-correct)");
            Console.WriteLine("  jet_fabrication  — Design + PicoGK STL fabrication (all gates + geometry)");
            Console.WriteLine("  jet_cycle       — Gate 1 only: Brayton cycle thermodynamics");
            Console.WriteLine("  jet_blades      — Gates 1+2: Cycle + blade geometry");
            Console.WriteLine("  jet_validate    — Gates 1-4+6: Full validation without fabrication");
            Console.WriteLine("  jet_twin        — Design + fleet aging (0→30,000 FH) → health CSV");
            Console.WriteLine("  jet_cfd         — NS-CFD MacCormack 2D compressible all blade rows");
            Console.WriteLine("  jet_fea         — 3D FEA (CST elements) + LCF/TMF fatigue life");
            Console.WriteLine("  jet_cert        — Certification (FAR 33 / EASA CS-E + acoustics)");
            Console.WriteLine("  jet_pinn        — PhysicsNeMo adjoint optimization + training data");
            Console.WriteLine("  jet_envelope    — NPSS off-design flight envelope sweep");
            Console.WriteLine("  shape           — PicoGK lens showcase");
            Console.WriteLine("  lattice         — PicoGK lattice showcase");
            Console.WriteLine("  quasi           — PicoGK quasi-crystal showcase");
        }
    }

}
