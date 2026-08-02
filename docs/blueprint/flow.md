# 🌀 Jet Engine Computational Platform - Complete Architecture & Flow Blueprint

This document provides a unified overview of the **Jet Engine Computational Design Platform**, mapping the codebase architecture, iterative design solvers, execution sequences, and physics equations.

---

## 🗺️ 1. Codebase Architecture & System Flowchart

This flowchart maps out how data starts at the orchestrator (`Program.cs`) and flows through the modular C# domain models, physics solvers, auxiliary systems, and output/fabrication generators.

```mermaid
flowchart TD
    %% Styling
    classDef entry fill:#1e293b,stroke:#a855f7,stroke-width:2.5px,color:#f8fafc;
    classDef setup fill:#1e293b,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef solver fill:#1e293b,stroke:#f59e0b,stroke-width:2px,color:#f8fafc;
    classDef physics fill:#1e293b,stroke:#ec4899,stroke-width:2px,color:#f8fafc;
    classDef aux fill:#0f172a,stroke:#64748b,color:#cbd5e1;
    classDef out fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#ecfdf5;

    %% Elements
    PROG["Program.cs<br>(CLI Entry Point & Orchestrator)"]
    class PROG entry;

    subgraph Phase1 ["1. Inputs & Sizing Thermodynamics"]
        MASS["MissionAndAtmosphere.cs<br>(MissionRequirements & Atmosphere)"]
        BRAY["BraytonCycle.cs<br>(BraytonCycleSolver & CycleOptimizer)"]
    end
    class MASS setup;
    class BRAY solver;

    subgraph Phase2 ["2. Flowpath & Airfoil Generation"]
        FLOW["FlowPath.cs<br>(EngineFlowPath & Velocity Triangles)"]
    end
    class FLOW solver;

    subgraph Phase3 ["3. Component Physics & Aerodynamics"]
        COMB["Combustor.cs<br>(CombustorDesign & Acoustics)"]
        AERO["FluidAeroDynamics.cs<br>(AeroValidator & Navier-Stokes CFD)"]
    end
    class COMB,AERO physics;

    subgraph Phase4 ["4. Mechanical & Rotor Stress Validation"]
        THERM["Thermostructural.cs<br>(ThermoStructural Stress & FEA)"]
        ROTOR["Rotordynamics.cs<br>(Spool Whirl & Bearing Coupling)"]
        SHAFT["ShaftsAndBearings.cs<br>(Shaft Mechanicals & Seal Analysis)"]
    end
    class THERM,ROTOR,SHAFT physics;

    subgraph Phase5 ["5. Auxiliary Subsystems & Controls"]
        SYS["EngineSystems.cs<br>(FMEA, Fuel, Startup, De-Icing)"]
        CASE["CasingAndMounts.cs<br>(Casing & Mount System)"]
        CTRL["ControlAndValidation.cs<br>(FADEC, Mission Simulation, Audits)"]
    end
    class SYS,CASE,CTRL aux;

    subgraph Phase6 ["6. Optimization & Output"]
        NEMO["AnalysisAndSimulations.cs<br>(PhysicsNeMo PINN Client)"]
        SHAPE["ImplicitShapes.cs<br>(PicoGK Voxel Meshing)"]
        FAB["Fabrication.cs<br>(STL Fabrication Task)"]
    end
    class NEMO solver;
    class SHAPE,FAB out;

    %% Connections
    PROG -->|Configures Requirements| MASS
    PROG -->|Runs Solvers| BRAY
    MASS -->|Station Mass & Temps| BRAY
    
    BRAY -->|Thermodynamic Stations| FLOW
    FLOW -->|Geometric Profiles| COMB
    FLOW -->|Blade Speeds & Mach| AERO
    
    FLOW & BRAY -->|Aerodynamic Loads| THERM
    FLOW & BRAY -->|RPMs & Centroids| ROTOR
    FLOW & BRAY -->|Axial Forces| SHAFT
    
    BRAY & FLOW -->|Boundary Values| SYS
    BRAY & FLOW -->|Thrust Mount Limits| CASE
    BRAY & FLOW -->|Sizing Bounds| CTRL
    
    FLOW & BRAY -->|Dataset Training| NEMO
    NEMO -->|Adjoint Parameter Optimization| BRAY
    
    FLOW & COMB & CASE -->|Dimensions| SHAPE
    SHAPE -->|Water-tight Voxels| FAB
    FAB -->|Generates Production Mesh| PROG
```

---

## 🔄 2. Master Closed-Loop MDAO Design Flowchart

The following flowchart outlines the multi-disciplinary design optimization (MDAO) closed loop executed inside `ClosedLoopDesigner.DesignEngine(...)`. If any physical constraint or gate fails, design parameters (e.g. BPR, OPR, TIT) are dynamically adjusted and re-solved until convergence.

```mermaid
flowchart TD
    %% Styling
    classDef init fill:#0f172a,stroke:#38bdf8,stroke-width:2px,color:#f8fafc;
    classDef gate fill:#7f1d1d,stroke:#f87171,stroke-width:2px,color:#fee2e2;
    classDef step fill:#1e293b,stroke:#a855f7,stroke-width:2px,color:#f8fafc;
    classDef success fill:#064e3b,stroke:#34d399,stroke-width:2px,color:#ecfdf5;

    Start(["Start Design Engine<br>(req, maxGlobalIter)"]) --> Init["Initialize Mission Profile & Constraints<br>(MissionRequirements)"]:::init
    
    Init --> IterStart{"Loop: Global Iteration < maxGlobalIter"}:::step
    
    %% Gate 1
    IterStart -->|Yes| G1["[GATE 1] Cycle Solver<br>(CycleOptimizer.SolveWithAutoCorrect)"]:::step
    G1 --> G1Check{"Is Cycle Valid?<br>(Mass flows, Spool work)"}:::gate
    G1Check -->|No| G1Fix["Auto-Correct Gate 1:<br>Reduce TIT (-25K)<br>Reduce BPR (-0.3)"]:::gate
    G1Fix --> G1
    
    %% Gate 2
    G1Check -->|Yes| G2["[GATE 2] Geometry Sizing<br>(FlowPathGenerator.Generate)"]:::step
    
    %% Gate 3A
    G2 --> G3A["[GATE 3A] Aerodynamic Check<br>(AeroValidator.ValidateBlades)"]:::step
    G3A --> G3ACheck{"Do Stages Pass Aero?<br>(DF <= 0.45, De Haller >= 0.72)"}:::gate
    G3ACheck -->|No| G3AFix["Auto-Correct Gate 3A:<br>Reduce OPR by 3%<br>(req.OverallPressureRatio *= 0.97)"]:::gate
    G3AFix --> IterStart
    
    %% Gate 3B
    G3ACheck -->|Yes| G3B["[GATE 3B] Combustor Sizing<br>(CombustorDesign.Design)"]:::step
    
    %% Gate 4A
    G3B --> G4A["[GATE 4A] Thermo-Structural FEA<br>(ThermoStructural.AnalyzeAllStages)"]:::step
    G4A --> G4ACheck{"Do Blades Pass Stress?<br>(Yield & Larson-Miller Creep)"}:::gate
    G4ACheck -->|No| G4AFix{"Auto-Correct Gate 4A:<br>If HPC: Reduce OPR by 2%<br>If Turbine: Reduce TIT by 25K"}:::gate
    G4AFix --> IterStart
    
    %% Gate 4B & Shaft Thrust
    G4ACheck -->|Yes| G4B["[GATE 4B] Rotordynamics<br>(RotorDynamics.AnalyzeSpool)"]:::step
    G4B --> G4BCheck["Check Critical Speed Whirl Margins<br>(Log warnings if margin < 15%)"]:::step
    
    G4BCheck --> Gap4["[GAP 4] Axial Shaft Thrust Balancing<br>(ShaftMechanicals.AnalyzeShaftThrust)"]:::step
    Gap4 --> Gap4Check{"Are Bearings Overloaded?<br>(Bearing Force <= Limit)"}:::gate
    Gap4Check -->|No| Gap4PTO["Size Power Take-Off (PTO)<br>(ShaftMechanicals.SizePowerTakeOff)"]:::step
    Gap4Check -->|Yes: HP Spool| Gap4HPFix["Auto-Correct HP:<br>Reduce OPR by 1%<br>(req.OverallPressureRatio *= 0.99)"]:::gate
    Gap4Check -->|Yes: LP Spool| Gap4LPFix["Auto-Correct LP:<br>Reduce BPR by 0.3<br>(req.BypassRatio -= 0.3)"]:::gate
    Gap4HPFix --> IterStart
    Gap4LPFix --> IterStart
    
    %% Gap 5 Combustor Diffuser
    Gap4PTO --> Gap5["[GAP 5] Combustor Diffuser<br>(CombustorDiffuser.Design)"]:::step
    Gap5 --> Gap5PFeedback["Feed actual Diffuser Pressure Loss<br>back to req.CombustorPressureLoss"]:::step
    Gap5PFeedback --> Gap5Check{"Is Flame Blowout Risk Active?<br>(Reference velocity too high)"}:::gate
    Gap5Check -->|Yes| Gap5Fix["Auto-Correct Gap 5:<br>Increase Combustor Area<br>(Increase OPR: req.OverallPressureRatio += 0.5)"]:::gate
    Gap5Fix --> IterStart
    
    %% Anti-icing & Gearbox Oil
    Gap5Check -->|No| G3E["[GATE 3E] Anti-Icing Bleed<br>(AntiIcingBleed.Evaluate)"]:::step
    G3E --> G4D["[GATE 4D] Gearbox Oil Thermal<br>(GearboxOilThermal.Evaluate)"]:::step
    G4D --> G4DCheck{"Oil Overtemp Risk?"}:::gate
    G4DCheck -->|Yes| G4DFix["Auto-Correct Gate 4D:<br>Reduce TIT by 10K<br>(req.TurbineInletTemp_K -= 10)"]:::gate
    G4DFix --> IterStart
    
    %% Spool Transient
    G4DCheck -->|No| G5C["[GATE 5C] Spool Transient Sizing<br>(SpoolTransient.Analyze)"]:::step
    G5C --> G5CCheck{"Transient Surge Risk?"}:::gate
    G5CCheck -->|Yes| G5CFix["Auto-Correct Gate 5C:<br>Add VSV/VBV Margin<br>(Reduce OPR: req.OverallPressureRatio *= 0.99)"]:::gate
    G5CFix --> IterStart
    
    %% Thrust Reverser & Manufacturing
    G5CCheck -->|No| G5E["[GATE 5E] Thrust Reverser & Landing<br>(ThrustReverser.Evaluate)"]:::step
    G5E --> G6["[GATE 6] DMLS Manufacturability<br>(ManufacturingValidator.Validate)"]:::step
    
    %% High fidelity simulation layers
    G6 --> HighFid["[SIMULATION & PHYSICS LAYERS]<br>Throughflow (L1) | Comp Map (L2) | Film Cooling (L3)<br>Aeroelasticity (L4) | Bearings (L5) | Seals (L6)<br>Materials (L7) | Melt Pool (L8) | FADEC (L9)<br>Mission Sim (L10) | NSGA-II Sweep (L11)<br>NASA Rotor 37 | Digital Twin Aging"]:::step
    
    HighFid --> Success["[SUCCESS] DESIGN CONVERGED<br>Return (cycle, flowPath, combustor)"]:::success
    
    IterStart -->|No| Failure(["Return Best Available Sizing"]):::gate
```

---

## ⚡ 3. Validation Run Execution Sequence

When executing `dotnet run --project src/JetEngine.csproj`, the platform runs a diagnostic verification across all gates and auxiliary simulation layers:

```mermaid
sequenceDiagram
    autonumber
    actor CLI as Developer (dotnet run)
    participant Program as Program.Main
    participant Designer as ClosedLoopDesigner
    participant Aero as AeroValidator
    participant Structural as ThermoStructural
    participant Rotor as RotorDynamics
    participant Shaft as ShaftMechanicals
    participant Diffuser as CombustorDiffuser
    participant Transient as SpoolTransient
    participant CFD as NavierStokesCFD
    participant FEA as FiniteElementAnalysis
    participant Twin as DigitalTwin
    
    CLI->>Program: Run "validate"
    Program->>Designer: DesignEngine(req, maxGlobalIter: 20)
    activate Designer
    Note over Designer: Global Loop Iteration 1..20
    Designer->>Aero: ValidateBlades(fp, req)
    Aero-->>Designer: AeroCheckResult (Passed/Failed)
    Designer->>Structural: AnalyzeAllStages(fp, cycle)
    Structural-->>Designer: StressResults (Passed/Failed)
    Designer->>Rotor: AnalyzeSpool("HP Spool", HP_RPM, ...)
    Rotor-->>Designer: RotorResult (Passed/Failed)
    Designer->>Shaft: AnalyzeShaftThrust(fp, cycle)
    Shaft-->>Designer: ThrustCheckResult (Passed/Failed)
    Designer->>Diffuser: Design(cycle, fp, comb)
    Diffuser-->>Designer: DiffuserResult (FlameBlowoutRisk)
    Designer->>Transient: Analyze(fp, cycle, "HP/LP Spool")
    Transient-->>Designer: SpoolTransientResult (SurgeRisk)
    deactivate Designer
    
    Note over Program: Sizing Complete. Running Diagnostic Simulation Layers.
    Program->>CFD: AnalyzeAllBladeRows(fp, cycle)
    Program->>FEA: AnalyzeAllStages3D(fp, cycle)
    Program->>Twin: AssessHealth(cycle, fp, ...)
    Program-->>CLI: Success/Fail Report & Diagnostic Data
```

---

## 🧠 4. Equation & Physics Data Flow Loop

```
                  [Flight Mach M_0 & Altitude h]
                                │
                                ▼
 1. Standard Atmosphere Pressure & Temp (NASA/TM-2005-213659):
    T(z) = T_std + LR_std * (z - z_std)  ---> Senses freestream boundaries.
                                │
                                ▼
 2. Dimensionless Total Temperature & Pressure (Mattingly AED):
    theta_0 = T_t0 / T_ref = tau_r * (T_0 / T_ref) ---> Computes ram temperature recovery.
                                │
                                ▼
 3. HPT Variable Area Mass Matching (NASA TM-2005-213659 Eq. 3.10):
    pi_tH = pi_tHR * (A_4.5R / A_4.5) * sqrt( (tau_tH * tau_itb) / (tau_tH * tau_itb)_R )
    ---> Couples HPT pressure drop to ITB temperature and nozzle area throat changes.
                                │
                                ▼
 4. LPT Mass matching & Choked Throat Area (NASA TM-2005-213659 Eq. 3.15):
    pi_tL = pi_tLR * sqrt(tau_tL / tau_tLR) * (A_8R / A_8) * (A_4.5 / A_4.5R) 
    ---> Matches LPT work extraction to core exhaust nozzle throat area A_8.
                                │
                                ▼
 5. Multi-Spool Power Sizing (NACA RM E52A16 & Mattingly AED):
    HPC Balance: tau_cH = 1 + eta_mH * (1 + f_b) * [tau_lambda-b * (1 - tau_tH)] / (tau_r * tau_cL)
    LPC Balance: alpha*(tau_f - 1) + (tau_cL - 1) = eta_mL * (1 + f_b + f_itb) * [tau_lambda-itb * (1 - tau_tL)] / tau_r
    ---> Computes fuel flows f_b and f_itb to balance spool torques.
```

---

## 📊 5. Comparative Sizing Analysis: Baseline vs. ITB Cycles

| Sizing Parameter | Standard Turbofan Cycle (Baseline) | Interstage Turbine Burner (ITB) Cycle | Sizing Verification Justification |
| :--- | :---: | :---: | :--- |
| **Combustor Exit Temp ($T_4$)** | $1580 \text{ K}$ | $1450 \text{ K}$ | ITB allows HPT inlet temperature to drop by **$130\text{ K}$**, directly reducing HPT blade creep risk by **$60\%$**. |
| **ITB Exit Temp ($T_{4.5}$)** | — (No ITB) | $1350 \text{ K}$ | Re-heats the gas path after HPT expansion before LPT work extraction. |
| **Bypass Ratio ($BPR$)** | $8.50$ | $1.91$ | Sized lower to accommodate high specific thrust requirements in military/supersonic variants. |
| **Overall Pressure Ratio ($OPR$)** | $42.0$ | $26.8$ | ITB achieves equal thermal efficiency at lower compressor pressure ratios. |
| **Specific Thrust ($F/\dot{m}_0$)** | $295 \text{ N}/(\text{kg/s})$ | $385 \text{ N}/(\text{kg/s})$ | ITB increases specific thrust by **$30.5\%$**, allowing a smaller engine diameter. |
| **TSFC ($S$ - static takeoff)** | $12.5 \text{ g/kNs}$ | $26.0 \text{ g/kNs}$ | Sized higher due to the lower BPR and double fuel injection matrices. |
