# JetEngine V6 Computational Design Platform: Workflow Chart

This document maps out the execution flow, physical solver modules, and the multi-disciplinary optimization (MDAO) closed-loop feedback design system implemented in [JetEngine V6.cs](file:///c:/Users/suryakumar%20M%20S/Downloads/Jet%20(2)/Jet/JetEngine%20V6.cs).

---

## 🗺️ Master Iterative Design Flowchart

The following flowchart shows the iterative, closed-loop execution inside `ClosedLoopDesigner.DesignEngine(...)`. If any gate or physical constraint fails, it dynamically adjusts the design parameters (such as BPR, OPR, or TIT) and loops back to **Gate 1** to re-solve the thermodynamic cycle.

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

## ⚙️ Detailed Phase-by-Phase Description

### 1. Thermodynamic & Aerodynamic Sizing (Gate 1 & 2)
*   **Station Properties Sizing:** The master `MissionRequirements` dictate the entry limits (Thrust, Mach, Altitude).
*   **Brayton Cycle Solver:** Solves the unmixed turbofan cycle from station 0 (ambient) through station 18 (nozzles). Incorporates variables such as:
    *   **Evaporative water/methanol injection** cooling at the inlet (Station 2).
    *   **Turbine cooling bleed mixing** (HPT coolant fraction extracted at Station 3 and re-injected at Station 45, dropping mixed-out gas enthalpy).
    *   **Supersonic shock loss correction** on the fan blade tip ($M_{1r,tip} > 1.0$) which drops fan stage polytropic efficiency.

### 2. Blade Geometry & Aerodynamic Verification (Gate 3A & 3B)
*   **Flow Path Generation:** Translates thermodynamic station volumes and RPM constraints into hub/tip radii profiles and axial stage blade sections.
*   **Aero Checks:** Calculates flow angles, velocity triangles, Diffusion Factors (must be $\le 0.45$), and De Haller numbers (must be $\ge 0.72$) to prevent local flow separation and aerodynamic stall.
*   **Combustor Design:** Applies Lefebvre combustor correlations to design the primary, intermediate, and dilution zones, balancing loading, pattern factor, and emissions indexes.

### 3. Structural, Vibration, and Rotordynamics (Gate 4A & 4B)
*   **Thermo-Structural FEA:** Solves centrifugal stress ($\sigma_{cf} = \rho \omega^2 \int r\,dr$) and thermal gradient stresses. Validates against the material's yield strength and Larson-Miller creep parameter (LMP) life boundaries.
*   **Rotor Critical Speeds:** Timoshenko beam solver calculates the critical whirl speeds of the High-Pressure (HP) and Low-Pressure (LP) shafts. Employs Campbell diagrams to verify a 15% clearance margin.
*   **Axial Thrust Balancing:** Resolves stage-by-stage pressure loads to compute net axial shaft thrust, sizing the balance piston to protect thrust bearings from overloading.

### 4. Thermal Systems & transient control (Gate 4D, 5C & 5E)
*   **Diffuser & Blowout Sizing:** Models diffuser pressure recovery and flags flame blowout risk if reference velocity exceeds limits.
*   **Gearbox Lube Oil Thermal Balance:** Computes heat dissipation from gears and bearings, verifying oil temperature is within material constraints (under the limit of Air-Cooled & Fuel-Cooled Oil Coolers).
*   **Spool Transient Acceleration:** Integrates spool polar moment of inertia to model throttle slam (idle $\to$ takeoff).
*   **Thrust Reverser & Landing:** Estimates dry runway stopping distance with bypass thrust reversal.

### 5. DMLS Manufacturability (Gate 6)
*   Verifies additive manufacturing (DMLS) requirements: wall thickness constraints, overhang limits, powder removal accessibility, and residual thermal stresses.

---

## 🛠️ Validation Run Execution Flow

When running `dotnet run jet_validate`, the platform executes a sequential diagnostic check across all gates and auxiliary simulation layers:

```mermaid
sequenceDiagram
    autonumber
    actor CLI as Developer (dotnet run jet_validate)
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

