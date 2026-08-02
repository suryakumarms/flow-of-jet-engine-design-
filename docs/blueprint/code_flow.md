# 🌀 AeroSync Jet Engine Codebase Connection Map

Below is the dependency map and architecture design of the **AeroSync Jet Engine Workspace**. It outlines the relationship between the modular C# files, how they share data, and their specific responsibilities in the thermodynamic, mechanical, aerodynamic, and structural design loops.

---

## 🗺️ System Flowchart

This flowchart illustrates how data starts at the command-line orchestrator and flows through the physical calculation models down to optimization and fabrication.

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

