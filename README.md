# Flow of Jet Engine Design

A comprehensive, physics-informed computational framework and implicit geometry modeling engine for modern jet engine design, analysis, and simulation. Built in **C# (.NET 10.0)** leveraging the **LEAP71 / PicoGK** implicit modeling stack, alongside a **Python Simulation Backend**.

---

## 🌟 Architecture Overview

```
flow-of-jet-engine-design/
├── src/                          # Main C# Source Code (.NET 10.0 Project)
│   ├── JetEngine.csproj          # C# Project File
│   ├── Program.cs                # Entry Point
│   ├── Physics/                  # Aerothermal & Thermodynamic Physics Modules
│   │   ├── MissionAndAtmosphere.cs
│   │   ├── BraytonCycle.cs
│   │   ├── FluidAeroDynamics.cs
│   │   └── Thermostructural.cs
│   ├── Components/               # Engine Turbomachinery & Hardware Components
│   │   ├── Fan.cs
│   │   ├── Compressor.cs
│   │   ├── Combustor.cs
│   │   ├── Turbine.cs
│   │   ├── CasingAndMounts.cs
│   │   └── ShaftsAndBearings.cs
│   ├── Systems/                  # Systems, Controls, Geometry & Analysis
│   │   ├── FlowPath.cs
│   │   ├── EngineSystems.cs
│   │   ├── ImplicitShapes.cs
│   │   ├── AnalysisAndSimulations.cs
│   │   ├── ControlAndValidation.cs
│   │   ├── Fabrication.cs
│   │   └── ValidationGates.cs
│   └── Legacy/                   # Monolithic Snapshots & Draft Variations
├── backend/                      # Python Simulation Backend Server
│   └── server.py                 # FastAPI / Flask Simulation API Server
├── lib/                          # External Dependencies & Framework Integrations
│   └── SAM26_V2/                 # PicoGK & LEAP71 Geometry Libraries
├── docs/                         # Documentation, Blueprints & Literature
│   ├── blueprint/                # Core Architecture & System Flow Documentation
│   ├── audit_and_reports/        # Codebase Audits & Literature Analysis
│   └── literature/               # Technical Papers, Books & NASA Reports (PDFs)
├── media/                        # Presentations, Visualizations & Screenshots
└── data/                         # Simulation Logs & Output Datasets
```

---

## 🚀 Key Features & Modules

- **Thermodynamic Cycle & Brayton Analysis**: Multi-spool cycle analysis, flight envelope matching, mission profile analysis.
- **Turbomachinery Modeling**: Axial & centrifugal compressor stages, fan design, combustor thermochemistry, high/low pressure turbines.
- **Computational Fluid Dynamics & Aerodynamics**: Fluid aerodynamics, boundary layer integration, shock wave calculations.
- **Thermostructural & Rotordynamics**: Thermal stress analysis, shaft stiffness, bearing reaction forces, modal frequency sweeps.
- **Implicit Geometry Generation**: Computational geometry generation integrated with LEAP71 ShapeKernel and PicoGK voxel kernel for additive manufacturing (3D printing).
- **Control & Validation**: FADEC system emulation, health monitoring, validation gates, automated simulation sweeps.

---

## 🛠️ Getting Started

### Prerequisites

- [.NET 10.0 SDK](https://dotnet.microsoft.com/download)
- [Python 3.10+](https://www.python.org/downloads/) (for simulation backend server)

### Building the Project

To build the main C# project using .NET CLI:

```bash
dotnet build src/JetEngine.csproj
```

To run the jet engine design simulation pipeline:

```bash
dotnet run --project src/JetEngine.csproj
```

### Running the Python Backend

```bash
cd backend
python server.py
```

---

## 📄 License & References

Refer to `docs/literature/` for technical literature and reference material cited throughout the project.
