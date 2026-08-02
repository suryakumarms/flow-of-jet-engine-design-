# NASA Jet Engine Design Workflow - Library Prioritization Report

This report ranks and evaluates the 20 open-source software libraries integrated into the MDAO (Multidisciplinary Design Optimization) jet engine design loop, as defined in [Library_index.md](file:///C:/Users/suryakumar%20M%20S/Desktop/jet_engine_resources/Library_index.md). 

The ranking is based on their physical criticalities, execution costs, topological impacts, and mathematical coupling within the 20-Gate NASA Jet Engine Systems Engineering pipeline.

---

## 🥇 High-Priority Libraries

These libraries form the core physical and mathematical solvers of the MDAO pipeline. Any failures, instability, or approximations in these libraries directly derail cycle thermodynamics, aerodynamic efficiency, structural safety, or spool dynamics.

### 1. NASA pyCycle
*   **Category:** Thermodynamics & Cycle Analysis
*   **Purpose:** 0D gas turbine thermodynamic cycle solver.
*   **Governing Physics & Equations:** 
    *   One-dimensional steady aerothermodynamics of turbomachinery components.
    *   Conservation of mass, momentum, and energy across stations (Brayton Cycle).
    *   Enthalpy, entropy, and specific heat calculations based on gas tables.
*   **Why High Priority:** pyCycle defines the baseline thermodynamic parameters ($T_4$, pressure ratios, bleed fractions, mass flows). All downstream geometry, CFD, and FEA models depend on boundary conditions derived from pyCycle. It is the primary computational gatekeeper at **Gate 1**.

### 2. NASA turbo-design
*   **Category:** Throughflow & Meridional Flowpath Sizing
*   **Purpose:** Streamline throughflow turbomachinery solver.
*   **Governing Physics & Equations:** 
    *   Radial equilibrium equations:
        $$\frac{1}{\rho}\frac{dP}{dr} = \frac{V_\theta^2}{r} - V_m \frac{dV_m}{dr}$$
    *   Euler's turbine equation:
        $$\Delta h_0 = U_2 V_{\theta 2} - U_1 V_{\theta 1}$$
*   **Why High Priority:** Bridges the massive gap between 0D thermodynamics and 3D blade profiling. It determines the flowpath boundaries, blade count, stage loading, and blade-row velocity triangles. Without it, airfoil coordinate lofting has no physical basis.

### 3. ParaBlade
*   **Category:** Blade Parametric Generation
*   **Purpose:** Generates parametric 3D airfoil surfaces for fans, compressors, and turbines.
*   **Governing Physics & Equations:** 
    *   NURBS (Non-Uniform Rational B-Splines) surface lofting, camber line distributions, and thickness profiling (NACA/double-circular arc profiles).
*   **Why High Priority:** ParaBlade translates aerothermal streamflow targets into physical blade shapes. The surface contours generated here are directly subjected to aerodynamic CFD (SU2) and structural stress checks (NASTRAN).

### 4. Cantera
*   **Category:** Combustion Kinetics & Stoichiometry
*   **Purpose:** Chemical kinetics, thermodynamics, and transport solver.
*   **Governing Physics & Equations:** 
    *   Reacting flow chemistry, Gibbs free energy minimization, and stoichiometric mass action laws:
        $$r_i = k_f \prod [C_j]^{\nu'_j} - k_r \prod [C_j]^{\nu''_j}$$
*   **Why High Priority:** Governs the combustor emissions loop (**Gate 3C**). It feeds chemical kinetics and stoichiometry parameters back into `pyCycle` to adjust fuel-air ratio schedules to meet CAEP/8 emissions limits.

### 5. SU2
*   **Category:** Aerodynamics & Compressible Flow CFD
*   **Purpose:** Multi-physics compressible Navier-Stokes solver.
*   **Governing Physics & Equations:** 
    *   Compressible RANS (Reynolds-Averaged Navier-Stokes) equations coupled with Menter's Shear Stress Transport (SST) turbulence model:
        $$\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot \mathbf{F}_c - \nabla \cdot \mathbf{F}_v = \mathbf{Q}$$
*   **Why High Priority:** Evaluates blade stall margins, cascade losses, and overall aerodynamic efficiency (**Gate 3A** and **Gate 3H**). SU2 CFD is highly expensive but essential to prove that blades do not stall or exhibit flow separation.

### 6. OpenFOAM
*   **Category:** Reacting CFD & Conjugate Heat Transfer (CHT)
*   **Purpose:** Reacting flow CFD and thermal boundary solver.
*   **Governing Physics & Equations:** 
    *   Reacting Navier-Stokes, combustion species transport, and conjugate heat transfer solid-fluid coupling:
        $$\rho C_p \frac{\partial T}{\partial t} + \nabla \cdot (\rho C_p \mathbf{u} T) = \nabla \cdot (k \nabla T) + S_h$$
*   **Why High Priority:** OpenFOAM handles combustor thermal patterning (**Gate 3B**) and conjugated heat transfer to determine the temperature map of turbine blades. This heat map is critical for creep analysis (**Gate 4A**).

### 7. ROSS
*   **Category:** Shaft Rotordynamics
*   **Purpose:** Flexible rotor assemblies dynamic behavior.
*   **Governing Physics & Equations:** 
    *   Timoshenko beam element formulations, gyroscopic moment coupling, and bearing stiffness matrices:
        $$\mathbf{M}\ddot{\mathbf{q}} + (\mathbf{C} + \mathbf{G})\dot{\mathbf{q}} + \mathbf{K}\mathbf{q} = \mathbf{f}(t)$$
*   **Why High Priority:** Validates high-speed shafts against critical whirl speeds (**Gate 4B**). Shaft resonances can literally tear an engine apart, making Campbell diagram analysis mandatory.

### 8. OpenTurbofanArchitecting
*   **Category:** Mechanical Transmission & Gearbox Sizing
*   **Purpose:** Sizing of power gearboxes and engine architectures.
*   **Governing Physics & Equations:** 
    *   Planetary gear tooth friction losses, mechanical efficiencies, and thermal heat dissipation:
        $$\dot{Q}_{lube} = P_{in} (1 - \eta_{gb})$$
*   **Why High Priority:** Sizes the planetary gearbox for geared turbofan layouts. Directly impacts LPT speed optimizations, fan speeds, and oil cooling thermal loads (**Gate 4D**).

---

## 🥈 Medium-Priority Libraries

These libraries act as geometric lofting interfaces, property databases, or auxiliary subsystem sizing tools. While important, their failures do not violate core physics but rather geometric boundaries or hydraulic sub-loops.

### 1. NASA Aviary
*   **Category:** Aircraft Sizing & Weight Estimation
*   **Purpose:** Evaluates engine installation inside flight missions.
*   **Physics/Equations:** Empirical weight correlations and flight path integration.
*   **Reasoning:** Governs the top-level mission matching (**Gate 5D**). While vital for final system closure, the engine details are treated largely as black-box performance decks during early sizing stages.

### 2. NASA PyTurbo
*   **Category:** Geometry Transformations
*   **Purpose:** 2D airfoil sections projection into 3D space.
*   **Physics/Equations:** Coordinate systems transformations, axial/tangential offsets.
*   **Reasoning:** Performs the coordinate lofts. Its geometric transformations must align with PicoGK solid models, but it does not run active physical solves.

### 3. PicoGK
*   **Category:** Computational Geometry (Voxel Kernel)
*   **Purpose:** Procedural 3D voxel engine.
*   **Physics/Equations:** Implicit voxel sign-distance field (SDF) algorithms and boolean operations.
*   **Reasoning:** Highly critical for generating watertight Solid geometry, internal cooling channels, and casing. However, it is a geometric modeling tool rather than a physical solver.

### 4. OpenVSP
*   **Category:** Nacelle & Aircraft Geometry
*   **Purpose:** Sizing of outer aircraft geometries (nacelle, inlets, nozzles).
*   **Physics/Equations:** Parametric surface generation, area rule profiling.
*   **Reasoning:** Essential for sizing nacelle installation drag and blocker doors, but relies on downstream solvers (SU2, NASTRAN) for physical validation.

### 5. CoolProp
*   **Category:** Fluid Properties
*   **Purpose:** Equation of State (EOS) database.
*   **Physics/Equations:** Helmholtz energy formulations for thermodynamic states.
*   **Reasoning:** Serves as a utility library providing thermodynamic values for fuel loops and gearbox oil properties.

### 6. TESPy
*   **Category:** Systems Sizing
*   **Purpose:** Hydraulic networks and thermal balance.
*   **Physics/Equations:** Matrix-based network flow solver, pressure losses, mass balances.
*   **Reasoning:** Models the secondary air system (SAS) and gearbox oil lube circuits. Secondary loop balances depend on boundaries set by pyCycle.

### 7. libAcoustics
*   **Category:** Aeroacoustics
*   **Purpose:** Far-field noise estimation.
*   **Physics/Equations:** Lighthill's acoustic analogy and Ffowcs Williams-Hawkings (FW-H) equation.
*   **Reasoning:** Validates engine compliance with ICAO Chapter 14 noise standards (**Gate 5B**). A vital certification step, but secondary to the structural and aerothermal integrity of the engine itself.

### 8. SMT
*   **Category:** Surrogate Modeling
*   **Purpose:** Kriging and Radial Basis Function (RBF) interpolators.
*   **Physics/Equations:** Bayesian Gaussian process models and multivariate regression.
*   **Reasoning:** Speeds up optimization by bypassing CFD/FEA solvers. Essential for MDAO efficiency, but the final designs must still be verified with high-fidelity solvers.

---

## 🥉 Low-Priority Libraries

These libraries are either legacy offline portals, pipeline orchestrators, or non-destructive tracking systems. Their presence is structural or procedural rather than physical.

### 1. NASA LEWICE3D
*   **Category:** Icing & Meteorological Physics
*   **Purpose:** Ice accretion prediction under supercooled conditions.
*   **Physics/Equations:** Eulerian/Lagrangian droplet trajectories and Messinger heat balance.
*   **Reasoning:** Crucial for FAA flight safety certification, but icing constraints (**Gate 3E**) are treated as localized edge-case checks rather than core thermodynamic sizing factors.

### 2. COSMIC NASTRAN (NASTRAN-95)
*   **Category:** Structural Finite Element Analysis (FEA)
*   **Purpose:** Linear structural, thermal, and modal analyses.
*   **Physics/Equations:** Elastic stiffness equations:
        $$\mathbf{K}\mathbf{u} = \mathbf{f}$$
*   **Reasoning:** Though structurally critical, the open-source COSMIC NASTRAN (released in 1995) is structurally outdated and suffers from solver limits, lack of modern contact interfaces, and non-linear creep integration. For production FEA, modern commercial solvers or CalculiX/Code_Aster are used.

### 3. Prefect
*   **Category:** Orchestration
*   **Purpose:** Coordinates the DAG execution of solvers.
*   **Physics/Equations:** None.
*   **Reasoning:** Strictly a workflow orchestration layer. Failure in Prefect halts the pipeline execution but does not impact the design physics.

### 4. MLflow
*   **Category:** Experiment Tracking
*   **Purpose:** Tracks MDAO parameter sweeps and registers surrogate models.
*   **Physics/Equations:** None.
*   **Reasoning:** Solely a data logging and version control interface.

