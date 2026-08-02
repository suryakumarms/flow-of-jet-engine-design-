# Preliminary Jet Engine Design Compiler — Missing Equations & Physics Audit

This document outlines the high-priority mathematical and physical equations that must be integrated into `JetEngine.cs` to elevate it from a conceptual compiler to a **100% mathematically complete, 1D/2D preliminary design loop**. 

These equations bridge the gap between simple meanline estimations and realistic engineering boundaries, ensuring that aerodynamic shock losses, turbine thermal cooling limits, blade fatigue bending stresses, shaft thrust imbalances, and combustor diffuser limits are accounted for.

---

## 1. Thermodynamic Cycle: Turbine Bleed & Coolant Mixing

### The Physics Gap
At turbine inlet temperatures ($T_4 > 1650 \text{ K}$), single-crystal turbine blades will melt without convective and film cooling. By not modeling bleed air, the cycle solver overestimates specific thrust and underestimates fuel consumption. Coolant extracted from the compressor reduces the mass flow through the burner and degrades the turbine’s enthalpy during downstream mixing.

### Standard 1D Sizing Equations

#### 1. Coolant Effectiveness ($\eta_{\text{cool}}$)
We compute the required cooling effectiveness based on local gas temperature ($T_{g, \text{rel}}$), maximum allowable metal temperature ($T_{\text{metal}}$), and compressor coolant air temperature ($T_3$):
$$\eta_{\text{cool}} = \frac{T_{g, \text{rel}} - T_{\text{metal}}}{T_{\text{metal}} - T_3}$$

#### 2. Coolant Mass Flow Fraction ($\varepsilon_{\text{cool}}$)
The mass flow fraction ($\varepsilon_{\text{cool}} = \dot{m}_{\text{cool}} / \dot{m}_{\text{core}}$) is calculated using a semi-empirical technology factor ($C_{\text{tech}} \approx 0.05$ to $0.08$ for convective/film-cooled blades):
$$\varepsilon_{\text{cool}} = C_{\text{tech}} \cdot \frac{\eta_{\text{cool}}}{1 - \eta_{\text{cool}}}$$

#### 3. Coolant Enthalpy Mixing at Stage Exit
The coolant air mixes back into the flowpath at the turbine blade trailing edge, dropping the mixed-out gas enthalpy ($h_{45}$) for the next stage:
$$h_{45} = (1 - \varepsilon_{\text{cool}}) \cdot h_4 + \varepsilon_{\text{cool}} \cdot h_3$$
The mixed-out temperature $T_{45}$ is determined by back-calculating from the mixed-out enthalpy using the variable specific heat lookup:
$$T_{45} = \text{Lookup}(h_{45}, \gamma)$$

### C# Integration Blueprint
1. In `class MissionRequirements`, add fields for `Double MaxMetalTemp_K` (e.g., $1250 \text{ K}$) and `Double TechFactor_C` (e.g., $0.06$).
2. In `BraytonCycleSolver.Solve`, inside the turbine expansion loop, calculate the local cooling mass fraction $\varepsilon_{\text{cool}}$ for the first HPT stage.
3. Subtract $\varepsilon_{\text{cool}} \cdot \dot{m}_{\text{core}}$ from the compressor exit mass flow, and perform the enthalpy mixing equation prior to the turbine stage expansion calculation.

---

## 2. Aerodynamics: Supersonic Blade Tip Mach & Shock Losses

### The Physics Gap
Compressor and fan blades twist outward from the hub to the tip. Because the rotational speed $U = \omega \cdot r$ increases linearly with radius, the blade tips experience high relative Mach numbers. If the tip speed exceeds Mach 1.2, supersonic shock waves form, creating severe shock losses that decrease stage efficiency.

```
                  V_1r (Relative Tip Velocity)
                     /|
                    / |
                   /  | V_z (Axial Velocity)
                  /   |
                 /____|
              U_tip = ω · r_tip (Blade Tangential Velocity)
```

### Standard 1D Sizing Equations

#### 1. Tip Relative Inlet Velocity ($V_{1r, \text{tip}}$)
Calculated using the axial core velocity ($V_z$) and rotational velocity at the blade tip ($U_{\text{tip}}$):
$$U_{\text{tip}} = \omega \cdot r_{\text{tip}}$$
$$V_{1r, \text{tip}} = \sqrt{V_z^2 + (U_{\text{tip}} - V_{\theta 1})^2}$$

#### 2. Tip Relative Mach Number ($M_{1r, \text{tip}}$)
Using the local static speed of sound ($a_1 = \sqrt{\gamma R T_1}$):
$$M_{1r, \text{tip}} = \frac{V_{1r, \text{tip}}}{\sqrt{\gamma R T_1}}$$

#### 3. Shock Loss Correction ($\Delta \eta_{\text{shock}}$)
If $M_{1r, \text{tip}} > 1.0$, reduce the stage isentropic efficiency by a shock loss coefficient ($\varpi_{\text{shock}}$):
$$\Delta \eta_{\text{shock}} = 0.08 \cdot \left(M_{1r, \text{tip}} - 1.0\right)^{1.5}$$
$$\eta_{\text{stage, effective}} = \eta_{\text{stage, design}} - \Delta \eta_{\text{shock}}$$

### C# Integration Blueprint
1. In `class BladeStage`, add fields for `HubRadius` and `TipRadius`.
2. In `FlowPathGenerator.Generate`, calculate the tip radius: `TipRadius = HubRadius + Height`.
3. In `AeroValidator.ValidateBlades`, calculate $M_{1r, \text{tip}}$. If it exceeds $1.0$, log a warning and dynamically subtract $\Delta \eta_{\text{shock}}$ from the stage polytropic efficiency, forcing the cycle solver to re-converge.

---

## 3. Structural: Aerodynamic Gas Bending Stress

### The Physics Gap
The structural validator in `JetEngine.cs` only audits centrifugal stresses. In reality, the blades act as wings that deflect high-momentum gas. This deflection creates huge aerodynamic lift and drag forces that bend the blade forward and circumferentially. This bending stress is the primary driver of **High-Cycle Fatigue (HCF)**.

### Standard 1D Sizing Equations

#### 1. Tangential Aerodynamic Force per Blade ($F_t$)
Derived from the change in gas tangential momentum across a single blade row with $N_{\text{blades}}$:
$$F_t = \frac{\dot{m}_{\text{core}} \cdot \left(V_{\theta 1} - V_{\theta 2}\right)}{N_{\text{blades}}}$$

#### 2. Root Bending Moment ($M_b$)
Assuming the aerodynamic force acts at the blade's radial centroid ($h_{\text{centroid}} \approx \text{Height} / 2$):
$$M_b = F_t \cdot \frac{h}{2}$$

#### 3. Aerodynamic Bending Stress ($\sigma_b$)
Calculated at the root cross-section using the section modulus ($Z_{xx}$):
$$Z_{xx} = \frac{C_{\text{chord}} \cdot t_{\text{max}}^2}{10}$$
$$\sigma_b = \frac{M_b}{Z_{xx}} = \frac{5 \cdot F_t \cdot h}{C_{\text{chord}} \cdot t_{\text{max}}^2}$$

#### 4. Total Structural Combined Stress ($\sigma_{\text{total}}$)
Combined tension (centrifugal) and bending (aerodynamic) stress at the blade root:
$$\sigma_{\text{total}} = \sigma_{\text{centrifugal}} + \sigma_{\text{bending}}$$
$$\text{Safety Factor} = \frac{\sigma_{\text{yield}}(T)}{\sigma_{\text{total}}} \ge 1.5$$

### C# Integration Blueprint
1. In `ThermoStructural.AnalyzeStage`, pull the tangential gas velocities $V_{\theta 1}$ and $V_{\theta 2}$ from the velocity triangle calculations.
2. Implement the section modulus $Z_{xx}$ based on chord length and maximum thickness.
3. Compute $\sigma_{\text{bending}}$ and add it to the centrifugal stress prior to checking against the material's yield strength.

---

## 4. Mechanical: Axial Shaft Thrust Balancing

### The Physics Gap
A multi-spool jet engine experiences severe axial force imbalances. The compressor stages push the air backward, generating a massive forward force on the shaft. Conversely, the turbine stages generate a rearward force. The net difference ($F_{\text{net}}$) is borne by the thrust bearings. If $F_{\text{net}}$ is too high, the bearings will experience friction failure and seize.

### Standard 1D Sizing Equations

#### 1. Rotor Axial Gas Force ($F_{\text{gas}}$)
Calculated by integrating the pressure change across the rotor blades and disks:
$$F_{\text{gas}} = \sum_{i=1}^{\text{stages}} \left[ \dot{m} \left(V_{z1} - V_{z2}\right) + \left(p_1 - p_2\right) \cdot A_{\text{annulus}} \right]$$

#### 2. Shaft Net Axial Force ($F_{\text{net}}$)
For a single spool (e.g., HP spool):
$$F_{\text{net, HP}} = F_{\text{gas, HP Compressor}} - F_{\text{gas, HP Turbine}}$$

#### 3. Balance Piston Offset
To balance the shaft, designers vent high-pressure compressor bleed air to a cavity behind a compressor disk to create a counter-balancing force:
$$F_{\text{balance}} = \Delta p_{\text{cavity}} \cdot A_{\text{disk face}}$$
$$F_{\text{bearing, net}} = F_{\text{net, HP}} - F_{\text{balance}} \le F_{\text{bearing, limit}}$$

### C# Integration Blueprint
1. Create a `static class ShaftMechanicals` that iterates through all compressor and turbine stages on a spool.
2. Integrate the dynamic pressure ($p$) and annulus area ($A$) to compute $F_{\text{gas}}$.
3. Track the bearing net force and output the required area of the balance piston back to the flowpath compiler.

---

## 5. Combustion: Diffuser Expansion & Blowout Limits

### The Physics Gap
Air leaving the compressor flows at speeds of $120$ to $150 \text{ m/s}$ (Mach $0.35$). If this air directly enters the combustor dome, it will instantly blow out the flame, as the localized flow speed exceeds the chemical flame speed of kerosene ($0.5 \text{ m/s}$). Sizing must model a diffuser to slow the flow down.

### Standard 1D Sizing Equations

#### 1. Diffuser Area Ratio ($AR$)
To slow the flow speed from $V_3$ down to a reference combustor inlet speed ($V_{\text{ref}} \approx 20 \text{ m/s}$), size the diffuser exit-to-inlet area ratio:
$$AR = \frac{A_{\text{combustor inlet}}}{A_{\text{compressor exit}}} = \frac{V_3}{V_{\text{ref}}}$$

#### 2. Diffuser Pressure Loss ($\Delta p_{\text{diffuser}}$)
Models the stagnation pressure drop as a function of the diffuser expansion angle ($\theta_{\text{diffuser}} \approx 7^\circ$ to prevent boundary layer separation):
$$\Delta p_{\text{diffuser}} = C_{\text{loss}} \cdot \left( \frac{1}{2} \rho V_3^2 \right) \cdot \left(1 - \frac{1}{AR}\right)^2$$

### C# Integration Blueprint
1. Inside `CombustorDesign.Design`, calculate the compressor exit velocity ($V_3$).
2. Size the diffuser expansion geometry and compute the diffuser pressure drop.
3. Subtract the diffuser pressure drop from the combustor inlet pressure $P_3$ before passing the thermodynamic state into the cycle solver.
