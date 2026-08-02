# Jet Engine Sizing & Fabrication Updates (update_xx)

This document details the complete set of structural additions and physical equations to be integrated into the jet engine design platform.

---

## 🛠️ Part 1: 3D Geometry & STL Generation (Physical Parts)

We will update the `JetEngineFabrication.Generate` method to add the following components to the generated STL files:

1.  **Blade Twist (Radial Stagger Interpolation)**:
    We will modify the `SdfBladeRow` constructor to take both `staggerHub` and `staggerTip` angles and interpolate them radially inside `fSignedDistance`:
    $$\gamma(r) = \gamma_{\text{hub}} + \frac{r - r_{\text{hub}}}{r_{\text{tip}} - r_{\text{hub}}} \cdot (\gamma_{\text{tip}} - \gamma_{\text{hub}})$$
    This ensures the blades twist aerodynamically from root to tip.
2.  **Hollow Turbine Blade Cooling Cavities (for HPT)**:
    HPT blades will be made hollow by subtracting a scaled-down inner blade row (60% chord, 40% thickness) extending from the disk hub (open root to receive cooling bleed air) to near the tip.
3.  **Blade Platforms, Shrouds, and Squealer Tips**:
    *   **Platforms**: Thin circular disk slices added at the hub radius of HPT and LPT stages.
    *   **Tip Shrouds**: Thin circular shroud rings at the tip of LPT stages.
    *   **Squealer Tips**: Simple geometric tip recesses at the tips of HPT stages.
4.  **Stator Vanes (Outlet Guide Vanes & Interstages)**:
    We will generate stationary stator blade rows (`Jet_Stators.stl`) placed between the rotor stages to redirect the flow.
5.  **Exhaust Struts & Nozzle Exit Plug**:
    Instead of the exhaust cone floating in mid-air, we will add 6 radial structural struts inside `Jet_Nozzle.stl` connecting the nozzle plug to the outer skin.
6.  **Bypass Flow Splitter (Cowl Splitter)**:
    We will add a tapered cylindrical splitter wall inside the Fan casing to separate bypass duct air from the core engine entrance.
7.  **Combustor Fuel Injectors**:
    We will add 12 cylinders distributed circumferentially on the combustor dome to represent fuel nozzles.
8.  **Shaft Support Bearings**:
    We will add front, center, and rear bearing rings around the spools and export them as `Jet_Bearings.stl`.
9.  **Voxel Thickness & Resolution Fixes**:
    *   Scale up the combustor liner and nozzle walls to **$6.0\text{ mm}$** (to prevent PicoGK voxel erosion).
    *   Scale up the casing skins to **$5.0\text{ mm}$** (inner/outer) and set the gyroid lattice period to **$25.0\text{ mm}$** (up from $0.06\text{ mm}$).
    *   Set the inner rotor disk radii to **$25.0\text{ mm}$** (LP shaft) and **$40.0\text{ mm}$** (HP shaft) to close the floating gaps.

---

## 📈 Part 2: 1D/2D Aerothermal & Mechanical Sizing Equations

We will replace the hardcoded heuristics in the validation modules with first-principles physics equations:

1.  **Tapered Blade Centrifugal Stress (Chen et al. paper)**:
    Update `ThermoStructural.AnalyzeAllStages` to apply the taper factor ($C_t = 0.5 + 0.5 \frac{A_{\text{tip}}}{A_{\text{root}}}$) to centrifugal stresses:
    $$\sigma_{\text{cf, root}} = C_t \cdot \rho \omega^2 \left( r_{\text{tip}}^2 - r_{\text{hub}}^2 \right) / 2$$
    *(We use $C_t \approx 0.675$ for compressor stages, and $C_t \approx 0.60$ for turbine stages).*
2.  **Rotating Disk Stresses (Lame-Maxwell Equations)**:
    Calculate the maximum hoop stress ($\sigma_{\theta, \text{max}}$) at the inner bore ($r_{\text{inner}} = r_{\text{shaft}}$) of each stage's disk:
    $$\sigma_{\theta, \text{max}} = \frac{3+\nu}{4} \rho \omega^2 \left( r_{\text{hub}}^2 + \frac{1-\nu}{3+\nu} r_{\text{shaft}}^2 \right)$$
    Ensure this stress does not exceed the disk yield strength (**Gate 4A**).
3.  **Messinger Cowl Anti-Icing Balance (NASA LEWICE paper)**:
    Update `AntiIcingBleed.Evaluate` to solve the mass and energy balance of liquid water droplet impingement:
    $$\dot{Q}_{\text{req}} = h_c (273.15 - T_{\infty}) A_{\text{cowl}} + \dot{m}_{\text{evap}} L_v + \dot{m}_{\text{imp}} c_w (273.15 - T_{\infty}) - q_{\text{kin}} - q_{\text{aero}}$$
    $$\text{Bleed Fraction} = \frac{\dot{Q}_{\text{req}}}{\varepsilon_{\text{hex}} \cdot c_{p, \text{bleed}} \cdot (T_3 - 273.15) \cdot \dot{m}_{\text{core}}}$$
4.  **Flight Installation Drag Sizing (Mattingly Book)**:
    Incorporate inlet spillage drag ($D_{\text{spill}}$) and nozzle boattail drag ($D_{\text{boattail}}$) into `NozzleAero.Evaluate` to determine the Net Installed Thrust.
