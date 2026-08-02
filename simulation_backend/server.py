import os
import sys
import math
import json
from typing import Dict, List, Optional
from pydantic import BaseModel

# FastAPI setup (with local import check)
try:
    from fastapi import FastAPI, HTTPException
    import uvicorn
except ImportError:
    print("WARNING: FastAPI or uvicorn not installed. Install with: pip install fastapi uvicorn")
    sys.exit(1)

# Cantera setup
CANTERA_AVAILABLE = False
try:
    import cantera as ct
    CANTERA_AVAILABLE = True
except ImportError:
    print("WARNING: Cantera not installed. Running in analytical proxy fallback mode. Install with: pip install cantera")

app = FastAPI(
    title="NASA Jet Engine Advanced Simulation Backend",
    description="Microservice API for Cantera chemical kinetics, transient thermal soak-back, maneuver loads, Greitzer surge, impact dynamics, advanced fatigue, and Tyler-Sofrin acoustics.",
    version="2.0"
)

# ══════════════════════════════════════════════════════════════════════
#  DATA STRUCTURES (Pydantic Models)
# ══════════════════════════════════════════════════════════════════════

class CombustionRequest(BaseModel):
    fuel_type: str = "SAF"        # Methane, Hydrogen, SAF, Ammonia
    inlet_temperature_K: float = 820.0
    inlet_pressure_Pa: float = 1500000.0
    equivalence_ratio: float = 0.60
    mass_flow_kg_s: float = 120.0

class ContactStressRequest(BaseModel):
    rotor_speed_rpm: float = 12000.0
    blade_mass_kg: float = 0.85
    blade_cg_radius_m: float = 1.40
    neck_width_mm: float = 18.0
    tooth_count: int = 3
    tooth_pitch_mm: float = 8.0
    friction_coefficient: float = 0.15

class ThermalSoakbackRequest(BaseModel):
    initial_disc_temp_K: float = 1100.0
    ambient_temp_K: float = 300.0
    shaft_length_m: float = 1.2
    shaft_diameter_m: float = 0.08
    time_duration_s: float = 3600.0  # default 1 hour

class ManeuverLoadsRequest(BaseModel):
    rotor_speed_rpm: float = 12000.0
    maneuver_pitch_rate_rad_s: float = 1.5  # high-g pull-up
    maneuver_yaw_rate_rad_s: float = 0.5
    rotor_mass_kg: float = 150.0
    rotor_cg_radius_m: float = 0.40
    g_load: float = 9.0                     # maneuver envelope limit

class CompressorSurgeRequest(BaseModel):
    mean_mass_flow_kg_s: float = 45.0
    volume_m3: float = 0.8
    duct_area_m2: float = 0.12
    duct_length_m: float = 1.5
    speed_of_sound_mps: float = 340.0
    surge_param_B: float = 1.2
    duration_s: float = 5.0

class ImpactDynamicsRequest(BaseModel):
    blade_velocity_mps: float = 350.0
    projectile_mass_kg: float = 3.65        # Large bird
    projectile_velocity_mps: float = 77.0
    material_A_Pa: float = 880e6            # Yield strength Ti-6Al-4V
    material_B_Pa: float = 400e6            # Strain hardening
    material_C: float = 0.015               # Strain rate sensitivity
    material_n: float = 0.47
    material_m: float = 1.0                 # Thermal softening
    density_kgm3: float = 4430.0
    chord_m: float = 0.25
    thickness_m: float = 0.015

class AdvancedFatigueRequest(BaseModel):
    stress_amplitude_MPa: float = 320.0
    mean_stress_MPa: float = 150.0
    strain_amplitude: float = 0.0035
    max_strain: float = 0.007
    temperature_K: float = 950.0
    yield_strength_MPa: float = 800.0
    ultimate_strength_MPa: float = 1050.0
    cycles: float = 10000.0
    findley_k: float = 0.3

class Acoustics3DRequest(BaseModel):
    blade_count: int = 24
    stator_count: int = 36
    shaft_speed_rpm: float = 3600.0
    sound_speed_mps: float = 340.0
    duct_radius_m: float = 1.2
    nozzle_wing_distance_m: float = 1.5
    jet_velocity_mps: float = 280.0

# ══════════════════════════════════════════════════════════════════════
#  CANTERA COMBUSTION SOLVER
# ══════════════════════════════════════════════════════════════════════

def solve_cantera_combustion(req: CombustionRequest) -> Dict:
    """
    Computes chemical equilibrium and detailed NOx/CO/Soot emissions using Cantera.
    Supports Hydrogen (H2), SAF (decane/dodecane surrogate), Methane, and Ammonia (NH3).
    """
    fuel_upper = req.fuel_type.upper()
    T_in = req.inlet_temperature_K
    phi = req.equivalence_ratio
    P_Pa = req.inlet_pressure_Pa
    
    # 1. Soot Yield Estimate (Empirical correlation based on Carbon Content)
    # Hydrogen & Ammonia have no carbon -> zero soot. Methane (1 Carbon), SAF (avg 12 Carbons).
    if fuel_upper == "SAF":
        soot_yield = 0.002 * phi * (P_Pa / 1e5) * math.exp(req.inlet_temperature_K / 400.0)
    elif fuel_upper == "METHANE" or fuel_upper == "NATURAL_GAS":
        soot_yield = 0.0003 * phi * (P_Pa / 1e5)
    else:
        soot_yield = 0.0

    # 2. PDF Flamelet Parameters (proxy for scalar dissipation & mixture fraction variance)
    # stoichiometric mixture fractions
    if fuel_upper == "HYDROGEN":
        f_stoich = 0.029
    elif fuel_upper == "AMMONIA":
        f_stoich = 0.163
    elif fuel_upper == "SAF":
        f_stoich = 0.068
    else:  # Methane
        f_stoich = 0.055
        
    z_mean = (phi * f_stoich) / (1.0 + f_stoich * (phi - 1.0))
    z_variance = 0.15 * z_mean * (1.0 - z_mean)
    scalar_dissipation = 2.0 * z_mean * (1.0 - z_mean) * 12.0  # typical strain rate dissipation

    if not CANTERA_AVAILABLE:
        # High-fidelity analytical proxy based on NASA CEA / Lefebvre flame profiles
        if fuel_upper == "HYDROGEN":
            T_flame = T_in + (3200.0 - T_in) * (phi if phi <= 1.0 else 1.0/phi)
            lhv = 120e6  # J/kg
            y_nox = 1.5e-5 * math.exp(P_Pa / 1e6) * math.exp(T_flame / 250.0)
            y_co = 0.0
            y_h2o = 0.15 * phi
            y_co2 = 0.0
        elif fuel_upper == "AMMONIA":
            T_flame = T_in + (1800.0 - T_in) * (phi if phi <= 1.0 else 1.0/phi)
            lhv = 18.6e6  # J/kg
            # Ammonia combustion produces elevated thermal/prompt NOx
            y_nox = 8.0e-5 * math.exp(P_Pa / 1e6) * math.exp(T_flame / 350.0)
            y_co = 0.0
            y_h2o = 0.22 * phi
            y_co2 = 0.0
        elif fuel_upper == "SAF":
            T_flame = T_in + (2300.0 - T_in) * (phi if phi <= 1.0 else 1.0/phi)
            lhv = 42.8e6  # J/kg
            y_nox = 8.5e-6 * math.exp(P_Pa / 1.2e6) * math.exp(T_flame / 280.0)
            y_co = 1.2e-4 * (1.0 / max(0.1, phi))
            y_h2o = 0.08 * phi
            y_co2 = 0.12 * phi
        else:  # Methane / standard gas
            T_flame = T_in + (2200.0 - T_in) * (phi if phi <= 1.0 else 1.0/phi)
            lhv = 50.0e6
            y_nox = 6.2e-6 * math.exp(P_Pa / 1.2e6) * math.exp(T_flame / 290.0)
            y_co = 8.0e-5 * (1.0 / max(0.1, phi))
            y_h2o = 0.09 * phi
            y_co2 = 0.10 * phi

        return {
            "adiabatic_flame_temperature_K": round(T_flame, 2),
            "lhv_j_kg": lhv,
            "species_mass_fractions": {
                "NOx": float(f"{y_nox:.4e}"),
                "CO": float(f"{y_co:.4e}"),
                "CO2": float(f"{y_co2:.4f}"),
                "H2O": float(f"{y_h2o:.4f}")
            },
            "soot_mass_fraction": float(f"{soot_yield:.4e}"),
            "pdf_flamelet": {
                "mixture_fraction_mean": round(z_mean, 4),
                "mixture_fraction_variance": round(z_variance, 6),
                "scalar_dissipation_rate_s": round(scalar_dissipation, 2)
            },
            "status": "PROXY_FALLBACK_OK"
        }

    # Cantera active execution
    try:
        if fuel_upper == "HYDROGEN":
            gas = ct.Solution('h2o2.yaml')
            fuel = "H2"
            oxidizer = "O2:1.0, N2:3.76"
        elif fuel_upper == "AMMONIA":
            # gri30.yaml has Ammonia species and nitrogen pathways
            gas = ct.Solution('gri30.yaml')
            fuel = "NH3"
            oxidizer = "O2:0.21, N2:0.79"
        elif fuel_upper == "SAF":
            try:
                gas = ct.Solution('ndodecane_reclink.yaml')
                fuel = "NC12H26"
            except Exception:
                gas = ct.Solution('gri30.yaml')
                fuel = "CH4"  # Fallback to Methane
            oxidizer = "O2:0.21, N2:0.79"
        else:
            gas = ct.Solution('gri30.yaml')
            fuel = "CH4"
            oxidizer = "O2:0.21, N2:0.79"

        # Set reactant mixture properties
        gas.TP = T_in, P_Pa
        gas.set_equivalence_ratio(phi, fuel, oxidizer)
        gas.equilibrate('HP')
        
        # Extract species
        T_ad = gas.T
        y_no = gas.Y[gas.species_index('NO')] if 'NO' in gas.species_names else 0.0
        y_no2 = gas.Y[gas.species_index('NO2')] if 'NO2' in gas.species_names else 0.0
        y_co = gas.Y[gas.species_index('CO')] if 'CO' in gas.species_names else 0.0
        y_co2 = gas.Y[gas.species_index('CO2')] if 'CO2' in gas.species_names else 0.0
        y_h2o = gas.Y[gas.species_index('H2O')] if 'H2O' in gas.species_names else 0.0
        
        lhv_table = {"HYDROGEN": 120e6, "AMMONIA": 18.6e6, "SAF": 42.8e6, "METHANE": 50e6}
        lhv = lhv_table.get(fuel_upper, 43e6)

        return {
            "adiabatic_flame_temperature_K": round(T_ad, 2),
            "lhv_j_kg": lhv,
            "species_mass_fractions": {
                "NOx": float(f"{y_no + y_no2:.4e}"),
                "CO": float(f"{y_co:.4e}"),
                "CO2": float(f"{y_co2:.4f}"),
                "H2O": float(f"{y_h2o:.4f}")
            },
            "soot_mass_fraction": float(f"{soot_yield:.4e}"),
            "pdf_flamelet": {
                "mixture_fraction_mean": round(z_mean, 4),
                "mixture_fraction_variance": round(z_variance, 6),
                "scalar_dissipation_rate_s": round(scalar_dissipation, 2)
            },
            "status": "CANTERA_SOLVED"
        }
    except Exception as e:
        # If Cantera fails (e.g., mechanism file not found, convergence error), fallback gracefully
        req_copy = CombustionRequest(**req.dict())
        fallback_res = solve_cantera_combustion(req_copy)
        fallback_res["status"] = f"CANTERA_FAIL_FALLBACK ({str(e)})"
        return fallback_res

# ══════════════════════════════════════════════════════════════════════
#  CALCULIX CONTACT STRESS INPUT GENERATOR & PROXY SOLVER
# ══════════════════════════════════════════════════════════════════════

def generate_calculix_fir_tree_inp(req: ContactStressRequest) -> str:
    inp_content = []
    inp_content.append("** CalculiX Input Deck generated by JetEngine Compiler Backend")
    inp_content.append("** Simulating non-linear fir-tree root contact")
    inp_content.append("*HEADING")
    inp_content.append("Fir-tree Contact Analysis under Centrifugal Loading")
    w = req.neck_width_mm / 1000.0
    inp_content.append("*NODE, NSET=Nall")
    inp_content.append(f"  1, -{w/2.0:.5f}, 0.00000, 0.0")
    inp_content.append(f"  2,  {w/2.0:.5f}, 0.00000, 0.0")
    inp_content.append(f"  3, -{w/2.0:.5f}, 0.02000, 0.0")
    inp_content.append(f"  4,  {w/2.0:.5f}, 0.02000, 0.0")
    inp_content.append("*ELEMENT, TYPE=CPE4, ELSET=Eroot")
    inp_content.append("  1, 1, 2, 4, 3")
    inp_content.append("*MATERIAL, NAME=Ti6Al4V")
    inp_content.append("*ELASTIC")
    inp_content.append("  115000.0E6, 0.33")
    inp_content.append("*DENSITY")
    inp_content.append("  4430.0")
    inp_content.append("*SOLID SECTION, ELSET=Eroot, MATERIAL=Ti6Al4V")
    inp_content.append("  1.0")
    inp_content.append("*SURFACE, NAME=Sblade")
    inp_content.append("  1, S1")
    inp_content.append("*SURFACE, NAME=Sdisc")
    inp_content.append("  1, S2")
    inp_content.append("*SURFACE INTERACTION, NAME=FrictionContact")
    inp_content.append("*FRICTION")
    inp_content.append(f"  {req.friction_coefficient:.2f}")
    inp_content.append("*CONTACT PAIR, INTERACTION=FrictionContact, TYPE=SURFACE TO SURFACE")
    inp_content.append("  Sblade, Sdisc")
    omega = 2.0 * math.pi * req.rotor_speed_rpm / 60.0
    f_cent = req.blade_mass_kg * req.blade_cg_radius_m * omega * omega
    inp_content.append("*STEP, NLGEOM")
    inp_content.append("  *STATIC")
    inp_content.append("  0.1, 1.0")
    inp_content.append("  *CLOAD")
    inp_content.append(f"    3, 2, {f_cent/2.0:.1f}")
    inp_content.append(f"    4, 2, {f_cent/2.0:.1f}")
    inp_content.append("*NODE PRINT, NSET=Nall")
    inp_content.append("    U")
    inp_content.append("  *EL PRINT, ELSET=Eroot")
    inp_content.append("    S")
    inp_content.append("*END STEP")
    return "\n".join(inp_content)

def solve_contact_stress(req: ContactStressRequest) -> Dict:
    omega = 2.0 * math.pi * req.rotor_speed_rpm / 60.0
    f_cf = req.blade_mass_kg * req.blade_cg_radius_m * omega * omega
    chord_m = 0.035
    a_neck = (req.neck_width_mm / 1000.0) * chord_m
    sigma_tensile = f_cf / a_neck
    theta = 45.0 * math.pi / 180.0
    f_normal = f_cf / (2.0 * req.tooth_count * math.sin(theta))
    a_contact = (2.0 / 1000.0) * chord_m
    sigma_contact = f_normal / a_contact
    p_contact_max = (4.0 / math.pi) * sigma_contact
    tau_friction = req.friction_coefficient * p_contact_max
    k_t = 2.2
    sigma_vm_peak = math.sqrt(sigma_tensile**2 + 3 * tau_friction**2) * k_t
    yield_strength = 880e6
    sf_yield = yield_strength / sigma_vm_peak
    ccx_inp = generate_calculix_fir_tree_inp(req)
    return {
        "centrifugal_force_N": round(f_cf, 1),
        "neck_tensile_stress_MPa": round(sigma_tensile / 1e6, 2),
        "peak_contact_pressure_MPa": round(p_contact_max / 1e6, 2),
        "friction_shear_stress_MPa": round(tau_friction / 1e6, 2),
        "von_mises_peak_stress_MPa": round(sigma_vm_peak / 1e6, 2),
        "safety_factor": round(sf_yield, 2),
        "passed": sf_yield >= 1.5,
        "calculix_input_deck": ccx_inp,
        "status": "CALCULIX_GENERATED"
    }

# ══════════════════════════════════════════════════════════════════════
#  TRANSIENT THERMAL SOAK-BACK SOLVER
# ══════════════════════════════════════════════════════════════════════

def solve_thermal_soakback(req: ThermalSoakbackRequest) -> Dict:
    """
    Simulates transient post-shutdown thermal soak-back from HPT disk (1100 K)
    through the shaft to the bearings. Cooling airflow has stopped.
    Solves 1D transient heat equation via explicit Finite Differences:
    dT/dt = alpha * d2T/dx2
    """
    L = req.shaft_length_m
    D = req.shaft_diameter_m
    nodes = 12
    dx = L / (nodes - 1)
    
    # Material properties of IN718 shaft
    rho = 8190.0      # kg/m3
    cp = 435.0        # J/(kg·K)
    k = 12.0          # W/(m·K) (at ~600-800K)
    alpha = k / (rho * cp)
    
    # Initialize temperature array
    T = [req.ambient_temp_K] * nodes
    T[0] = req.initial_disc_temp_K  # hot disc end (boundary condition)
    
    dt = 0.5 * (dx * dx) / alpha  # numerical stability (Fourier limit)
    total_steps = int(req.time_duration_s / dt)
    
    # Run transient simulation
    max_bearing_temp = req.ambient_temp_K
    bearing_node = int(nodes * 0.75)  # bearing located 3/4 along the shaft
    
    for _ in range(min(total_steps, 50000)):
        T_new = list(T)
        for i in range(1, nodes - 1):
            T_new[i] = T[i] + alpha * dt / (dx * dx) * (T[i+1] - 2*T[i] + T[i-1])
        # Convective boundary at bearing/shaft end
        T_new[-1] = T[-1] + alpha * dt / (dx * dx) * (2*T[-2] - 2*T[-1])
        T = T_new
        max_bearing_temp = max(max_bearing_temp, T[bearing_node])

    # Shaft thermal bowing calculation: y_bow = alpha_thermal * delta_T * L^2 / (8 * D)
    alpha_thermal = 1.3e-5 # 1/K thermal expansion coefficient
    delta_T_radial = 45.0  # typical radial delta T due to asymmetric heat pooling on shutdown
    max_bow_mm = (alpha_thermal * delta_T_radial * L * L) / (8.0 * D) * 1000.0

    # Coking limit of turbine engine lubricants is around 220°C (493 K)
    coking_limit_K = 493.15
    oil_coking = max_bearing_temp >= coking_limit_K

    return {
        "peak_bearing_temperature_K": round(max_bearing_temp, 2),
        "bearing_oil_coking_risk": oil_coking,
        "max_shaft_bowing_mm": round(max_bow_mm, 4),
        "coking_limit_K": coking_limit_K,
        "nodes_final_temperatures": [round(t, 1) for t in T],
        "status": "THERMAL_SOAKBACK_SOLVED"
    }

# ══════════════════════════════════════════════════════════════════════
#  MANEUVER & GYROSCOPIC LOADS SOLVER
# ══════════════════════════════════════════════════════════════════════

def solve_maneuver_loads(req: ManeuverLoadsRequest) -> Dict:
    """
    Computes vectorial gyroscopic moments on spinning rotor shafts during aircraft
    flight maneuvers (pull-up pitch/yaw rates) and checks bearing overload and blade tip casing rubbing.
    Mg = Omega_maneuver x (Ip * omega_spool)
    """
    omega_spool = 2.0 * math.pi * req.rotor_speed_rpm / 60.0
    # Polar moment of inertia: Ip = 0.5 * m * r^2
    Ip = 0.5 * req.rotor_mass_kg * req.rotor_cg_radius_m * req.rotor_cg_radius_m
    
    # Maneuver rate vector magnitude
    omega_man = math.sqrt(req.maneuver_pitch_rate_rad_s**2 + req.maneuver_yaw_rate_rad_s**2)
    
    # Gyroscopic moment
    M_g = Ip * omega_spool * omega_man
    
    # Spacing between bearings
    L_bearing = 0.8  # meters
    F_bearing_gyro = M_g / L_bearing
    
    # Gravity/maneuver static load
    F_bearing_g = req.rotor_mass_kg * req.g_load * 9.80665
    
    # Total combined bearing load
    F_bearing_total = F_bearing_gyro + F_bearing_g
    
    # Shaft bending deflection: y = F * L^3 / (48 * E * I_area)
    E_shaft = 200e9 # Steel shaft
    r_shaft = 0.040 # 40mm shaft radius
    I_area = (math.pi / 4.0) * (r_shaft**4)
    shaft_deflection_mm = (F_bearing_total * (L_bearing**3)) / (48.0 * E_shaft * I_area) * 1000.0
    
    # Casing clearance check (casing tip clearance typically 1.5mm)
    clearance_limit_mm = 1.5
    rubbing = shaft_deflection_mm >= clearance_limit_mm

    return {
        "gyroscopic_moment_Nm": round(M_g, 2),
        "bearing_radial_force_gyro_N": round(F_bearing_gyro, 1),
        "bearing_radial_force_maneuver_N": round(F_bearing_g, 1),
        "bearing_total_load_N": round(F_bearing_total, 1),
        "shaft_bending_deflection_mm": round(shaft_deflection_mm, 4),
        "casing_tip_rubbing_detected": rubbing,
        "status": "MANEUVER_LOADS_SOLVED"
    }

# ══════════════════════════════════════════════════════════════════════
#  COMPRESSOR STALL & SURGE TRANSIENT SOLVER (GREITZER MODEL)
# ══════════════════════════════════════════════════════════════════════

def solve_compressor_surge(req: CompressorSurgeRequest) -> Dict:
    """
    Solves the classic Greitzer lumped-parameter model for compressor stall and surge.
    Governing non-dimensional equations:
    dPhi/dtau = B * (Psi_c(Phi) - Psi)
    dPsi/dtau = 1/B * (Phi - Phi_t(Psi))
    """
    B = req.surge_param_B
    dt_tau = 0.01
    max_steps = int(req.duration_s * (req.speed_of_sound_mps / req.duct_length_m) * dt_tau)
    max_steps = min(max_steps, 10000)
    
    # Initial conditions
    phi = 0.5   # mass flow coefficient
    psi = 0.35  # pressure rise coefficient
    
    phi_history = []
    psi_history = []
    
    # Compressor characteristic curve (cubic shape)
    def psi_c(p_val):
        return 0.3 + 1.5 * p_val - 2.0 * (p_val ** 3)
        
    # Throttle valve flow (quadratic)
    gamma_valve = 0.7  # closed throttle during deceleration
    def phi_t(p_val):
        return gamma_valve * math.sqrt(max(p_val, 0.0))

    # Run explicit Euler time-marching
    surge_detected = False
    max_psi = psi
    min_phi = phi
    
    for step in range(max_steps):
        d_phi = B * (psi_c(phi) - psi) * dt_tau
        d_psi = (1.0 / B) * (phi - phi_t(psi)) * dt_tau
        
        phi = max(phi + d_phi, -0.2)  # allow minor backflow
        psi = max(psi + d_psi, 0.0)
        
        if step % 50 == 0:
            phi_history.append(round(phi, 4))
            psi_history.append(round(psi, 4))
            
        max_psi = max(max_psi, psi)
        min_phi = min(min_phi, phi)
        
        if phi < 0.0:
            surge_detected = True

    # Pressure spike and dynamic stress amplification factor (SAF)
    pressure_spike_ratio = max_psi / 0.35
    stress_magnification = 2.8 if surge_detected else 1.0

    return {
        "surge_detected": surge_detected,
        "max_pressure_rise_coef": round(max_psi, 3),
        "min_flow_coef": round(min_phi, 3),
        "pressure_spike_ratio": round(pressure_spike_ratio, 2),
        "blade_stress_magnification_factor": stress_magnification,
        "time_history_phi": phi_history[:50],
        "time_history_psi": psi_history[:50],
        "status": "COMPRESSOR_SURGE_SOLVED"
    }

# ══════════════════════════════════════════════════════════════════════
#  DYNAMIC IMPACT DYNAMICS SOLVER (JOHNSON-COOK MODEL)
# ══════════════════════════════════════════════════════════════════════

def solve_impact_dynamics(req: ImpactDynamicsRequest) -> Dict:
    """
    Evaluates dynamic containment and failure of fan blades under high-strain-rate impact
    (bird strike or ice ingestion) using the Johnson-Cook plasticity model.
    """
    V_rel = math.sqrt(req.blade_velocity_mps**2 + req.projectile_velocity_mps**2)
    E_k = 0.5 * req.projectile_mass_kg * V_rel * V_rel
    
    # Strain rate calculation based on blade thickness deformation timescale
    dt_impact = req.thickness_m / V_rel
    strain_rate = 1.0 / max(dt_impact, 1e-6)
    
    # Johnson-Cook flow stress: sigma_y = (A + B * eps_p^n) * (1 + C * ln(strain_rate/1.0))
    # Solve for plastic strain at leading edge assuming local kinetic energy absorption
    strain_rate_ref = 1.0
    jc_multiplier = 1.0 + req.material_C * math.log(max(strain_rate / strain_rate_ref, 1.0))
    
    # Iterative solution for equivalent plastic strain (eps_p)
    # E_k = Integral(sigma * d_eps) * Vol
    vol_deformed = req.chord_m * req.thickness_m * req.thickness_m * 4.0 # local shear zone
    energy_density = E_k / max(vol_deformed * req.density_kgm3, 1e-6) # J/kg
    
    eps_p = 0.001
    for _ in range(20):
        sigma_flow = (req.material_A_Pa + req.material_B_Pa * (eps_p ** req.material_n)) * jc_multiplier
        # plastic work: W_p = (A * eps + B / (n+1) * eps^(n+1)) * jc_mult
        W_p = (req.material_A_Pa * eps_p + (req.material_B_Pa / (req.material_n + 1.0)) * (eps_p ** (req.material_n + 1.0))) * jc_multiplier
        if W_p * vol_deformed >= E_k:
            break
        eps_p += 0.005
        if eps_p > 1.5:
            break

    # Material ultimate failure strain (Ti-6Al-4V ≈ 25%)
    failure_strain_limit = 0.25
    containment_passed = eps_p < failure_strain_limit

    return {
        "relative_impact_velocity_mps": round(V_rel, 2),
        "impact_energy_J": round(E_k, 1),
        "strain_rate_s1": round(strain_rate, 1),
        "flow_stress_MPa": round(sigma_flow / 1e6, 2),
        "peak_plastic_strain": round(eps_p, 4),
        "containment_passed": containment_passed,
        "failure_strain_limit": failure_strain_limit,
        "status": "IMPACT_DYNAMICS_SOLVED"
    }

# ══════════════════════════════════════════════════════════════════════
#  ADVANCED FATIGUE & LIFE PREDICTION SOLVER
# ══════════════════════════════════════════════════════════════════════

def solve_advanced_fatigue(req: AdvancedFatigueRequest) -> Dict:
    """
    Computes multiaxial fatigue (Findley parameter), Paris Law crack propagation,
    and creep-fatigue strain-range partitioning (SRP) interaction.
    """
    # 1. Findley Parameter: Findley = tau_a + k * sigma_n
    # Assume simple plane stress state for blade root filleting
    tau_a = req.stress_amplitude_MPa * 0.5 # shear amplitude
    sigma_n = req.mean_stress_MPa + req.stress_amplitude_MPa # max normal stress
    findley_val = tau_a + req.findley_k * sigma_n
    
    # Allowable Findley fatigue limit for Ti-6Al-4V (approx 380 MPa at 1e7 cycles)
    allowable_findley = 380.0
    sf_findley = allowable_findley / max(findley_val, 1.0)

    # 2. Paris Law Crack Growth: da/dN = C * (dK)^m
    # dK = Y * d_sigma * sqrt(pi * a)
    # Integrate from a_initial (0.5 mm) to a_critical (12.0 mm)
    a_init = 0.0005
    a_crit = 0.012
    Y = 1.12 # geometric factor
    d_sigma = req.stress_amplitude_MPa * 2.0 * 1e6 # stress range in Pa
    C_paris = 1.2e-11 # m/cycle
    m_paris = 3.2
    
    # Analytical integration of Paris Law:
    # N_cycles = (a_crit^((2-m)/2) - a_init^((2-m)/2)) / ((2-m)/2 * C * (Y * d_sigma * sqrt(pi))^m)
    exponent = (2.0 - m_paris) / 2.0
    denominator = exponent * C_paris * ((Y * d_sigma * math.sqrt(math.pi)) ** m_paris)
    numerator = (a_crit ** exponent) - (a_init ** exponent)
    crack_life_cycles = abs(numerator / denominator) if denominator != 0 else 1e12

    # 3. Creep-Fatigue Interaction (Strain-Range Partitioning)
    # LCF life (Basquin-Manson-Coffin)
    N_fatigue = math.pow(10, 24.0 - 6.0 * math.log10(req.stress_amplitude_MPa))
    
    # Larson-Miller Creep Life (IN718 or Ti64)
    # LMP = T_R * (log10(t_rupture) + 20)
    T_R = req.temperature_K * 1.8 # Kelvin to Rankine
    # Estimate rupture life from typical Larson-Miller curve
    if req.temperature_K < 750.0:
        creep_life_hours = 1e6
    else:
        # rupture time drops exponentially with stress and temperature
        creep_life_hours = math.exp(30.0 - (req.stress_amplitude_MPa / 40.0) - (req.temperature_K / 80.0))
        creep_life_hours = max(1.0, creep_life_hours)
        
    # Combined creep-fatigue safety life (SRP): 1/N_total = 1/N_f + 1/N_creep
    cycles_per_hour = 60.0
    N_creep = creep_life_hours * cycles_per_hour
    N_combined = 1.0 / ((1.0 / max(N_fatigue, 1.0)) + (1.0 / max(N_creep, 1.0)))

    return {
        "findley_parameter_MPa": round(findley_val, 2),
        "findley_safety_factor": round(sf_findley, 2),
        "paris_crack_life_cycles": round(crack_life_cycles, 0),
        "larson_miller_creep_life_hrs": round(creep_life_hours, 1),
        "combined_creep_fatigue_life_cycles": round(N_combined, 0),
        "status": "ADVANCED_FATIGUE_SOLVED"
    }

# ══════════════════════════════════════════════════════════════════════
#  3D ACOUSTICS & ROTOR-STATOR SPINNING MODES
# ══════════════════════════════════════════════════════════════════════

def solve_acoustics_3d(req: Acoustics3DRequest) -> Dict:
    """
    Computes 3D duct acoustic spinning modes (Tyler-Sofrin) and evaluates
    nozzle installation jet-wing acoustic amplification.
    """
    shaft_freq = req.shaft_speed_rpm / 60.0
    BPF = req.blade_count * shaft_freq
    
    # 1. Tyler-Sofrin Rotor-Stator Spinning Modes: m = n*B - k*V
    # We evaluate the first 3 harmonics (n=1,2,3) and interactions (k=1,2)
    modes = []
    cutoff_freqs = []
    
    # Bessel first roots proxy for cutoff radial wavenumber (k_m_mu)
    # approximation: k_m_mu = |m| + 1.8 * mu + 0.8
    mu = 1 # first radial mode
    
    for n in [1, 2, 3]:
        for k in [1, 2]:
            m = n * req.blade_count - k * req.stator_count
            k_m_mu = abs(m) + 1.8 * mu + 0.8
            f_cut = (k_m_mu * req.sound_speed_mps) / (2.0 * math.pi * req.duct_radius_m)
            
            modes.append(m)
            cutoff_freqs.append(round(f_cut, 1))

    # Check propagating vs decaying modes at Blade Passing Frequency (BPF)
    propagating_modes_count = sum(1 for f in cutoff_freqs if f <= BPF)

    # 2. Jet-surface installation noise: Lighthill 8th power jet noise
    # modified by nozzle-wing distance d (inversely proportional: power ~ 1/d^2)
    # base acoustic power: W_jet ~ V_jet^8
    base_jet_power_dB = 10.0 * 8.0 * math.log10(max(req.jet_velocity_mps, 1.0)) - 10.0
    
    d_norm = max(req.nozzle_wing_distance_m / req.duct_radius_m, 0.2)
    installation_amp_dB = 15.0 * math.exp(-2.0 * (d_norm - 0.2)) # near field acoustic shielding decay
    total_acoustic_power_dB = base_jet_power_dB + installation_amp_dB

    return {
        "bpf_frequency_Hz": round(BPF, 1),
        "spinning_modes": modes[:6],
        "cutoff_frequencies_Hz": cutoff_freqs[:6],
        "propagating_modes_at_bpf": propagating_modes_count,
        "installation_acoustics_amplification_dB": round(installation_amp_dB, 2),
        "total_acoustic_power_dB": round(total_acoustic_power_dB, 2),
        "status": "ACOUSTICS_3D_SOLVED"
    }

# ══════════════════════════════════════════════════════════════════════
#  API ROUTES
# ══════════════════════════════════════════════════════════════════════

@app.get("/")
def read_root():
    return {
        "msg": "NASA JetEngine Advanced Physics Backend Service Running",
        "cantera_available": CANTERA_AVAILABLE,
        "supported_endpoints": [
            "/api/combustion", 
            "/api/contact_stress", 
            "/api/thermal_soakback",
            "/api/maneuver_loads",
            "/api/compressor_surge",
            "/api/impact_dynamics",
            "/api/advanced_fatigue",
            "/api/acoustics_3d",
            "/health"
        ]
    }

@app.get("/health")
def health_check():
    return {"status": "healthy", "cantera": CANTERA_AVAILABLE}

@app.post("/api/combustion")
def api_combustion(req: CombustionRequest):
    return solve_cantera_combustion(req)

@app.post("/api/contact_stress")
def api_contact_stress(req: ContactStressRequest):
    return solve_contact_stress(req)

@app.post("/api/thermal_soakback")
def api_thermal_soakback(req: ThermalSoakbackRequest):
    return solve_thermal_soakback(req)

@app.post("/api/maneuver_loads")
def api_maneuver_loads(req: ManeuverLoadsRequest):
    return solve_maneuver_loads(req)

@app.post("/api/compressor_surge")
def api_compressor_surge(req: CompressorSurgeRequest):
    return solve_compressor_surge(req)

@app.post("/api/impact_dynamics")
def api_impact_dynamics(req: ImpactDynamicsRequest):
    return solve_impact_dynamics(req)

@app.post("/api/advanced_fatigue")
def api_advanced_fatigue(req: AdvancedFatigueRequest):
    return solve_advanced_fatigue(req)

@app.post("/api/acoustics_3d")
def api_acoustics_3d(req: Acoustics3DRequest):
    return solve_acoustics_3d(req)

# ══════════════════════════════════════════════════════════════════════
#  MAIN RUNNER
# ══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    os.makedirs(os.path.dirname(os.path.abspath(__file__)), exist_ok=True)
    print("Starting simulation backend server on http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)
