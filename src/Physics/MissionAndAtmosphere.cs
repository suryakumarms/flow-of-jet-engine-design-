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
    public class MissionRequirements
    {
        // --- Mission Profile ---
        public double ThrustRequired_N       { get; set; } = 150000.0;  // 33,700 lbf class
        public double CruiseMach             { get; set; } = 0.82;
        public double CruiseAltitude_m       { get; set; } = 10668.0;   // 35,000 ft
        public double TakeoffAltitude_m      { get; set; } = 0.0;       // Sea level
        
        // --- Cycle Parameters (Initial Guesses, will be optimized) ---
        public double BypassRatio            { get; set; } = 9.0;
        public double OverallPressureRatio   { get; set; } = 40.0;
        public double FanPressureRatio       { get; set; } = 1.55;
        public double LPCPressureRatio       { get; set; } = 2.5;
        public double TurbineInletTemp_K     { get; set; } = 1750.0;    // T4
        
        // --- Component Efficiencies (Polytropic) ---
        public double EtaFan                 { get; set; } = 0.91;
        public double EtaLPC                 { get; set; } = 0.90;
        public double EtaHPC                 { get; set; } = 0.88;
        public double EtaHPT                 { get; set; } = 0.92;
        public double EtaLPT                 { get; set; } = 0.93;
        public double EtaCombustor           { get; set; } = 0.995;
        public double EtaInlet               { get; set; } = 0.98;
        public double EtaNozzleCore          { get; set; } = 0.98;
        public double EtaNozzleBypass        { get; set; } = 0.97;
        public double EtaMechanicalHP        { get; set; } = 0.99;
        public double EtaMechanicalLP        { get; set; } = 0.99;
        public double CombustorPressureLoss  { get; set; } = 0.04;      // ΔP/P fraction
        
        // --- Fuel ---
        public double FuelHeatingValue_J     { get; set; } = 43.1e6;    // Jet-A LHV (J/kg)
        
        // --- Constraints ---
        public double MaxTipSpeed_mps        { get; set; } = 450.0;     // Fan tip speed
        public double MinSurgeMargin         { get; set; } = 0.15;      // 15%
        public double MaxExitTemp_K          { get; set; } = 1950.0;    // Material limit
        
        // --- Manufacturing ---
        public string ManufacturingProcess   { get; set; } = "DMLS";
        public string PrimaryMaterial        { get; set; } = "Inconel 718";
        
        // --- Turbine Cooling (Gap 1) ---
        // Maximum allowable metal temperature for HPT blades (K)
        // CMSX-4 uncooled limit ≈ 1250 K; TBC adds ~100 K headroom
        public double MaxMetalTemp_K     { get; set; } = 1250.0;
        // Semi-empirical cooling technology factor C_tech (convective+film)
        // 0.05 = mature film cooling; 0.08 = transpiration/impingement
        public double CoolingTechFactor  { get; set; } = 0.06;

        // --- Water/Methanol Injection (takeoff thrust recovery) ---
        public bool   WaterInjectionActive { get; set; } = false;
        public double WaterInjectionRatio  { get; set; } = 0.02; // 2% water to air mass ratio
        
        // --- Derived ---
        public double HPCPressureRatio => OverallPressureRatio / (FanPressureRatio * LPCPressureRatio);
    }

    public static class Atmosphere
    {
        private const double T0      = 288.15;    // Sea-level temp (K)
        private const double P0      = 101325.0;  // Sea-level pressure (Pa)
        private const double Rho0    = 1.225;     // Sea-level density (kg/m³)
        private const double LapseRate = -0.0065;  // K/m (troposphere)
        private const double g0      = 9.80665;
        private const double R_air   = 287.058;
        private const double gamma   = 1.4;

        public static (double T, double P, double rho, double a) AtAltitude(double h_m)
        {
            double T, P, rho;
            if (h_m <= 11000.0) // Troposphere
            {
                T   = T0 + LapseRate * h_m;
                P   = P0 * Math.Pow(T / T0, -g0 / (LapseRate * R_air));
                rho = P / (R_air * T);
            }
            else // Stratosphere (simplified, up to ~25 km)
            {
                double T11  = T0 + LapseRate * 11000.0;
                double P11  = P0 * Math.Pow(T11 / T0, -g0 / (LapseRate * R_air));
                T   = T11; // Isothermal in lower stratosphere
                P   = P11 * Math.Exp(-g0 * (h_m - 11000.0) / (R_air * T11));
                rho = P / (R_air * T);
            }
            double a = Math.Sqrt(gamma * R_air * T); // Speed of sound
            return (T, P, rho, a);
        }
    }

    public class GasStation
    {
        public string Name          { get; set; } = "";
        public int    StationNumber { get; set; }
        
        // Total (stagnation) quantities
        public double Tt    { get; set; }  // Total temperature (K)
        public double Pt    { get; set; }  // Total pressure (Pa)
        
        // Mass flow
        public double MassFlow  { get; set; }  // kg/s
        
        // Composition tracking
        public double FuelAirRatio { get; set; }  // f
        
        // Gas properties (vary with composition and temperature)
        public double Gamma   { get; set; } = 1.4;
        public double Cp      { get; set; } = 1005.0;  // J/(kg·K)
        
        // Mach number (if known)
        public double Mach { get; set; }
        
        // Static quantities (computed from total + Mach)
        public double Ts => Tt / (1.0 + (Gamma - 1.0) / 2.0 * Mach * Mach);
        public double Ps => Pt * Math.Pow(Ts / Tt, Gamma / (Gamma - 1.0));
        
        // Velocity
        public double V => Mach * Math.Sqrt(Gamma * (Cp * (Gamma - 1.0) / Gamma) * Ts);
        
        public GasStation Clone()
        {
            return (GasStation)MemberwiseClone();
        }

        public override string ToString()
            => $"S{StationNumber} [{Name}]: Tt={Tt:F1}K  Pt={Pt/1000:F1}kPa  ṁ={MassFlow:F2}kg/s  γ={Gamma:F3}  f={FuelAirRatio:F5}";
    }

    public class CycleResult
    {
        public Dictionary<int, GasStation> Stations { get; set; } = new();
        
        // Performance
        public double NetThrust_N        { get; set; }
        public double TSFC_gkNs          { get; set; }  // g/(kN·s)
        public double ThermalEfficiency  { get; set; }
        public double PropulsiveEfficiency { get; set; }
        public double OverallEfficiency  { get; set; }
        public double SpecificThrust     { get; set; }  // N·s/kg
        
        // Mass flows
        public double CoreMassFlow       { get; set; }  // kg/s
        public double BypassMassFlow     { get; set; }
        public double BypassRatio        => CoreMassFlow > 0 ? BypassMassFlow / CoreMassFlow : 0.0;
        public double FuelFlow           { get; set; }  // kg/s
        
        // Power balance
        public double HPT_Power          { get; set; }  // W
        public double LPT_Power          { get; set; }
        public double HPC_Power          { get; set; }
        public double FanPower           { get; set; }
        
        // Sizing
        public double FanDiameter_m      { get; set; }
        public double CoreDiameter_m     { get; set; }
        // Stored design params for off-design / digital twin
        public double EtaFan             { get; set; }
        public double EtaHPC             { get; set; }
        public double TurbineInletTemp_K { get; set; }
        public double OverallPressureRatio{ get; set; }
        
        // Cooling bleed (Gap 1 outputs)
        public double HPT_CoolantFraction { get; set; }   // ε_cool = ṁ_cool/ṁ_core
        public double HPT_BleedMassFlow   { get; set; }   // kg/s extracted from HPC exit
        public double HPT_MixedTemp_K     { get; set; }   // T_mixed after coolant reinjection
        
        // Validation
        public bool   IsValid            { get; set; }
        public List<string> Warnings     { get; set; } = new();
        public List<string> Errors       { get; set; } = new();
        
        public void Print()
        {
            Console.WriteLine("════════════════════════════════════════════════════════");
            Console.WriteLine("  BRAYTON CYCLE SOLUTION");
            Console.WriteLine("════════════════════════════════════════════════════════");
            foreach (var s in Stations.OrderBy(kv => kv.Key))
                Console.WriteLine($"  {s.Value}");
            Console.WriteLine("────────────────────────────────────────────────────────");
            Console.WriteLine($"  Net Thrust:        {NetThrust_N:F0} N ({NetThrust_N/4.448:F0} lbf)");
            Console.WriteLine($"  TSFC:              {TSFC_gkNs:F2} g/(kN·s)");
            Console.WriteLine($"  Thermal η:         {ThermalEfficiency*100:F1}%");
            Console.WriteLine($"  Propulsive η:      {PropulsiveEfficiency*100:F1}%");
            Console.WriteLine($"  Overall η:         {OverallEfficiency*100:F1}%");
            Console.WriteLine($"  Specific Thrust:   {SpecificThrust:F1} N·s/kg");
            Console.WriteLine($"  Core ṁ:            {CoreMassFlow:F2} kg/s");
            Console.WriteLine($"  Bypass ṁ:          {BypassMassFlow:F2} kg/s");
            Console.WriteLine($"  Fuel flow:         {FuelFlow:F3} kg/s");
            Console.WriteLine($"  Fan diameter:      {FanDiameter_m*1000:F0} mm");
            Console.WriteLine($"  Core diameter:     {CoreDiameter_m*1000:F0} mm");
            Console.WriteLine($"  HPT coolant frac:  ε={HPT_CoolantFraction:F4}  ṁ_cool={HPT_BleedMassFlow:F3} kg/s");
            Console.WriteLine($"  HPT mixed T45:     {HPT_MixedTemp_K:F1} K");
            if (Warnings.Count > 0)
            {
                Console.WriteLine("  ⚠ WARNINGS:");
                foreach (var w in Warnings) Console.WriteLine($"    - {w}");
            }
            if (Errors.Count > 0)
            {
                Console.WriteLine("  ✗ ERRORS:");
                foreach (var e in Errors) Console.WriteLine($"    - {e}");
            }
            Console.WriteLine("════════════════════════════════════════════════════════");
        }
    }

}
