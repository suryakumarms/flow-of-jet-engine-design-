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
    public static class EngineMountSystem
    {
        public class MountResult
        {
            public double ForwardMountForce_kN { get; set; }
            public double AftMountForce_kN { get; set; }
            public double MountSafetyFactor { get; set; }
            public double PylonDeflection_mm { get; set; }
            public bool MountStructuralPassed { get; set; }
        }

        public static MountResult Solve(double thrust_N, double engineWeight_N, double g_maneuver = 9.0, double gyroMoment_Nm = 15000.0, double mountThickness_mm = 25.0)
        {
            var r = new MountResult();
            
            // 3-link mount system (two forward mounts, one aft mount)
            double F_vertical = engineWeight_N * g_maneuver;
            
            // Moment balance
            double engine_length = 2.5; // m
            r.ForwardMountForce_kN = (0.70 * thrust_N + 0.5 * F_vertical + gyroMoment_Nm / engine_length) / 1000.0;
            r.AftMountForce_kN = (0.30 * thrust_N + 0.5 * F_vertical + gyroMoment_Nm / engine_length) / 1000.0;
            
            // Mount capability scales linearly with thickness (titanium yield limit)
            double allowable_mount_force_kN = 12.0 * mountThickness_mm; 
            r.MountSafetyFactor = allowable_mount_force_kN / Math.Max(r.ForwardMountForce_kN, 1.0);
            
            // Pylon stiffness deflection: K_pylon scales with mount thickness
            double k_pylon = 45e6 * (mountThickness_mm / 15.0);
            r.PylonDeflection_mm = (F_vertical + thrust_N) / k_pylon * 1000.0;
            
            r.MountStructuralPassed = r.MountSafetyFactor >= 1.5 && r.PylonDeflection_mm < 5.0; // 5mm deflection limit
            
            return r;
        }
    }

}
