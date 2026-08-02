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
    public class SdfCylinder : IImplicit
    {
        readonly Vector3 _a, _b;
        readonly float _r;
        public SdfCylinder(Vector3 a, Vector3 b, float r) { _a = a; _b = b; _r = r; }
        public float fSignedDistance(in Vector3 p)
        {
            var ba = _b - _a;
            var pa = p - _a;
            float h = Vector3.Dot(ba, ba);
            float t = Vector3.Dot(pa, ba) / h;
            float tc = Math.Clamp(t, 0f, 1f);
            return (pa - ba * tc).Length() - _r;
        }
    }

    public class SdfAnnulus : IImplicit
    {
        readonly Func<float, float> _rInner, _rOuter;
        readonly float _zMin, _zMax;
        public SdfAnnulus(Func<float, float> rInner, Func<float, float> rOuter, float zMin, float zMax)
        {
            _rInner = rInner; _rOuter = rOuter; _zMin = zMin; _zMax = zMax;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zMin || p.Z > _zMax) return 1000f;
            float rPt = new Vector2(p.X, p.Y).Length();
            float ri = _rInner(p.Z);
            float ro = _rOuter(p.Z);
            return Math.Max(rPt - ro, ri - rPt);
        }
    }

    public class SdfRevolution : IImplicit
    {
        readonly Func<float, float> _rFunc;
        readonly float _offset, _thickness, _zMin, _zMax;
        public SdfRevolution(Func<float, float> rFunc, float offset, float thickness, float zMin, float zMax)
        {
            _rFunc = rFunc; _offset = offset; _thickness = thickness; _zMin = zMin; _zMax = zMax;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zMin || p.Z > _zMax) return 1000f;
            float rBase = _rFunc(p.Z);
            float rPt = new Vector2(p.X, p.Y).Length();
            float ri = rBase + _offset;
            float ro = ri + _thickness;
            return Math.Max(rPt - ro, ri - rPt);
        }
    }

    public class SdfBlade : IImplicit
    {
        readonly float _hubR, _tipR, _chord, _thickness, _stagger, _zCenter;
        readonly float _thetaCenter;  // Angular position on disk (rad)

        public SdfBlade(float hubR, float tipR, float chord, float thickness,
                        float stagger, float zCenter, float thetaCenter)
        {
            _hubR = hubR; _tipR = tipR; _chord = chord;
            _thickness = thickness; _stagger = stagger;
            _zCenter = zCenter; _thetaCenter = thetaCenter;
        }

        public float fSignedDistance(in Vector3 p)
        {
            // Convert to cylindrical
            float r = new Vector2(p.X, p.Y).Length();
            float theta = MathF.Atan2(p.Y, p.X);

            // Radial bounds
            if (r < _hubR - 1f || r > _tipR + 1f) return 1000f;
            float dRad = Math.Max(_hubR - r, r - _tipR);

            // Angular: blade at _thetaCenter, width = chord/r
            float angWidth = _chord / r;
            float dTheta = theta - _thetaCenter;
            // Wrap to [-π, π]
            while (dTheta > MathF.PI) dTheta -= 2f * MathF.PI;
            while (dTheta < -MathF.PI) dTheta += 2f * MathF.PI;

            // Rotate by stagger: the blade is angled in the (theta, z) plane
            float localTheta = dTheta * r;  // Arc distance
            float localZ = p.Z - _zCenter;

            // Stagger rotation
            float ct = MathF.Cos(_stagger), st = MathF.Sin(_stagger);
            float u = localTheta * ct + localZ * st;  // Along chord
            float v = -localTheta * st + localZ * ct;  // Perpendicular

            // Chord-wise: |u| < chord/2
            float dChord = Math.Abs(u) - _chord / 2f;

            // Thickness-wise: |v| < thickness/2 (NACA-like: thicker at 30% chord)
            float tLocal = _thickness * (1f - 4f * (u / _chord) * (u / _chord));
            tLocal = Math.Max(tLocal, _thickness * 0.3f);
            float dThick = Math.Abs(v) - tLocal / 2f;

            float dBlade = Math.Max(dChord, dThick);
            return Math.Max(dBlade, dRad);
        }
    }

    public class SdfBladeRow : IImplicit
    {
        readonly float _hubR, _tipR, _chord, _thickness, _stagger, _zCenter;
        readonly int _count;

        public SdfBladeRow(float hubR, float tipR, float chord, float thickness,
                           float stagger, float zCenter, int count)
        {
            _hubR = hubR; _tipR = tipR; _chord = chord;
            _thickness = thickness; _stagger = stagger;
            _zCenter = zCenter; _count = count;
        }

        public float fSignedDistance(in Vector3 p)
        {
            float r = new Vector2(p.X, p.Y).Length();
            if (r < _hubR - 2f || r > _tipR + 2f) return 1000f;
            if (Math.Abs(p.Z - _zCenter) > _chord * 2f) return 1000f;

            float theta = MathF.Atan2(p.Y, p.X);
            float sector = 2f * MathF.PI / _count;

            // Find nearest blade
            float tMod = ((theta % sector) + sector) % sector;
            float dTheta = tMod - sector / 2f;

            // Blade distance in tangential-axial plane
            float localT = dTheta * r;
            float localZ = p.Z - _zCenter;
            float ct = MathF.Cos(_stagger), st = MathF.Sin(_stagger);
            float u = localT * ct + localZ * st;
            float v = -localT * st + localZ * ct;

            float dChord = Math.Abs(u) - _chord / 2f;
            float tLocal = _thickness * (1f - 3f * (u / _chord) * (u / _chord));
            tLocal = Math.Max(tLocal, _thickness * 0.25f);
            float dThick = Math.Abs(v) - tLocal / 2f;
            float dRad = Math.Max(_hubR - r, r - _tipR);

            return Math.Max(Math.Max(dChord, dThick), dRad);
        }
    }

    public class SdfGyroid : IImplicit
    {
        readonly float _s, _t;
        public SdfGyroid(float period, float threshold)
        {
            _s = 2f * MathF.PI / period; _t = threshold;
        }
        public float fSignedDistance(in Vector3 p)
        {
            float val = MathF.Sin(p.X * _s) * MathF.Cos(p.Y * _s)
                      + MathF.Sin(p.Y * _s) * MathF.Cos(p.Z * _s)
                      + MathF.Sin(p.Z * _s) * MathF.Cos(p.X * _s);
            return (_t - val) * 2f;
        }
    }

    public class SdfDisk : IImplicit
    {
        readonly float _rIn, _rOut, _zCenter, _thick;
        public SdfDisk(float rIn, float rOut, float zCenter, float thickness)
        {
            _rIn = rIn; _rOut = rOut; _zCenter = zCenter; _thick = thickness;
        }
        public float fSignedDistance(in Vector3 p)
        {
            float r = new Vector2(p.X, p.Y).Length();
            float dR = Math.Max(_rIn - r, r - _rOut);
            float dZ = Math.Abs(p.Z - _zCenter) - _thick / 2f;
            return Math.Max(dR, dZ);
        }
    }

    public class SdfSpinner : IImplicit
    {
        readonly float _zMin, _zMax, _rMax;
        public SdfSpinner(float zMin, float zMax, float rMax)
        {
            _zMin = zMin; _zMax = zMax; _rMax = rMax;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zMin || p.Z > _zMax)
            {
                float dZ = p.Z < _zMin ? _zMin - p.Z : p.Z - _zMax;
                float rPt = new Vector2(p.X, p.Y).Length();
                float dR = rPt - (p.Z < _zMin ? 0f : _rMax);
                return Math.Max(dR, dZ);
            }
            float t = (p.Z - _zMin) / (_zMax - _zMin);
            float rTarget = _rMax * MathF.Sqrt(t);
            float r = new Vector2(p.X, p.Y).Length();
            return r - rTarget;
        }
    }

    public class SdfNacelleInlet : IImplicit
    {
        readonly float _rTip, _zFace, _wallT, _lipRadius;
        public SdfNacelleInlet(float rTip, float zFace, float wallThickness, float lipRadius)
        {
            _rTip = rTip; _zFace = zFace; _wallT = wallThickness; _lipRadius = lipRadius;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z > _zFace + _lipRadius * 4f || p.Z < _zFace - _wallT * 3f) return 1000f;
            float r = new Vector2(p.X, p.Y).Length();
            // Toroidal inlet lip: torus of radius _rTip - _lipRadius centred at (rTip-lipRadius, zFace)
            float dx = r - (_rTip - _lipRadius);
            float dz = p.Z - _zFace;
            float torusDist = MathF.Sqrt(dx * dx + dz * dz) - _lipRadius;
            // Outer cowl cylinder extending aft
            float cowlDist = Math.Max(
                Math.Abs(r - _rTip + _wallT / 2f) - _wallT / 2f,
                p.Z - (_zFace + _lipRadius * 2f));
            return Math.Min(torusDist, cowlDist);
        }
    }

    public class SdfHubDrum : IImplicit
    {
        readonly float _rHub, _zFront, _zRear, _wallT;
        public SdfHubDrum(float rHub, float zFront, float zRear, float wallThickness)
        {
            _rHub = rHub; _zFront = zFront; _zRear = zRear; _wallT = wallThickness;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zFront - 2f || p.Z > _zRear + 2f) return 1000f;
            float r = new Vector2(p.X, p.Y).Length();
            float dZ = Math.Max(_zFront - p.Z, p.Z - _zRear);
            // Annular drum shell: ri = rHub - wallT, ro = rHub
            float dR = Math.Max(r - _rHub, (_rHub - _wallT) - r);
            return Math.Max(dR, dZ);
        }
    }

    public class SdfBypassDuct : IImplicit
    {
        readonly Func<float, float> _rInner, _rOuter;
        readonly float _zStart, _zEnd, _wallT;
        public SdfBypassDuct(Func<float, float> rInner, Func<float, float> rOuter, float zStart, float zEnd, float wallThickness)
        {
            _rInner = rInner; _rOuter = rOuter; _zStart = zStart; _zEnd = zEnd; _wallT = wallThickness;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zStart || p.Z > _zEnd) return 1000f;
            float r = new Vector2(p.X, p.Y).Length();
            float ri = _rInner(p.Z);
            float ro = _rOuter(p.Z);
            // Two thin shells at ri and ro
            float innerShell = Math.Abs(r - ri) - _wallT;
            float outerShell = Math.Abs(r - ro) - _wallT;
            return Math.Min(innerShell, outerShell);
        }
    }

    public class SdfSpinnerBackPlate : IImplicit
    {
        readonly float _rHub, _zFace, _thickness;
        public SdfSpinnerBackPlate(float rHub, float zFace, float thickness)
        {
            _rHub = rHub; _zFace = zFace; _thickness = thickness;
        }
        public float fSignedDistance(in Vector3 p)
        {
            float r = new Vector2(p.X, p.Y).Length();
            if (r > _rHub + 2f) return 1000f;
            float dZ = Math.Abs(p.Z - _zFace) - _thickness / 2f;
            float dR = r - _rHub;
            return Math.Max(dZ, dR);
        }
    }

    public class SdfChevronCut : IImplicit
    {
        readonly float _zStart, _zEnd, _rMin, _rMax;
        readonly int _numTeeth;
        public SdfChevronCut(float zStart, float zEnd, float rMin, float rMax, int numTeeth)
        {
            _zStart = zStart; _zEnd = zEnd; _rMin = rMin; _rMax = rMax; _numTeeth = numTeeth;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zStart || p.Z > _zEnd) return 1000f;
            float angle = MathF.Atan2(p.Y, p.X);
            float r = new Vector2(p.X, p.Y).Length();
            float t = (p.Z - _zStart) / (_zEnd - _zStart);
            float normAngle = (angle + MathF.PI) / (2f * MathF.PI) * _numTeeth;
            float triangle = Math.Abs((normAngle % 1f) - 0.5f) * 2f;
            if (t > triangle)
            {
                float dR = Math.Max(_rMin - r, r - _rMax);
                float dZ = Math.Abs(p.Z - (_zStart + _zEnd)/2f) - (_zEnd - _zStart)/2f;
                return Math.Max(dR, dZ);
            }
            return 1000f;
        }
    }

    public class SdfTwistedBladeRow : IImplicit
    {
        readonly float _rH,_rT,_ch,_th,_sH,_sT,_zC; readonly int _n;
        public SdfTwistedBladeRow(float rH,float rT,float ch,float th,float sHd,float sTd,float zC,int n)
        {_rH=rH;_rT=rT;_ch=ch;_th=th;_sH=sHd*MathF.PI/180f;_sT=sTd*MathF.PI/180f;_zC=zC;_n=n;}
        public float fSignedDistance(in Vector3 p)
        {
            float r=MathF.Sqrt(p.X*p.X+p.Y*p.Y);
            if(r<_rH*.9f||r>_rT*1.05f) return 1000f;
            float t=Math.Clamp((r-_rH)/Math.Max(_rT-_rH,.001f),0f,1f);
            float sg=_sH+t*(_sT-_sH),phi=MathF.Atan2(p.Y,p.X);
            float sec=2f*MathF.PI/_n,md=10f;
            for(int i=0;i<_n;i++){
                float dp=phi-i*sec;
                while(dp>sec/2)dp-=sec; while(dp<-sec/2)dp+=sec;
                float da=dp*r,dax=p.Z-_zC;
                float dl=da*MathF.Cos(sg)+dax*MathF.Sin(sg),dperp=-da*MathF.Sin(sg)+dax*MathF.Cos(sg);
                float dx=MathF.Max(MathF.Abs(dl)-_ch/2f,0f),dy=MathF.Max(MathF.Abs(dperp)-_th/2f,0f);
                float d=MathF.Sqrt(dx*dx+dy*dy)-_th*.15f; if(d<md)md=d;
            }
            return md;
        }
    }

    public class SdfHollowCavity : IImplicit
    {
        readonly float _rH,_rT,_ch,_th,_zC; readonly int _n;
        public SdfHollowCavity(float rH,float rT,float ch,float th,float zC,int n)
        {_rH=rH;_rT=rT;_ch=ch*.6f;_th=th*.4f;_zC=zC;_n=n;}
        public float fSignedDistance(in Vector3 p)
        {
            float r=MathF.Sqrt(p.X*p.X+p.Y*p.Y);
            if(r<_rH||r>_rT*.95f) return 1000f;
            float phi=MathF.Atan2(p.Y,p.X),sec=2f*MathF.PI/_n,md=10f;
            for(int i=0;i<_n;i++){
                float dp=phi-i*sec;
                while(dp>sec/2)dp-=sec; while(dp<-sec/2)dp+=sec;
                float da=dp*r,dax=p.Z-_zC;
                float dx=MathF.Max(MathF.Abs(da)-_ch/2f,0f),dy=MathF.Max(MathF.Abs(dax)-_th/2f,0f);
                float d=MathF.Sqrt(dx*dx+dy*dy)-_th*.1f; if(d<md)md=d;
            }
            return -md;
        }
    }

    public class SdfLabyrinthSeals : IImplicit
    {
        readonly float _rShaft, _hTooth, _tTooth, _pitch, _zStart, _zEnd;
        readonly int   _nTeeth;
        public SdfLabyrinthSeals(float rShaft, float toothHeight, float toothThickness,
                                  float pitch, float zStart, float zEnd, int nTeeth)
        {
            _rShaft = rShaft; _hTooth = toothHeight; _tTooth = toothThickness;
            _pitch  = pitch;  _zStart = zStart;       _zEnd   = zEnd;
            _nTeeth = nTeeth;
        }
        public float fSignedDistance(in Vector3 p)
        {
            if (p.Z < _zStart - 1f || p.Z > _zEnd + 1f) return 1000f;
            float r = MathF.Sqrt(p.X * p.X + p.Y * p.Y);
            // Shaft body SDF
            float shaftDist = r - _rShaft;
            // Tooth SDF: periodic axial repetition
            float zRel = p.Z - _zStart;
            float zMod = zRel - _pitch * MathF.Floor(zRel / _pitch); // wrap to one period
            float toothCenter = _pitch / 2.0f;
            float dzTooth = MathF.Abs(zMod - toothCenter) - _tTooth / 2.0f; // axial slab
            float drTooth = r - (_rShaft + _hTooth);                         // radial limit
            float toothDist = MathF.Max(dzTooth, drTooth);                   // box-shaped tooth
            // Take the union of shaft and teeth
            return MathF.Min(shaftDist, toothDist);
        }
    }

    public class SdfSerpentineCooling : IImplicit
    {
        readonly float _rH, _rT, _ch, _th, _zC;
        readonly int   _n;       // blade count
        public SdfSerpentineCooling(float rH, float rT, float ch, float th, float zC, int n)
        { _rH = rH; _rT = rT; _ch = ch; _th = th; _zC = zC; _n = n; }
        public float fSignedDistance(in Vector3 p)
        {
            float r = MathF.Sqrt(p.X * p.X + p.Y * p.Y);
            if (r < _rH * 0.98f || r > _rT * 0.95f) return 1000f;
            float phi = MathF.Atan2(p.Y, p.X);
            float sec = 2f * MathF.PI / _n;
            float bestDist = 1000f;
            for (int i = 0; i < _n; i++)
            {
                float dp = phi - i * sec;
                while (dp >  sec / 2) dp -= sec;
                while (dp < -sec / 2) dp += sec;
                float da = dp * r;     // tangential offset (arc-length approx)
                float dz = p.Z - _zC; // axial offset
                // Channel 1: forward pass (z negative side)
                float w1 = _ch * 0.12f, h1 = (_rT - _rH) * 0.35f;
                float d1 = BoxSdf2D(da + _th * 0.25f, dz - _ch * 0.05f, w1, h1);
                // Channel 2: return pass (z positive side)
                float d2 = BoxSdf2D(da - _th * 0.25f, dz + _ch * 0.05f, w1, h1);
                // Channel 3: trailing edge slot
                float d3 = BoxSdf2D(da, dz, _th * 0.08f, _ch * 0.35f);
                // Film holes: small circular holes on pressure surface
                // Modelled as spheres spaced axially
                float dFilm = 1000f;
                int nHoles = 5;
                for (int j = 0; j < nHoles; j++)
                {
                    float zH = _zC - _ch * 0.3f + j * _ch * 0.12f;
                    float filmR = 0.3f * _th;   // hole radius 0.3mm nominal
                    float distToHole = MathF.Sqrt(da * da + (p.Z - zH) * (p.Z - zH)) - filmR;
                    if (distToHole < dFilm) dFilm = distToHole;
                }
                float bladeDist = MathF.Min(MathF.Min(d1, d2), MathF.Min(d3, dFilm));
                if (bladeDist < bestDist) bestDist = bladeDist;
            }
            return -bestDist; // negative = inside (void)
        }
        static float BoxSdf2D(float x, float y, float hw, float hh)
        {
            float qx = MathF.Max(MathF.Abs(x) - hw, 0f);
            float qy = MathF.Max(MathF.Abs(y) - hh, 0f);
            return MathF.Sqrt(qx * qx + qy * qy) - 0.5f;
        }
    }

    public class SdfPreSwirlSlots : IImplicit
    {
        readonly float _rInner, _zFace, _slotH, _slotW, _swirl_angle_rad;
        readonly int   _nSlots;
        public SdfPreSwirlSlots(float rInner, float zFace, float slotHeight,
                                 float slotWidth, float swirlAngleDeg, int nSlots)
        {
            _rInner = rInner; _zFace = zFace; _slotH = slotHeight;
            _slotW  = slotWidth; _nSlots = nSlots;
            _swirl_angle_rad = swirlAngleDeg * MathF.PI / 180f;
        }
        public float fSignedDistance(in Vector3 p)
        {
            float r = MathF.Sqrt(p.X * p.X + p.Y * p.Y);
            if (r < _rInner - _slotH * 2f || r > _rInner + _slotH ||
                p.Z < _zFace - _slotW     || p.Z > _zFace + _slotW) return 1000f;
            float phi = MathF.Atan2(p.Y, p.X);
            float sec = 2f * MathF.PI / _nSlots;
            float bestDist = 1000f;
            for (int i = 0; i < _nSlots; i++)
            {
                float dp = phi - i * sec;
                while (dp >  sec / 2) dp -= sec;
                while (dp < -sec / 2) dp += sec;
                float da  = dp * r; // tangential offset
                float dr  = r - _rInner;
                // Rotate slot coordinate by swirl angle in (da, dr) plane
                float da_rot = da * MathF.Cos(_swirl_angle_rad) - dr * MathF.Sin(_swirl_angle_rad);
                float dr_rot = da * MathF.Sin(_swirl_angle_rad) + dr * MathF.Cos(_swirl_angle_rad);
                float dz    = MathF.Abs(p.Z - _zFace) - _slotW / 2f;
                float slotDist = MathF.Max(
                    MathF.Max(MathF.Abs(da_rot) - _slotW / 2f, dz),
                    MathF.Abs(dr_rot) - _slotH);
                if (slotDist < bestDist) bestDist = slotDist;
            }
            return bestDist;
        }
    }

    public class SdfBalancingBoss : IImplicit
    {
        readonly float _rBoss, _zFace, _bossH, _bossW;
        readonly int   _nBoss;
        public SdfBalancingBoss(float rBoss, float zFace, float bossHeight,
                                 float bossWidth, int nBoss)
        { _rBoss = rBoss; _zFace = zFace; _bossH = bossHeight; _bossW = bossWidth; _nBoss = nBoss; }
        public float fSignedDistance(in Vector3 p)
        {
            float r = MathF.Sqrt(p.X * p.X + p.Y * p.Y);
            if (MathF.Abs(r - _rBoss) > _bossW * 2f ||
                MathF.Abs(p.Z - _zFace) > _bossH * 3f) return 1000f;
            float phi = MathF.Atan2(p.Y, p.X);
            float sec = 2f * MathF.PI / _nBoss;
            float bestDist = 1000f;
            for (int i = 0; i < _nBoss; i++)
            {
                float dp = phi - i * sec;
                while (dp >  sec / 2) dp -= sec;
                while (dp < -sec / 2) dp += sec;
                float da  = dp * r;  // tangential arc-length
                float dr  = MathF.Abs(r - _rBoss) - _bossW / 2f;       // radial extent
                float dz  = MathF.Abs(p.Z - _zFace) - _bossH / 2f;    // axial height
                float dtan= MathF.Abs(da) - _bossW / 2f;               // tangential width
                float bDist = MathF.Max(MathF.Max(dtan, dr), dz);
                if (bDist < bestDist) bestDist = bDist;
            }
            return bestDist;
        }
    }

    public class SdfFirTreeRow : IImplicit
    {
        readonly float _rH, _zC, _ch, _w, _d, _p;
        readonly int _n, _nt;

        public SdfFirTreeRow(float rHub, float zCenter, float chord, float width, float toothD, int numTeeth, float pitch, int count)
        {
            _rH = rHub; _zC = zCenter; _ch = chord; _w = width; _d = toothD; _nt = numTeeth; _p = pitch; _n = count;
        }

        public float fSignedDistance(in Vector3 p)
        {
            float r = MathF.Sqrt(p.X * p.X + p.Y * p.Y);
            if (r < _rH - 40f || r > _rH + 10f) return 1000f;
            
            float phi = MathF.Atan2(p.Y, p.X);
            float sec = 2f * MathF.PI / _n;
            float dp = phi % sec;
            if (dp > sec / 2) dp -= sec;
            if (dp < -sec / 2) dp += sec;
            
            float localX = dp * r;
            float localY = r; 
            float localZ = p.Z;
            
            float dZ = Math.Abs(localZ - _zC) - _ch / 2f;
            float dY = Math.Max((_rH - 30f) - localY, localY - _rH);
            
            float xVal = Math.Abs(localX);
            float yLocal = localY - (_rH - 30f);
            float toothProfile = _w / 2f;
            if (yLocal > 0 && yLocal < 30f)
            {
                float yCycle = yLocal % _p;
                if (yCycle < _p * 0.6f)
                {
                    toothProfile += _d;
                }
            }
            
            float dX = xVal - toothProfile;
            return Math.Max(dX, Math.Max(dY, dZ));
        }
    }

    public class SdfSpline : IImplicit
    {
        readonly float _zStart, _zEnd, _rIn, _rOut, _toothD;
        readonly int _numTeeth;

        public SdfSpline(float zStart, float zEnd, float rIn, float rOut, float toothD, int numTeeth)
        {
            _zStart = zStart; _zEnd = zEnd; _rIn = rIn; _rOut = rOut; _toothD = toothD; _numTeeth = numTeeth;
        }

        public float fSignedDistance(in Vector3 p)
        {
            float dZ = Math.Max(_zStart - p.Z, p.Z - _zEnd);
            float r = MathF.Sqrt(p.X * p.X + p.Y * p.Y);
            
            float phi = MathF.Atan2(p.Y, p.X);
            float sec = 2f * MathF.PI / _numTeeth;
            float dp = phi % sec;
            if (dp > sec / 2) dp -= sec;
            if (dp < -sec / 2) dp += sec;
            
            float rBound = _rOut;
            if (Math.Abs(dp * r) < (sec * r * 0.4f))
            {
                rBound += _toothD;
            }
            
            float dR = Math.Max(_rIn - r, r - rBound);
            return Math.Max(dR, dZ);
        }
    }

}
