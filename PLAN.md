# OpenPADS Development Roadmap (`PLAN.md`)

**Open Parametric Aircraft Design Suite (OpenPADS)** — Full C++20 aircraft conceptual/preliminary design, geometry modeling, aerodynamics, CFD, and flight-simulation platform.

---

## Phase 1 — Core CAD UI & Parametric Modeler *(Complete)*
- Pure C++20 `OpenPADS_Core` library decoupled from OpenGL (`OpenPADS_Render`) and `Dear ImGui` / `ImPlot` (`OpenPADS_UI`).
- Aerospace Z-up 3D viewport (`X` aft, `Y` starboard, `Z` up) with procedural ground grid, axis triad, ray picking, and `Part` / `XSec` 3D `Move / Rotate / Scale` gizmos.
- Geometry-based surface & feature snapping (`GeometrySnap`) and snapshot `UndoStack` (`Ctrl+Z` / `Ctrl+Y`).
- Parametric multi-bay `Wing` (trapezoidal bays, chained cross-sections) and `Fuselage` (`Point`, `Circle`, `Ellipse`, `SuperEllipse`, `RoundedRect` with monotonic PCHIP lofting).
- `Airfoil` engine (NACA 4/5-digit, 49-airfoil UIUC `.dat` library, `.dat` upload/export, Kulfan CST sculpting in `ImPlot`).
- JSON project persistence (`.opads`) and world-space Binary STL / Wavefront OBJ export.

---

## Phase 2 — 3D Geometry Expansion & Importers *(Complete)*
- **Wing Control Surfaces:** Span/chord-fraction trailing-edge surfaces (ailerons, flaps, elevators, rudders) with live 3D deflection visualization and symmetric/antisymmetric pairing.
- **Nacelles & Pods (`Pod`):** Parametric axisymmetric / super-ellipse pods, engine nacelles, fuel tanks, and external stores with optional inlet/exhaust cavities.
- **Propellers & Rotors (`Propeller`):** Multi-blade parametric rotors with radial chord/twist/airfoil distributions, hub spinner, and collective pitch.
- **Landing Gear (`LandingGear`):** Struts, oleo pistons, forks, wheels/tires, optional wheel pants, steering/rake angles, and `XZ` symmetry for main gear.
- **Part Duplication & Mirroring:** One-click duplicate / mirror across `XZ` in the Model Tree.
- **CAD / OpenVSP Importers:**
  - **OpenVSP (`.vsp3`) XML Importer:** Parses OpenVSP `WingGeom`, `FuselageGeom`, `PodGeom`, and `PropellerGeom` natively into parametric OpenPADS components.
  - **Mesh Importer (`.stl` ASCII/Binary, `.obj`):** Automatic unit conversion (`m`, `cm`, `mm`, `ft`, `in`), axis orientation mapping, normal generation, and full `Transform` / `GeometrySnap` / re-export support (`ImportedMesh`).

---

## Phase 3 — Aerodynamic Solvers & Airfoil Polars
- 2D airfoil viscous/inviscid boundary-layer analysis (XFOIL-class polar sweeps: $C_l(\alpha), C_d(C_l), C_m(\alpha)$ across $Re, M$).
- 3D Vortex Lattice Method (VLM) and 3D Panel Method solvers operating directly on `Wing`, `ControlSurface`, and `Fuselage` geometry.
- Live 3D $\Delta C_p$ / spanwise lift loading overlays in `Viewport3D` and interactive polar comparison plots in `ImPlot`.

---

## Phase 4 — Full SU2 CFD Integration
- Automated watertight surface/far-field mesh preparation and boundary marker tagging (`Wing`, `Fuselage`, `ControlSurface`, symmetry plane).
- SU2 `.cfg` case generator (Euler / RANS / SST / SA turbulence, Mach/AoA sweeps, multipoint runs).
- Process runner with live residual/force convergence plotting in `ImPlot` and 3D surface $C_p / C_f$ contour visualization.

---

## Phase 5 — Aircraft Analysis, Drag Buildup, & Stability & Control
- Component-by-component **parasite drag buildup** ($C_{D_0}$ skin-friction, Reynolds/Mach compressibility, form factors, interference, excrescence, and landing-gear/store drag).
- **Mass, CG & Inertia** buildup (component mass properties, fuel/payload states, CG envelope).
- **Static & Dynamic Stability & Control:** Neutral point, static margin, control power ($C_{m_{\delta_e}}, C_{l_{\delta_a}}, C_{n_{\delta_r}}$), trim diagrams, and linearized longitudinal/lateral-directional eigenmodes.
- **Mission & Performance Analysis:** $L/D_{\max}$, range/endurance (Breguet), $V$-$n$ structural/flight envelope, climb rate, ceiling, takeoff, and landing distance.

---

## Phase 6 — Flight Simulator Integration
- 6-DOF nonlinear equations of motion and actuator/control-surface mixing fed directly from Phase 3–5 aero tables, propulsion models, and inertia tensors.
- Integration bridge with prior C++ flight simulator code for real-time piloted / autopilot flight testing directly from OpenPADS aircraft designs.
