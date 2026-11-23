# OpenPADS Development Roadmap (`PLAN.md`)

**Open Parametric Aircraft Design Suite (OpenPADS)** — Full C++20 aircraft conceptual/preliminary design, geometry modeling, aerodynamics, sizing & stability, 6-DOF mission simulation, and CFD validation platform.

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
- **Wing Control Surfaces:** Span/chord-fraction trailing-edge surfaces (ailerons, flaps, elevators, rudders) with live 3D deflection visualization and symmetric/differential pairing.
- **Nacelles & Pods (`Pod`):** Parametric axisymmetric / super-ellipse pods, engine nacelles, fuel tanks, and external stores.
- **Propellers & Rotors (`Propeller`):** Multi-blade parametric rotors with radial chord/twist/airfoil distributions, hub spinner, collective pitch, and solidity metrics.
- **Landing Gear (`LandingGear`):** Struts, oleo pistons, single/dual wheels, optional wheel pants, rake/splay angles, and `XZ` symmetry for main gear.
- **Part Duplication & Mirroring:** One-click `Duplicate` in the Model Tree.
- **CAD / OpenVSP Importers:**
  - **OpenVSP (`.vsp3`) XML Importer:** Parses OpenVSP `WingGeom`, `FuselageGeom`, `PodGeom`, `PropellerGeom`, and `Gear` bogies natively into parametric OpenPADS components (with `LenUnit` conversion).
    - *Known Issues / Deferred Fix:* Geometry fidelity still has edge cases likely tied to OpenPADS's own lofting/geometry representation limitations (e.g., collapsed noses when tangent/end angles aren't modeled in PCHIP lofting, flat-looking propeller blades, and flat tail cross-sections on `GeneralFuse`/`RoundedRect` conversions) — revisit when upgrading fuselage/propeller lofting geometry solvers.
  - **Mesh Importer (`.stl` ASCII/Binary, `.obj`):** Automatic unit scaling (`m`, `mm`, `in`, `ft`), `Y-Up → Z-Up` axis conversion, origin centering, and full `Transform` / `GeometrySnap` / re-export support (`ImportedMesh`).

---

## Phase 3 — Aerodynamic Solvers & Airfoil Polars
- 2D airfoil viscous/inviscid boundary-layer analysis (XFOIL-class polar sweeps: $C_l(\alpha), C_d(C_l), C_m(\alpha)$ across $Re, M$).
- 3D Vortex Lattice Method (VLM) and 3D Panel Method solvers operating directly on `Wing`, `ControlSurface`, and `Fuselage` geometry.
- Live 3D $\Delta C_p$ / spanwise lift loading overlays in `Viewport3D` and interactive polar comparison plots in `ImPlot`.

---

## Phase 4 — Aircraft Analysis, Sizing & Stability *(Pure `OpenPADS_Core`)*
Moved ahead of SU2 so a complete, fast conceptual design loop is immediately available right after Phase 3's VLM and airfoil polars:
- **Weight, Balance & Inertia:** Component-level mass buildup (structure, propulsion, payload, fuel tanks), live CG marker in the 3D viewport, CG travel envelope (full fuel $\rightarrow$ empty), and full $3 \times 3$ inertia tensor ($I_{xx}, I_{yy}, I_{zz}, I_{xz}$) computed dynamically as fuel burns.
- **Complete Drag Buildup:** Wetted-area skin-friction ($C_f$) with laminar/turbulent transition, component form factors, interference drag, and excrescence allowance—merged with Phase 3's VLM induced drag to produce full aircraft $C_L$–$C_D$ polars.
- **Propulsion Models:** Parametric thrust/power and SFC lookup tables (turbofan, turboprop, piston/electric) as a function of altitude, Mach, and throttle.
- **Stability & Control:** Control-surface effectiveness (ailerons, elevator, rudder, flaps), neutral point, static margin, trim curves ($\alpha_{\text{trim}}, \delta_{e,\text{trim}}$), and linearized state-space eigenmode analysis (short period, phugoid, Dutch roll, roll subsidence, spiral) plotted in `ImPlot`.

---

## Phase 5 — Hybrid Mission Profile & 6-DOF Maneuver Sequencer
Deterministic, self-contained engineering testbed operating directly on live `OpenPADS_Core` data:
- **Unified `AircraftState` & Aero Database:** Pulls aero tables, control derivatives, propulsion curves, and instantaneous fuel-dependent mass/CG/inertia straight from Phases 3 and 4 in memory (zero XML round-tripping).
- **Macro Mission Timeline (3-DOF Energy-State):** Drag-and-drop segment stack (`Takeoff`, `Climb`, `Cruise`, `Loiter`, `Descent`, `Landing`) that steps through range, time, altitude, and fuel burn in milliseconds while continuously updating mass, CG, and inertia.
- **Embedded 6-DOF Maneuver Blocks:** Insertable at any point in the mission timeline (e.g., engine-out right after takeoff, elevator doublet at cruise, rudder kick on light-weight approach). Automatically trims the aircraft at the exact handoff state from the preceding leg, runs a quaternion + RK4 6-DOF integration, logs full time histories, and hands control back to the next mission leg.
- **Telemetry, A/B Overlays & 3D Replay:** Multi-axis `ImPlot` graphs ($\alpha, \beta$, pitch/roll/yaw rates, $n_z$, airspeed, altitude), baseline-vs-modified overlay mode for instant design comparison, and a timeline scrubber that animates the 3D aircraft mesh flying any 6-DOF maneuver inside the OpenGL viewport.
- **External Exporters (Optional):** One-click export of the generated aero/inertia database to **JSBSim XML** (for FlightGear/Unreal/PX4 validation) or a clean JSON / Luau table.

---

## Phase 6 — High-Fidelity SU2 CFD Validation Pipeline
Isolated as the final phase so 3D boolean geometry and volume meshing never bottleneck the conceptual/preliminary design loop:
- **Watertight Geometry Prep:** Integrate the `manifold` C++ library (`vcpkg`) to run clean boolean unions on intersecting components (wing-fuselage, tail-fuselage) and export watertight `.stl` surfaces.
- **Automated Meshing Bridge:** Link the `Gmsh` C++ API to auto-generate 2D airfoil meshes and 3D farfield unstructured volume meshes (`.su2`) with named boundary markers (`WALL`, `FARFIELD`, `SYMMETRY`).
- **Native SU2 Runner:** Spawn precompiled Windows `SU2_CFD.exe` (`v8.6.0`) as an async background process using auto-generated `.cfg` files, streaming live residual convergence and $C_L / C_D / C_m$ history directly into `ImPlot`.
- **Tiered CFD Execution & Visualization:**
  - *2D Airfoil RANS (Airfoil Editor tab):* Fast viscous polars and stall validation in seconds.
  - *3D Inviscid Euler:* Fast compressible/transonic lift and wave-drag checks on a desktop CPU.
  - *3D RANS Case Generator / Runner:* Full boundary-layer prism mesh + `.cfg` export for high-fidelity runs, plus a surface $C_p$ colormap overlay rendered directly on the mesh in the OpenGL viewport.
