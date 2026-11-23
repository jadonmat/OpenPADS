# OpenPADS

**Open Parametric Aircraft Design Suite** — a modern C++20 aircraft design, parametric geometry modeling, and aerodynamic/flight-simulation workbench.

> See **[PLAN.md](PLAN.md)** for the full 6-phase roadmap (Geometry Expansion → Aero Solvers → SU2 CFD → Aircraft Analysis → Flight Sim) and **[ARCHITECTURE.md](ARCHITECTURE.md)** for the OOP class hierarchy and frame data flow.

---

## Features (Phase 2 Complete)

- **Clean OOP Layering:** Pure C++20 core model (`OpenPADS_Core`) decoupled from OpenGL rendering (`OpenPADS_Render`) and the dockable editor (`OpenPADS_UI`).
- **Full Aircraft Geometry Suite:**
  - Multi-bay trapezoidal `Wing` with trailing-edge **Control Surfaces** (ailerons, flaps, symmetric/differential deflection), `XZ` symmetry, watertight tip caps, and live $S, b, AR, c_{\text{MAC}}, S_{\text{wet}}$.
  - Multi-station `Fuselage` (`Point`, `Circle`, `Ellipse`, `SuperEllipse`, `RoundedRect`) with monotonic PCHIP smooth lofting, end capping, `Fineness Ratio`, `Wetted Area`, and `Volume`.
  - Parametric **`Pod` / Nacelle**, **`Propeller` / Rotor** (multi-blade twisted airfoil + spinner + disk area/solidity), and **`LandingGear`** (strut, oleo, single/dual wheels, wheel pant, `XZ` mirror).
- **Importers & Part Duplication:** Native **OpenVSP (`.vsp3`)** XML importer (`Wing`, `Fuselage`, `Pod`, `Propeller`), **STL (ASCII/Binary) & Wavefront OBJ** importer with live unit scaling & `Y-Up → Z-Up` conversion, and one-click `Duplicate`.
- **Interactive 3D Viewport Modeler:** Left-click 3D ray picking (automatically selects the nearest wing or fuselage cross-section), `Part` / `XSec (Tab)` target toggle, `Select (Q)` / `Move (W)` / `Rotate (R)` / `Scale (S)` 3D gizmos (chord, thickness, twist, width/height, span/sweep/dihedral directly in 3D), **Geometry Snap** (mesh-to-mesh contact & feature snapping between parts), and full `Ctrl+Z` / `Ctrl+Y` Undo/Redo history.
- **Dedicated Airfoil Editor (`ImPlot`):** Interactive 2D airfoil/camber plot, NACA 4/5-digit generators, searchable 49-airfoil UIUC `.dat` library (`data/airfoils/`) with `.dat` file upload, CST Bernstein sculpting with draggable control points, and `.dat` export.
- **Project Persistence & CAD Mesh Export:** JSON save/load (`.opads`) via `nlohmann/json` plus world-space **Binary STL** and **Wavefront OBJ** export.
- **Dockable CAD Workspace:** Built with `GLFW`, `OpenGL 4.5` (`GLAD`), `Dear ImGui` (Docking), and `ImPlot`.

### Viewport Controls

| Action | Input |
| :--- | :--- |
| Select Component / XSec | Left-click in 3D viewport (`Q`) |
| Toggle Part / XSec Target | `Tab` (`Part` vs `XSec` gizmo mode) |
| 3D Move / Rotate / Scale | `W` (Move) / `R` (Rotate / Twist) / `S` (Scale / Chord & Thickness) |
| Undo / Redo | `Ctrl+Z` / `Ctrl+Y` (or `Edit` menu / toolbar) |
| Geometry Snap Between Parts | Toggle **Geometry Snap** while dragging `Move (W)` center or axes |
| Orbit | Right-drag (or `Alt` + Left-drag) |
| Pan | Middle-drag (or `Shift` + Right-drag) |
| Zoom | Mouse Wheel |
| Camera Presets | `Iso`, `Top`, `Front`, `Side`, `Reset` toolbar buttons |

---

## Building (Windows / MSVC)

Dependencies (`glfw`, `glm`, `glad`, `imgui`, `Catch2`) are fetched automatically via CMake `FetchContent`.

```powershell
# From a Visual Studio Developer Command Prompt (x64)
cmake --preset x64-debug
cmake --build out/build/x64-debug
ctest --test-dir out/build/x64-debug --output-on-failure
```

Run the editor:

```powershell
.\out\build\x64-debug\OpenPADS.exe
```

---

## Phase 1 Roadmap

- [x] **Milestone 1:** CMake architecture, dockable UI shell, offscreen 3D viewport, `OrbitCamera`, and reference grid
- [x] **Milestone 2:** Parametric `Airfoil` (NACA 4/5-digit + Selig/Lednicer `.dat` library), `Parameter`, and multi-section `Wing` lofting with unified cross-section inspector
- [x] **Milestone 3:** Interactive 3D viewport modeler (click-to-select & 3D drag gizmos) and dedicated `ImPlot` Airfoil Editor (with CST sculpting & `.dat` export)
- [x] **Milestone 4:** Parametric `Fuselage` lofting (PCHIP smooth stations), JSON project save/load (`.opads`), and Binary STL / OBJ export
