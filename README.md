# OpenPADS

**Open Parametric Aircraft Design Suite** — a modern C++20 aircraft design, parametric geometry modeling, and aerodynamic/flight-simulation workbench.

> For a full breakdown of the OOP class hierarchy, module boundaries, and frame data flow, see **[ARCHITECTURE.md](ARCHITECTURE.md)**.

---

## Features (Milestone 3)

- **Clean OOP Layering:** Pure C++20 core model (`OpenPADS_Core`) decoupled from OpenGL rendering (`OpenPADS_Render`) and the dockable editor (`OpenPADS_UI`).
- **Interactive 3D Viewport Modeler:** Left-click 3D ray picking (automatically selects the nearest wing cross-section), `Select (Q)` / `Move (W)` / `XSec (E)` 3D axis manipulation gizmos to drag components or reshape span/sweep/dihedral directly in 3D.
- **Dedicated Airfoil Editor (`ImPlot`):** Interactive 2D airfoil/camber plot, NACA 4/5-digit generators, Selig/Lednicer `.dat` library (`data/airfoils/`), CST Bernstein sculpting with draggable control points, and `.dat` export.
- **Parametric Wing & Cross-Section Lofting:** Multi-bay trapezoidal `Wing` components (`Span`, `Root/Tip Chord`, `Sweep @ fraction`, `Dihedral`, `Twist`) with `XZ` symmetry, watertight tip caps, live aero metrics ($S, b, AR, c_{\text{MAC}}, S_{\text{wet}}$), and 3D cross-section ring highlighting.
- **Dockable CAD Workspace:** Built with `GLFW`, `OpenGL 4.5` (`GLAD`), `Dear ImGui` (Docking), and `ImPlot` featuring a **Model Tree**, **3D Viewport**, **Airfoil Editor**, and unified **Inspector**.

### Viewport Controls

| Action | Input |
| :--- | :--- |
| Select Component / XSec | Left-click in 3D viewport (`Q`) |
| 3D Move / XSec Handle | `W` (Move component) / `E` (Drag XSec Span/Sweep/Dihedral) |
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
- [ ] **Milestone 4:** Parametric `Fuselage` lofting, JSON project save/load, and STL/OBJ export
