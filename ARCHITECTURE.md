# OpenPADS Architecture & Class Hierarchy

OpenPADS separates **parametric aircraft data**, **3D rendering**, and **editor UI** into three static libraries linked by the `OpenPADS` executable. This keeps the geometry and engineering kernel (`OpenPADS_Core`) completely free of OpenGL/windowing dependencies so future aerodynamic solvers, optimizers, and flight-simulation modules can link it directly.

---

## Module Dependency Flow

```mermaid
flowchart LR
    Core["OpenPADS_Core (Pure C++20 + GLM)"]
    Render["OpenPADS_Render (OpenGL 4.5 + GLAD)"]
    UI["OpenPADS_UI (GLFW + Dear ImGui)"]
    App["OpenPADS Executable (main.cpp)"]
    Tests["OpenPADS_Tests (Catch2)"]

    Render --> Core
    UI --> Render
    UI --> Core
    App --> UI
    Tests --> Core
```

---

## Class Hierarchy & Ownership

```mermaid
classDiagram
    direction TB

    namespace OpenPADS_Core {
        class Aircraft {
            -string m_name
            -uint64 m_nextComponentId
            -optional~uint64~ m_selectedComponentId
            -vector~unique_ptr~Component~~ m_components
            +addComponent(unique_ptr~Component~) Component*
            +removeComponent(uint64 id) bool
            +findComponent(uint64 id) Component*
            +rebuildDirty() void
        }

        class Component {
            <<abstract>>
            #Mesh m_mesh
            -uint64 m_id
            -string m_name
            -bool m_visible
            -bool m_dirty
            -Transform m_transform
            +ensureBuilt() void
            +getTypeName()* string
            #rebuildMesh()* void
        }

        class Transform {
            -vec3 m_position
            -vec3 m_rotationDeg
            -vec3 m_scale
            +getMatrix() mat4
        }

        class Mesh {
            -vector~Vertex~ m_vertices
            -vector~uint32~ m_indices
            +addVertex(vec3 pos, vec3 normal) void
            +addTriangle(uint32 i0, uint32 i1, uint32 i2) void
        }
    }

    namespace OpenPADS_Render {
        class Viewport3D {
            -uint32 m_fbo
            -uint32 m_colorTexture
            -OrbitCamera m_camera
            -GridRenderer m_gridRenderer
            -Shader m_surfaceShader
            -unordered_map~uint64, MeshBuffer~ m_componentBuffers
            +initialize() bool
            +resize(int w, int h) void
            +render(Aircraft& aircraft) void
        }

        class OrbitCamera {
            -vec3 m_target
            -float m_distance
            -float m_yawDeg
            -float m_pitchDeg
            +orbit(float dYaw, float dPitch) void
            +pan(float dRight, float dUp) void
            +zoom(float steps) void
            +setPresetView(ViewPreset preset) void
            +getViewMatrix() mat4
            +getProjectionMatrix(float aspect) mat4
        }

        class GridRenderer {
            -Shader m_gridShader
            -Shader m_axisShader
            +initialize() bool
            +render(mat4 viewProj, vec3 camPos, vec3 target, bool grid, bool axes) void
        }

        class MeshBuffer {
            -uint32 m_vao
            -uint32 m_vbo
            -uint32 m_ebo
            +upload(const Mesh& mesh) void
            +draw() void
        }

        class Shader {
            -uint32 m_programId
            +compileFromSource(string_view vs, string_view fs) bool
            +bind() void
        }
    }

    namespace OpenPADS_UI {
        class Application {
            -GLFWwindow* m_window
            -Aircraft m_aircraft
            -Viewport3D m_viewport
            -vector~unique_ptr~Panel~~ m_panels
            +initialize() bool
            +run() int
        }

        class Panel {
            <<abstract>>
            #string m_title
            #bool m_open
            +onRender()* void
        }

        class ModelTreePanel {
            -Aircraft& m_aircraft
            +onRender() void
        }

        class InspectorPanel {
            -Aircraft& m_aircraft
            +onRender() void
        }

        class ViewportPanel {
            -Aircraft& m_aircraft
            -Viewport3D& m_viewport
            +onRender() void
        }
    }

    Aircraft *-- "0..*" Component : owns
    Component *-- "1" Transform : has
    Component *-- "1" Mesh : generates

    Application *-- "1" Aircraft : owns
    Application *-- "1" Viewport3D : owns
    Application *-- "0..*" Panel : owns

    Panel <|-- ModelTreePanel
    Panel <|-- InspectorPanel
    Panel <|-- ViewportPanel

    ModelTreePanel --> Aircraft : edits selection/visibility
    InspectorPanel --> Aircraft : edits properties
    ViewportPanel --> Viewport3D : displays FBO & forwards input
    ViewportPanel --> Aircraft : passes to render

    Viewport3D *-- "1" OrbitCamera : owns
    Viewport3D *-- "1" GridRenderer : owns
    Viewport3D *-- "1" Shader : surface shader
    Viewport3D *-- "0..*" MeshBuffer : caches by component ID
    Viewport3D ..> Aircraft : reads dirty state & meshes
    GridRenderer *-- "2" Shader : grid & axis shaders
```

---

## Per-Frame Update Flow

```mermaid
flowchart TD
    Poll["GLFW Poll Events & New ImGui Frame"] --> Dock["Application renders Dockspace & MenuBar"]
    Dock --> Tree["ModelTreePanel updates selection & visibility"]
    Dock --> Insp["InspectorPanel edits Transform / parameters -> markDirty()"]
    Dock --> VP["ViewportPanel resizes & calls Viewport3D::render(aircraft)"]
    VP --> Sync["Viewport3D::syncComponentMeshes() rebuilds dirty Components & uploads MeshBuffers"]
    Sync --> Draw["Viewport3D renders Components + GridRenderer into offscreen FBO"]
    Draw --> Display["ViewportPanel draws FBO texture via ImGui::Image & handles camera drag/zoom"]
    Display --> Present["ImGui::Render & glfwSwapBuffers"]
```

---

## File Map

| Module | File | Responsibility |
| :--- | :--- | :--- |
| **Core** | `src/core/Transform.hpp/.cpp` | Aerospace Z-up position, Euler rotation, scale, and model matrix |
| **Core** | `src/core/Mesh.hpp/.cpp` | Indexed triangle mesh (`Vertex` positions + normals) |
| **Core** | `src/core/Component.hpp/.cpp` | Abstract base class for aircraft components (`Wing`, `Fuselage`, etc.) |
| **Core** | `src/core/Aircraft.hpp/.cpp` | Root container owning components, IDs, selection, and dirty rebuilds |
| **Render** | `src/render/OrbitCamera.hpp/.cpp` | Spherical orbit/pan/zoom camera and CAD view presets |
| **Render** | `src/render/Shader.hpp/.cpp` | RAII OpenGL shader compiler and uniform helpers |
| **Render** | `src/render/MeshBuffer.hpp/.cpp` | GPU `VAO`/`VBO`/`EBO` buffer lifecycle for `Mesh` |
| **Render** | `src/render/GridRenderer.hpp/.cpp` | Procedural anti-aliased `Z = 0` grid and origin RGB triad |
| **Render** | `src/render/Viewport3D.hpp/.cpp` | Offscreen FBO renderer, lighting shader, and mesh cache |
| **UI** | `src/ui/Panel.hpp/.cpp` | Abstract base class for dockable UI panels |
| **UI** | `src/ui/ModelTreePanel.hpp/.cpp` | Hierarchy outliner for `Aircraft` and `Component` items |
| **UI** | `src/ui/InspectorPanel.hpp/.cpp` | Property editor for selected aircraft/component transforms & metrics |
| **UI** | `src/ui/ViewportPanel.hpp/.cpp` | Hosts `Viewport3D` image, view toolbar, and mouse camera controls |
| **UI** | `src/ui/Application.hpp/.cpp` | Window, OpenGL context, Dear ImGui dockspace, theme, and main loop |
| **App** | `src/main.cpp` | Program entry point |
| **Tests** | `tests/CoreTests.cpp` | Catch2 unit tests for `Transform`, `Component`, and `Aircraft` |
