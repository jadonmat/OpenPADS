cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project(OpenPADS VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
find_package(OpenGL REQUIRED)

# ------------------------------------------------------------------------------
# Dependencies via FetchContent
# ------------------------------------------------------------------------------
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)

set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG        v2.0.8
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  cmake
)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8-docking
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(glfw glm glad imgui)

glad_add_library(glad_gl_core_45 STATIC API gl:core=4.5)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# ------------------------------------------------------------------------------
# OpenPADS_Core (pure C++20 parametric aircraft & geometry kernel)
# ------------------------------------------------------------------------------
add_library(OpenPADS_Core STATIC
    src/core/Transform.hpp
    src/core/Transform.cpp
    src/core/Parameter.hpp
    src/core/Parameter.cpp
    src/core/Mesh.hpp
    src/core/Mesh.cpp
    src/core/Airfoil.hpp
    src/core/Airfoil.cpp
    src/core/Component.hpp
    src/core/Component.cpp
    src/core/WingSection.hpp
    src/core/WingSection.cpp
    src/core/Wing.hpp
    src/core/Wing.cpp
    src/core/Aircraft.hpp
    src/core/Aircraft.cpp
)
target_include_directories(OpenPADS_Core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(OpenPADS_Core PUBLIC glm::glm)
target_compile_definitions(OpenPADS_Core PUBLIC OPENPADS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

# ------------------------------------------------------------------------------
# OpenPADS_Render (OpenGL 4.5 3D viewport backend)
# ------------------------------------------------------------------------------
add_library(OpenPADS_Render STATIC
    src/render/OrbitCamera.hpp
    src/render/OrbitCamera.cpp
    src/render/Shader.hpp
    src/render/Shader.cpp
    src/render/MeshBuffer.hpp
    src/render/MeshBuffer.cpp
    src/render/GridRenderer.hpp
    src/render/GridRenderer.cpp
    src/render/Viewport3D.hpp
    src/render/Viewport3D.cpp
)
target_include_directories(OpenPADS_Render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(OpenPADS_Render PUBLIC
    OpenPADS_Core
    glad_gl_core_45
    glm::glm
    OpenGL::GL
)

# ------------------------------------------------------------------------------
# OpenPADS_UI (Dockable CAD editor UI)
# ------------------------------------------------------------------------------
add_library(OpenPADS_UI STATIC
    src/ui/Panel.hpp
    src/ui/Panel.cpp
    src/ui/ModelTreePanel.hpp
    src/ui/ModelTreePanel.cpp
    src/ui/InspectorPanel.hpp
    src/ui/InspectorPanel.cpp
    src/ui/ViewportPanel.hpp
    src/ui/ViewportPanel.cpp
    src/ui/Application.hpp
    src/ui/Application.cpp
)
target_include_directories(OpenPADS_UI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(OpenPADS_UI PUBLIC
    OpenPADS_Core
    OpenPADS_Render
    imgui
    glfw
)

# ------------------------------------------------------------------------------
# OpenPADS Executable
# ------------------------------------------------------------------------------
add_executable(OpenPADS
    src/main.cpp
)
target_link_libraries(OpenPADS PRIVATE OpenPADS_UI)

if (MSVC)
    foreach(target_name OpenPADS_Core OpenPADS_Render OpenPADS_UI OpenPADS)
        target_compile_options(${target_name} PRIVATE /W4 /permissive-)
        target_compile_definitions(${target_name} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endforeach()
else()
    foreach(target_name OpenPADS_Core OpenPADS_Render OpenPADS_UI OpenPADS)
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()

# ------------------------------------------------------------------------------
# Unit Tests
# ------------------------------------------------------------------------------
option(OPENPADS_BUILD_TESTS "Build OpenPADS unit tests" ON)
if (OPENPADS_BUILD_TESTS)
    enable_testing()
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.8.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(OpenPADS_Tests
        tests/CoreTests.cpp
    )
    target_link_libraries(OpenPADS_Tests PRIVATE
        OpenPADS_Core
        Catch2::Catch2WithMain
    )
    add_test(NAME OpenPADS_CoreTests COMMAND OpenPADS_Tests)
endif()
