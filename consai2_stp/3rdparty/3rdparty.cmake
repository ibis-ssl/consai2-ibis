set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

find_package(X11 REQUIRED)

if (NOT X11_Xi_FOUND)
  message(FATAL_ERROR "X11 Xi library is required")
endif ()

include(3rdparty/glad/glad.cmake)
include(3rdparty/glfw/glfw.cmake)
include(3rdparty/imgui/imgui.cmake)
