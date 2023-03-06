cmake_minimum_required(VERSION 3.24)
project(GamePhysicsWeekend)

set(CMAKE_CXX_STANDARD 17)

# glfw3
option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(3rdparty/glfw)

# vulkan
find_package(Vulkan REQUIRED)

add_library(vulkan STATIC IMPORTED)
set_target_properties(vulkan PROPERTIES
        IMPORTED_LOCATION "${Vulkan_LIBRARIES}"
)
set_target_properties(vulkan PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Vulkan_INCLUDE_DIR}"
)

file(GLOB_RECURSE BOOK_SRC
        Math/*.cpp
        Math/*.h
        Renderer/*.cpp
        Renderer/*.h
        Misc/*.cpp
        Misc/*.h
)

add_subdirectory(week00-boilerplate)
add_subdirectory(week01)
add_subdirectory(week02)
