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

file(GLOB_RECURSE BOOK_SRC Math/*.cpp Math/*.h Physics/*.cpp Physics/*.h Renderer/*.cpp Renderer/*.h)

add_executable(GamePhysicsWeekend
        ${BOOK_SRC}
        application.cpp
        application.h
        Fileio.cpp
        Fileio.h
        Scene.cpp
        Scene.h
        main.cpp
)
target_link_libraries(GamePhysicsWeekend glfw vulkan)
add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/book/data ${CMAKE_CURRENT_BINARY_DIR}/data
)
