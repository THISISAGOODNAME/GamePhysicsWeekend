set(APP_NAME week03)

file(GLOB_RECURSE PHYSICS_SRC
        Physics/*.cpp
        Physics/*.h
        )

add_executable(${APP_NAME}
        ${BOOK_SRC}
        ${PHYSICS_SRC}
        Scene.cpp
        Scene.h
        main.cpp
        )
target_link_libraries(${APP_NAME} glfw vulkan)
target_include_directories(${APP_NAME} PRIVATE .. ./ ../3rdparty/parallel-util/include)
add_custom_command(
        TARGET ${APP_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/3rdparty/book/data ${CMAKE_CURRENT_BINARY_DIR}/data
)