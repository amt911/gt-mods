add_library(orbit_logic STATIC src/orbit_math.cpp src/orbit_input.cpp)
target_include_directories(orbit_logic PUBLIC src)
target_compile_options(orbit_logic PRIVATE -Wall -Wextra)
set_target_properties(orbit_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(orbit_test tests/orbit_test.cpp)
target_link_libraries(orbit_test PRIVATE orbit_logic)
add_test(NAME orbit_test COMMAND orbit_test)
target_compile_options(orbit_test PRIVATE -UNDEBUG) # tests use plain assert

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
add_library(orbit_camera MODULE src/mod.cpp src/input_source.cpp)
target_link_libraries(orbit_camera PRIVATE orbit_logic PkgConfig::SDL2)
target_include_directories(orbit_camera PRIVATE "${GT_SDK_DIR}")
target_compile_options(orbit_camera PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(orbit_camera PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
