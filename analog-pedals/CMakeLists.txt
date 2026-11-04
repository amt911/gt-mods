add_library(pedal_input STATIC src/pedal_input.cpp)
target_include_directories(pedal_input PUBLIC src)
target_compile_options(pedal_input PRIVATE -Wall -Wextra)
set_target_properties(pedal_input PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(pedal_input_test tests/pedal_input_test.cpp)
target_link_libraries(pedal_input_test PRIVATE pedal_input)
add_test(NAME pedal_input_test COMMAND pedal_input_test)
target_compile_options(pedal_input_test PRIVATE -UNDEBUG) # tests use plain assert

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
add_library(analog_pedals MODULE src/mod.cpp src/input_source.cpp)
target_link_libraries(analog_pedals PRIVATE pedal_input PkgConfig::SDL2)
target_include_directories(analog_pedals PRIVATE "${GT_SDK_DIR}")
target_compile_options(analog_pedals PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(analog_pedals PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
