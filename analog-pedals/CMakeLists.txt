add_library(pedal_input STATIC src/pedal_input.cpp)
target_include_directories(pedal_input PUBLIC src)
target_compile_options(pedal_input PRIVATE -Wall -Wextra)

add_executable(pedal_input_test tests/pedal_input_test.cpp)
target_link_libraries(pedal_input_test PRIVATE pedal_input)
add_test(NAME pedal_input_test COMMAND pedal_input_test)
target_compile_options(pedal_input_test PRIVATE -UNDEBUG) # tests use plain assert
