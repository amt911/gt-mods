add_library(telemetry_logic STATIC src/telemetry_logic.cpp)
target_include_directories(telemetry_logic PUBLIC src)
target_compile_options(telemetry_logic PRIVATE -Wall -Wextra)
set_target_properties(telemetry_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(telemetry_test tests/telemetry_test.cpp)
target_link_libraries(telemetry_test PRIVATE telemetry_logic)
add_test(NAME telemetry_test COMMAND telemetry_test)
target_compile_options(telemetry_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(telemetry MODULE src/mod.cpp)
target_link_libraries(telemetry PRIVATE telemetry_logic)
target_include_directories(telemetry PRIVATE "${GT_SDK_DIR}")
target_compile_options(telemetry PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(telemetry PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
