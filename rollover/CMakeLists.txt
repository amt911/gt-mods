add_library(rollover_logic STATIC src/rollover_logic.cpp)
target_include_directories(rollover_logic PUBLIC src)
target_compile_options(rollover_logic PRIVATE -Wall -Wextra)
set_target_properties(rollover_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(rollover_test tests/rollover_test.cpp)
target_link_libraries(rollover_test PRIVATE rollover_logic)
add_test(NAME rollover_test COMMAND rollover_test)
target_compile_options(rollover_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(rollover MODULE src/mod.cpp)
target_link_libraries(rollover PRIVATE rollover_logic)
target_include_directories(rollover PRIVATE "${GT_SDK_DIR}")
target_compile_options(rollover PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(rollover PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)

add_executable(bodybox_test tests/bodybox_test.cpp)
target_include_directories(bodybox_test PRIVATE src)
add_test(NAME bodybox_test COMMAND bodybox_test)
target_compile_options(bodybox_test PRIVATE -UNDEBUG)
