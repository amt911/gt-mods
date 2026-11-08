add_library(cheats_logic STATIC src/cheats_logic.cpp)
target_include_directories(cheats_logic PUBLIC src)
target_compile_options(cheats_logic PRIVATE -Wall -Wextra)
set_target_properties(cheats_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(cheats_test tests/cheats_test.cpp)
target_link_libraries(cheats_test PRIVATE cheats_logic)
add_test(NAME cheats_test COMMAND cheats_test)
target_compile_options(cheats_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(cheats MODULE src/mod.cpp)
target_link_libraries(cheats PRIVATE cheats_logic)
target_include_directories(cheats PRIVATE "${GT_SDK_DIR}")
target_compile_options(cheats PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(cheats PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
