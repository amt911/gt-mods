add_library(damage_logic STATIC src/damage_logic.cpp)
target_include_directories(damage_logic PUBLIC src)
target_compile_options(damage_logic PRIVATE -Wall -Wextra)
set_target_properties(damage_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(damage_test tests/damage_test.cpp)
target_link_libraries(damage_test PRIVATE damage_logic)
add_test(NAME damage_test COMMAND damage_test)
target_compile_options(damage_test PRIVATE -UNDEBUG) # tests use plain assert

# The hit filter (race start, retry, reset are not hits) is the deformation mod's: one definition, one behaviour.
add_library(mechanical_damage MODULE src/mod.cpp)
target_link_libraries(mechanical_damage PRIVATE damage_logic deformation_logic)
target_include_directories(mechanical_damage PRIVATE "${GT_SDK_DIR}")
target_compile_options(mechanical_damage PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(mechanical_damage PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
