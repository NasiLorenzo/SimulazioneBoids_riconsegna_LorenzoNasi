cmake_minimum_required(VERSION 3.16)
project(progettopf VERSION 0.1.0)

# Enable testing support
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast")

# abilita asserzioni di debug (in gcc), l'address sanitizer e l'undefined-behaviour sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -D_GLIBCXX_ASSERTIONS  -fno-omit-frame-pointer -fsanitize=address,undefined")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG "  -fno-omit-frame-pointer -fsanitize=address,undefined")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -DNDEBUG)
endif()

#include_directories(YOUR_INCLUDE_PATH)
#include_directories(YOUR_INCLUDE_PATH)

find_package(TBB REQUIRED)
find_package(SFML 2.5 COMPONENTS window graphics REQUIRED)
find_package(OpenGL REQUIRED)

# Function to make the setting shorter
function(add_boids_executable target_name sources)
    add_executable(${target_name} ${sources})
    target_link_libraries(${target_name} PRIVATE TBB::tbb TBB::tbbmalloc)#comment this line if you are not on MacOS
endfunction()

# Define targets with the custom function
add_boids_executable(boidsfml "boids.cpp;boidsfml.cpp;doublevec.cpp;sfmlinterface.cpp;statistics.cpp")
target_link_libraries(boidsfml PRIVATE sfml-graphics sfml-window sfml-system)

add_boids_executable(boidss "boids.cpp;mainprova.cpp;doublevec.cpp")

add_boids_executable(simulation "boids.cpp;doublevec.cpp;base_run.cpp;statistics.cpp;simulation.cpp")
add_boids_executable(simulation3d "boids.cpp;doublevec.cpp;base_run_3d.cpp;statistics.cpp;simulation.cpp")
add_boids_executable(prova3d "prova3d.cpp;doublevec.cpp;boids.cpp")
target_compile_definitions(simulation3d PRIVATE -DDIM=3)
target_link_libraries(prova3d PRIVATE sfml-graphics sfml-window sfml-system OpenGL::GL)
target_compile_definitions(prova3d PRIVATE -DDIM=3)
if (BUILD_TESTING)
    # Testing executables
    add_boids_executable(boids.t "boids.cpp;boids.test.cpp;doublevec.cpp")
    add_boids_executable(boids3d.t "doublevec.cpp;boids.cpp;boids.test3d.cpp")
    target_compile_definitions(boids3d.t PRIVATE -DDIM=3)

    add_boids_executable(statistics.t "statistics.test.cpp;statistics.cpp;boids.cpp;doublevec.cpp")

    # Register the test with CTest
    add_test(NAME boids.t COMMAND boids.t)
endif()
