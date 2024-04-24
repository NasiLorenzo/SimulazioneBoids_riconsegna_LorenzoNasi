
cmake_minimum_required(VERSION 3.16)
project(progettopf VERSION 0.1.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++17, senza estensioni non-standard offerte dal compilatore usato 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /Users/lorenzonasi/pf_labs/debug/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

#include(FetchContent)
#FetchContent_Declare(SFML
#    GIT_REPOSITORY https://github.com/SFML/SFML.git
#    GIT_TAG 2.6.x)
#FetchContent_MakeAvailable(SFML)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast")

# abilita asserzioni di debug (in gcc), l'address sanitizer e l'undefined-behaviour sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -D_GLIBCXX_ASSERTIONS -fsanitize=address,undefined -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")

include_directories("/opt/homebrew/Cellar/sfml/2.6.1/include")
include_directories("/opt/homebrew/Cellar/sfml/2.6.1/include/SFML")
include_directories("/opt/homebrew/Cellar/sfml/2.6.1/lib")
find_package(SFML COMPONENTS graphics window system)
add_executable(boidsfml boidcleanup.cpp)
target_link_libraries(boidsfml PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(boidsfml PRIVATE cxx_std_17)

#install(TARGETS progettopf)

#target_link_libraries(boidsfml "/opt/homebrew/Cellar/sfml/2.6.1/lib -lsfml-window -lsfml-graphics -lsfml-system")

# se il testing e' abilitato...
#   per disabilitare il testing, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
#if (BUILD_TESTING)

  # aggiungi l'eseguibile all.t
   #add_executable(functions.t functions.test.cpp functions.cpp)
  # aggiungi l'eseguibile all.t alla lista dei test
   #add_test(NAME functions.t COMMAND functions.t)

#endif()
