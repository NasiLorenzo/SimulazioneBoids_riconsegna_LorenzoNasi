cmake_minimum_required(VERSION 3.16)
project(progettopf VERSION 0.1.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++17, senza estensioni non-standard offerte dal compilatore usato 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast")

# abilita asserzioni di debug (in gcc), l'address sanitizer e l'undefined-behaviour sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -D_GLIBCXX_ASSERTIONS  -fno-omit-frame-pointer")# -fsanitize=thread -g")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG "  -fno-omit-frame-pointer")# -fsanitize=thread")

include_directories("/opt/homebrew/Cellar/sfml/2.6.1/include")
include_directories("/opt/homebrew/Cellar/tbb/2021.13.0/include")
find_package(TBB REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
add_executable(boidsfml boids.cpp boidsfml.cpp doublevec.cpp sfmlboids.cpp)
add_executable(boidss boids.cpp mainprova.cpp doublevec.cpp)
add_executable(prove prove_varie.cpp doublevec.cpp)
target_link_libraries(boidsfml PRIVATE sfml-graphics sfml-window sfml-system)
target_link_libraries(boidsfml PRIVATE TBB::tbb TBB::tbbmalloc)
target_link_libraries(boidss PRIVATE TBB::tbb TBB::tbbmalloc)
target_link_libraries(prove PRIVATE TBB::tbb TBB::tbbmalloc)



if (BUILD_TESTING)

  # aggiungi l'eseguibile all.t
  add_executable(boids.t boids.cpp boids.test.cpp doublevec.cpp)
  target_link_libraries(boids.t PRIVATE TBB::tbb TBB::tbbmalloc)

  # aggiungi l'eseguibile all.t alla lista dei test
   add_test(NAME boids.t COMMAND boids.t)

endif()
