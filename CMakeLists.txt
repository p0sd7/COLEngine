cmake_minimum_required(VERSION 3.28)

project(MechanicsLab VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_TEST_SUITE OFF CACHE BOOL "" FORCE)

include(FetchContent)

FetchContent_Declare(
    SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.2
    GIT_SHALLOW ON
)

FetchContent_MakeAvailable(SFML)

add_library(physics_core STATIC
    src/physics/Math.h
    src/physics/Body.h
    src/physics/PhysicsSettings.h
    src/physics/Collision.h
    "src/physics/Collision.cpp"
    src/physics/PhysicsWorld.h
    src/physics/PhysicsWorld.cpp
 "src/app/AppCommand.h")

target_include_directories(physics_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

add_executable(mechanics_lab
    src/main.cpp
    src/configure/Config.h
    src/configure/Config.cpp

    src/app/Application.h
    src/app/Application.cpp

    src/platform/SfmlFrontend.h
    src/platform/SfmlFrontend.cpp

    src/scenes/SceneFactory.h
    src/scenes/SceneFactory.cpp
 "src/app/PhysicStats.h")

target_compile_definitions(mechanics_lab PRIVATE
    MECHANICS_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.ini"
)

target_link_libraries(mechanics_lab
    PRIVATE
        physics_core
        SFML::Graphics
)

if(MSVC)
    target_compile_options(physics_core PRIVATE /W4 /utf-8)
    target_compile_options(mechanics_lab PRIVATE /W4 /utf-8)
else()
    target_compile_options(physics_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(mechanics_lab PRIVATE -Wall -Wextra -Wpedantic)
endif()