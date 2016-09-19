PROJECT(CubicFineParticleSimulation)
cmake_minimum_required(VERSION 3.0)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

## Bullet Physics
include_directories(${CMAKE_SOURCE_DIR}/vendor/bullet/src)
add_subdirectory(vendor/bullet)

FILE(GLOB_RECURSE bullet_algorithm_files
	${CMAKE_SOURCE_DIR}/include/fine_particle/simulation/bullet_algorithm/*.hpp
	${CMAKE_SOURCE_DIR}/src/simulation/bullet_algorithm/*.cpp
)

FILE(GLOB_RECURSE simulation_files
	${CMAKE_SOURCE_DIR}/include/fine_particle/simulation/fine_particle_world.hpp
	${CMAKE_SOURCE_DIR}/src/simulation/fine_particle_world.cpp
    ${CMAKE_SOURCE_DIR}/include/fine_particle/simulation/mohr_stress_circle.hpp
    ${CMAKE_SOURCE_DIR}/src/simulation/mohr_stress_circle.cpp
)

FILE(GLOB_RECURSE particle_files
	${CMAKE_SOURCE_DIR}/include/fine_particle/simulation/particle/*.hpp
	${CMAKE_SOURCE_DIR}/src/simulation/particle/*.cpp
)

FILE(GLOB_RECURSE povray_files
	${CMAKE_SOURCE_DIR}/include/fine_particle/povray/povray_output.hpp
	${CMAKE_SOURCE_DIR}/src/povray/povray_output.cpp
)

SOURCE_GROUP(simulation\\particle FILES ${particle_files})
SOURCE_GROUP(simulation\\ FILES ${simulation_files})
SOURCE_GROUP(simulation\\bullet_algorithm FILES ${bullet_algorithm_files})
SOURCE_GROUP(povray FILES ${povray_files})

# インクルードフォルダを相対パスの検索対象に設定する
IF(WIN32)
add_definitions("/I${CMAKE_SOURCE_DIR}/include")
ELSE()
add_definitions("-I ${CMAKE_SOURCE_DIR}/include")
ENDIF()

add_library(cubic_fine_particle STATIC
	${simulation_files}
	${particle_files}
	${bullet_algorithm_files}
	${povray_files}
)


# ここからC++11を有効にする
add_definitions("-Wall -std=c++11")
add_subdirectory(example)
