PROJECT(RayTracer)

add_executable(povray_example povray_example.cpp)
target_link_libraries( povray_example
	cubic_fine_particle
	BulletDynamics
	BulletCollision
	LinearMath
)

add_executable(profiling_example profiling_example.cpp)
target_link_libraries( profiling_example
	cubic_fine_particle
	BulletDynamics
	BulletCollision
	LinearMath
)
