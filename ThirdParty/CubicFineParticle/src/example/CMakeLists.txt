PROJECT(RayTracer)

add_executable(povray_example povray_example.cpp)
target_link_libraries( povray_example
	cubic_fine_particle
	BulletDynamics
	BulletCollision
	LinearMath
)
