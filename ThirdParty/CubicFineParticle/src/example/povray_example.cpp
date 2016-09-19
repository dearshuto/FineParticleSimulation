//
//  povray_example.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <iostream>
#include <chrono>
#include <cstdlib>
#include <string>
#include <memory>
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/povray/povray_output.hpp"

int main(int argc, char** argv)
{
    std::unique_ptr<btCollisionConfiguration> collisionConfiguration(new btDefaultCollisionConfiguration());
    std::unique_ptr<btCollisionDispatcher> dispatcher(new btCollisionDispatcher(collisionConfiguration.get()));
    std::unique_ptr<btBroadphaseInterface> broardphase(new btDbvtBroadphase());
    std::unique_ptr<btConstraintSolver> solver(new btSequentialImpulseConstraintSolver());
    std::shared_ptr<fj::FineParticleWorld> world(new fj::FineParticleWorld());
    
    world->setGravity( btVector3(0, -9.8, 0) );

    // 床
    std::unique_ptr<btCollisionShape> groundShape(new btBoxShape( btVector3(btScalar(1000), btScalar(10), btScalar(1000))));
    btScalar mass0(0.);
    btVector3 localInertia0(0,0,0);
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,-10,0));
    std::unique_ptr<btDefaultMotionState> myMotionState0(new btDefaultMotionState(groundTransform));
    btRigidBody::btRigidBodyConstructionInfo rbInfo0(mass0,myMotionState0.get(),groundShape.get(),localInertia0);
    std::unique_ptr<btRigidBody> body(new btRigidBody(rbInfo0));
    body->setRollingFriction(1);
    body->setFriction(1);
    world->addRigidBody( std::move(body));
    world->SpringK = 5;
    // レンダリング
    fj::POVrayOutput output( (std::weak_ptr<fj::FineParticleWorld>(world)) );
    
	auto initializeStart = std::chrono::system_clock::now();
    for (int i = 0; i < 5; i++){
        for (int k = 0; k < 5; k++){
            for (int j = 0; j < 5; j++)
            {
                btVector3 position = btVector3(i, 1.0 + float(j)*1.1, k);
//                btMatrix3x3 matrix;
//
//                matrix.setEulerZYX(45, 45, 45);
//                position = matrix * position;
//                position += btVector3(0, 1, 0);
                
                std::unique_ptr<fj::Particle> particle = std::move(
                                                                   fj::Particle::generateParticle( fj::DiscritizedParticleShape::ShapeType::kCube, position)
                                                                   );
                world->addParticle(std::move(particle));
            }
        }
    }
	auto initializeEnd = std::chrono::system_clock::now();
	auto initializeTime = initializeEnd - initializeStart;
 
	std::cout << "Initialize = "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(initializeTime).count() / 1000.0
		<< "sec."
		<< std::endl;


	auto simulationStart = std::chrono::system_clock::now();
    auto simulationEnd = std::chrono::system_clock::now();
	auto simulationTime = simulationEnd - simulationStart;
	const int kStep = (argc < 2) ? 1000 : std::atoi(argv[1]);
    for (int i = 0; i < kStep; i++)
    {
		simulationStart = std::chrono::system_clock::now();
        world->stepSimulation(1.0/480.0);
		simulationEnd = std::chrono::system_clock::now();
		simulationTime = simulationEnd - simulationStart;

		std::cout << "Step " << i+1 << "/" << kStep << " = "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(simulationTime).count() / 1000.0
			<< "sec."
			<< std::endl;

		output.saveToFile(std::to_string(i) + ".pov");
    }
    
    auto destructStart = std::chrono::system_clock::now();
    world.reset();
    auto destructTime = std::chrono::system_clock::now() - destructStart;
    
    std::cout << "Destruct = "
    << std::chrono::duration_cast<std::chrono::milliseconds>(destructTime).count() / 1000.0
    << "sec."
    << std::endl;

    
    return 0;
}
