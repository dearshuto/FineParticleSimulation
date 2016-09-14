//
//  povray_example.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <iostream>
#include <cstdlib>
#include <string>
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
    world->addRigidBody( std::move(body), fj::CollisionGroup::kRigid, fj::CollisionFiltering::kRigid);
    world->SpringK = 100;
    // レンダリング
    fj::POVrayOutput output( (std::weak_ptr<fj::FineParticleWorld>(world)) );
    
    for (int i = 0; i < 10; i++){
        for (int k = 0; k < 10; k++){
            for (int j = 0; j < 10; j++)
            {
                btVector3 position = btVector3(i, 1.2 + float(j), k);
                btMatrix3x3 matrix;

                matrix.setEulerZYX(45, 45, 45);
                position = matrix * position;
                position += btVector3(0, 1, 0);
                
                std::unique_ptr<fj::Particle> particle = std::move(
                                                                   fj::Particle::generateParticle( fj::DiscritizedParticleShape::ShapeType::kCube, position)
                                                                   );
                world->addParticle(std::move(particle), fj::CollisionGroup::kRigidParticle, fj::CollisionFiltering::kRigidParticle);
            }
        }
    }
    
 
    const int kStep = (argc < 2) ? 1000 : std::atoi(argv[1]);
    for (int i = 0; i < kStep; i++)
    {
        world->stepSimulation(/*1.0/3600.0*/0.00001);
        output.saveToFile(std::to_string(i) + ".pov");
    }
    
    
    return 0;
}
