//
//  collapse_visualization.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <iostream>
#include <cstdlib>
#include <string>

#include "fine_particle/simulation/particle/collapse_detector.hpp"
#include "fine_particle/simulation/particle/gnuplot_visualizing_collapse_detector.hpp"
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/particle/particle.hpp"

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

    const auto& kCollapseDetector = std::make_shared<fj::Particle::CollapseDetector>();
    std::shared_ptr<fj::Particle::CollapseDetector> kGNUPLOTDetector = std::make_shared<fj::GnuplotVisualizingCollapseDetector>();
    
    for (int i = 0; i < 1; i++){
        for (int k = 0; k < 1; k++){
            for (int j = 0; j < 2; j++)
            {
                std::unique_ptr<fj::Particle> particle = std::move( fj::Particle::generateParticle( fj::DiscritizedParticleShape::ShapeType::kCube, btVector3(i, 5 + j, k)) );
                
                particle->setCollapseDetector(
                                              std::weak_ptr<fj::Particle::CollapseDetector>(kCollapseDetector)
                                              );
                
                world->addParticle(std::move(particle), fj::CollisionGroup::kRigidParticle, fj::CollisionFiltering::kRigidParticle);
            }
        }
    }
    
    world->getParticles()[0]->setCollapseDetector(std::weak_ptr<fj::Particle::CollapseDetector>(kGNUPLOTDetector));
 
    const int kStep = (argc < 2) ? 1000 : std::atoi(argv[1]);
    for (int i = 0; i < kStep; i++)
    {
        world->stepSimulation(1.0/3600.0);
    }
    
    
    return 0;
}
