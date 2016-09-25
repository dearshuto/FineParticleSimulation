//
//  profiling_example.cpp
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
#include "fine_particle/simulation/profile/simulation_time_profile.hpp"
#include "fine_particle/povray/povray_output.hpp"

int main(int argc, char** argv)
{
    std::unique_ptr<fj::SimulationProfile> timeProfile(new fj::SimulationTimeProfile());
    std::shared_ptr<fj::FineParticleWorld> world(new fj::FineParticleWorld());
    world->setGravity( btVector3(0, -9.8, 0) );
    world->addProfileSystem( std::move(timeProfile) );
    
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
    
    // 粒子生成
    for (int i = 0; i < 10; i++){
        for (int j = 0; j < 10; j++){
            for (int k = 0; k < 10; k++)
            {
                btVector3 position = btVector3(i, 1.0 + float(j)*1.1, k);
                btMatrix3x3 matrix;

                matrix.setEulerZYX(45, 45, 45);
                position = matrix * position;
                position += btVector3(0, 1, 0);
                
                std::unique_ptr<fj::Particle> particle = fj::Particle::generateParticle( fj::DiscritizedParticleShape::ShapeType::kCube, position);
                world->addParticle(std::move(particle));
            }
        }
    }
    
    
    // シミュレーションを進め, かかった時間を出力し, シミュレーション結果をpovray形式で吐き出す
    for (int i = 0; i < 100; i++)
    {
        world->stepSimulation(1.0/480.0);
    }
    
    world->terminate();
    
    return 0;
}