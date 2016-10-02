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
#include "fine_particle/additional/profile/simulation_time_profile.hpp"
#include "fine_particle/additional/profile/mohr_stress_circle_profile.hpp"
#include "fine_particle/additional/profile/mohr_stress_circle_distribution.hpp"
#include "fine_particle/additional/povray/povray_output.hpp"

int main(int argc, char** argv)
{
    std::shared_ptr<fj::FineParticleWorld> world(new fj::FineParticleWorld());
    std::weak_ptr<fj::FineParticleWorld> worldWeakPtr(world);
    world->setGravity( btVector3(0, -9.8, 0) );

    unsigned int simulationStep = 50;
    // 引数に何かしらが渡ってきたらプロファイル設定をおこなう
    if (1 < argc)
    {
        
        std::vector<std::string> commandOption;
        
        // オプションを全てバラす
        for (int i = 1; i < argc; i++)
        {
            commandOption.push_back( std::string(argv[i]) );
        }
        
        std::vector<std::string>::iterator iterator;
        
        // 出力ディレクトリの設定
        std::string outputDirectory(".");
        iterator = std::find_if(commandOption.begin(), commandOption.end()
                                , [](const std::string& option){
                                    return option == "-output";
                                });
        
        if (commandOption.end() != iterator)
        {
            outputDirectory = *(++iterator);
        }

        // シミュレーション回数
        iterator = std::find_if(commandOption.begin(), commandOption.end()
                                , [](const std::string& option){
                                    return option == "-step";
                                });
        
        if (commandOption.end() != iterator)
        {
            // -step の次に数字が来てるものとする
            try {
                simulationStep = std::stoi( *(++iterator) );
            } catch (const std::exception& e) {
                std::cout << "FUCK" << std::endl;
            }

        }
        
        // シミュレーション時間の最大値, 最小値, 平均を知りたいとき
        iterator = std::find_if(commandOption.begin(), commandOption.end()
                                , [](const std::string& option){
                                    return option == "-min_max_average_time";
                                });

        if (commandOption.end() != iterator)
        {
            auto timeProfile = world->addProfileSystem<fj::SimulationTimeProfile>(fj::AdditionalProcedure::Target::kSimulationTimeProfile);
            timeProfile->setOutputDirectory(outputDirectory);
            std::cout << "Min, Max, Average profile" << std::endl;
        }

        // 粒子にかかってる力の差分（モール応力円の半径）の分布が知りたいとき
        iterator = std::find_if(commandOption.begin(), commandOption.end()
                                , [](const std::string& option){
                                    return option == "-distribusion";
                                });
        if (commandOption.end() != iterator)
        {
            auto distribution = world->addProfileSystem<fj::MohrStressCircleDistribution>(fj::AdditionalProcedure::Target::kMohrStressCircleDistrubution);
            distribution->setGraph(0, 10, 0.25);
            distribution->setOutputDirectory(outputDirectory);
            std::cout << "Stress Distribution" << std::endl;
        }

        
        // 粉体崩壊曲線を見たいとき
        iterator = std::find_if(commandOption.begin(), commandOption.end()
                                , [](const std::string& option){
                                    return option == "-collapse_curve";
                                });
        if (commandOption.end() != iterator)
        {
            // --collapse_curveの次にフィルタ番号が指定されていないといけない
            try {
                const auto filter = std::stoi( *(++iterator) );
                
                auto mohrStressCircleProfile = world->addProfileSystem<fj::MohrStressCircleProfile>(fj::AdditionalProcedure::Target::kMohrStressCircleProfiler);
                mohrStressCircleProfile->setFilter( std::function<bool(const int)>([filter](const int index){return index == filter;} ) );
                mohrStressCircleProfile->setOutputDirectory(outputDirectory);
                std::cout << "Chase at " << filter << std::endl;
            } catch (const std::exception& e)
            {
                std::cout << "-collapse_curve number ← Use Integer" << std::endl;
            }
        }

    }
    
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
    for (int i = 0; i < simulationStep; i++)
    {
        world->stepSimulation(1.0/480.0);
    }
    
    world->terminate();
    
    return 0;
}
