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
#include "fine_particle/additional/povray/povray_output.hpp"

int main(int argc, char** argv)
{
    // デストラクタの時間を測定したいので, メモリ解放を管理しやすいスマートポインタを利用する
    std::unique_ptr<fj::FineParticleWorld> world(new fj::FineParticleWorld());
    world->setGravity( btVector3(0, -9.8, 0) );
    
        // レンダリング
    auto output = world->addProfileSystem<fj::POVRayOutput>();
    auto& location =  output->getCameraInformationPtr()->Location;
    location.X = -15;
    location.Y = 25;
    location.Z = 15;
    
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
	auto initializeStart = std::chrono::system_clock::now();
    for (int i = 0; i < 10; i++){
        for (int j = 0; j < 10; j++){
            for (int k = 0; k < 10; k++)
            {
                btVector3 position = btVector3(i, j, k);
                btMatrix3x3 matrix;

//                matrix.setEulerZYX(45, 45, 45);
//                position = matrix * position;
                position += btVector3(0, 0.6, 0);
                
                std::unique_ptr<fj::Particle> particle = fj::Particle::generateParticle( fj::DiscritizedParticleShape::ShapeType::kCube, position);
                particle->getWarrenSpringCurvePtr()->getParameterPtr()->Adhesion = 5.0;
                particle->getWarrenSpringCurvePtr()->getParameterPtr()->SheerIndex = 5;
                world->addParticle(std::move(particle));
            }
        }
    }
    
    // 粒子生成にかかった時間の出力
    // セカンドのオーダーではほとんどの場合で0sになってしまうので, ミリセカンドで時間を取得してセカンドに直す
	auto initializeTime = std::chrono::system_clock::now() - initializeStart;
	std::cout << "Initialize = "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(initializeTime).count() / 1000.0
		<< "sec."
		<< std::endl;


    // 時間測定用の変数を定義
	auto simulationStart = std::chrono::system_clock::now();
    auto simulationEnd = std::chrono::system_clock::now();
	auto simulationTime = simulationEnd - simulationStart;
	
    // 引数でシミュレーションステップが指定されていなければ1000回だけシミュレーションを回す
    // 引数で渡された値が数字以外だったときの処理は未定義
    const int kStep = (argc < 2) ? 1000 : std::atoi(argv[1]);
    const double kE = (argc < 3) ? 1 : std::atoi(argv[2]);

    //ぶつけてみる
    std::unique_ptr<btCollisionShape> groundShape_(new btSphereShape(1.5));
    btScalar mass_(10.5);
    btVector3 localInertia_(0,0,0);
    btTransform groundTransform_;
    groundTransform_.setIdentity();
    groundTransform_.setOrigin(btVector3(2, 12,8));
    std::unique_ptr<btDefaultMotionState> myMotionState_(new btDefaultMotionState(groundTransform_));
    btRigidBody::btRigidBodyConstructionInfo rbInfo_(mass_,myMotionState_.get(),groundShape_.get(),localInertia_);
    std::unique_ptr<btRigidBody> body_(new btRigidBody(rbInfo_));
    body_->setRollingFriction(1);
    body_->setFriction(1);
    world->addRigidBody( std::move(body_));

    
    // シミュレーションを進め, かかった時間を出力し, シミュレーション結果をpovray形式で吐き出す
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
    }

    // 粒子, ソルバの解放にかかる時間の測定
    auto destructStart = std::chrono::system_clock::now();
    world.reset();
    auto destructTime = std::chrono::system_clock::now() - destructStart;
    
    std::cout << "Destruct = "
    << std::chrono::duration_cast<std::chrono::milliseconds>(destructTime).count() / 1000.0
    << "sec."
    << std::endl;

    
    return 0;
}
