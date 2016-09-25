// Fill out your copyright notice in the Description page of Project Settings.

#include "FineParticle.h"
#include <fstream>
#include "fine_particle/simulation/particle/collapse_detector.hpp"
#include "fine_particle/simulation/profile/simulation_time_profile.hpp"
#include "FineParticleEmitter.h"


// Sets default values
AFineParticleEmitter::AFineParticleEmitter()
: SimulationTimeStep(1.0/60.)
, SimulationCycle(100)
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AFineParticleEmitter::BeginPlay()
{
	Super::BeginPlay();
    m_world.setGravity(btVector3(0, 0, -9.8));
    
    constexpr btScalar mass(0.);
    const btVector3 localInertia(0,0,0);
    m_plane.reset( new btBoxShape( btVector3(btScalar(1000), btScalar(10000), btScalar(10))) );
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, 0, -10));
    m_planeMotionState.reset( new btDefaultMotionState(transform) );
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, m_planeMotionState.get(), m_plane.get(),localInertia);
    std::unique_ptr<btRigidBody> plane(new btRigidBody(rbInfo));
    
    m_world.addRigidBody(std::move(plane));

    
    m_collapseDetector = std::make_shared<fj::Particle::CollapseDetector>();

	// Simulation Profiling
	FString dir = FPaths::GameDir();
	m_world.addProfileSystem( std::make_unique<fj::SimulationTimeProfile>(TCHAR_TO_ANSI(*dir)) );
}

// Called every frame
void AFineParticleEmitter::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
    
    for (int i = 0; i < SimulationCycle; i++)
    {
        m_world.stepSimulation(SimulationTimeStep);
        
    }

    synchronizeRenderParticlePosition();
}

void AFineParticleEmitter::synchronizeRenderParticlePosition()
{
    for (int i = 0; i < m_world.getParticles().size(); i++)
    {
        const auto& particle = m_world.getParticles()[i];
        btTransform trans;
        particle->getMotionState()->getWorldTransform(trans);
        const auto& kPosition = trans.getOrigin();
        
        m_particles[i]->SetActorLocation( FVector(kPosition.x(), kPosition.y(), kPosition.z()) );
        
//        UE_LOG(LogTemp, Warning, TEXT("%d Position : %f, %f, %f"), i, kPosition.x(), kPosition.y(), kPosition.z());
    }
    
}

void AFineParticleEmitter::CreateParticle(const FVector& position)
{
    const btVector3 kPosition{position.X, position.Y, position.Z};
    std::unique_ptr<fj::Particle> particle = std::move(fj::Particle::generateParticle(fj::DiscritizedParticleShape::ShapeType::kCube, kPosition) );

    
    auto visibleParticle = GetWorld()->SpawnActor<AParticleStaticMeshActor>();
    visibleParticle->SetActorLocation( position );
    visibleParticle->SetMobility(EComponentMobility::Movable);
    visibleParticle->setRadius( 10 * particle->getRadius() );
    
    m_particles.Push(visibleParticle);
    m_world.addParticle( std::move(particle) );
}

void AFineParticleEmitter::Terminate()
{
	m_world.terminate();
}

void AFineParticleEmitter::SetSimulationSpringK(const float SimulationSpringK)
{
    m_world.SpringK = SimulationSpringK;
}