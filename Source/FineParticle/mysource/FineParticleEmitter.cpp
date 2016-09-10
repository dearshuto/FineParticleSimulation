// Fill out your copyright notice in the Description page of Project Settings.

#include "FineParticle.h"
#include "FineParticleEmitter.h"


// Sets default values
AFineParticleEmitter::AFineParticleEmitter()
: SpringK(0.1)
, SimulationTimeStep(1.0/60.)
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
    
    m_world.addRigidBody(std::move(plane), fj::CollisionGroup::kRigid, fj::CollisionFiltering::kRigid);
    m_world.kSpringK = this->SpringK;
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
    std::unique_ptr<fj::Particle> particle = std::move(fj::Particle::generateParticle(position.X, position.Y, position.Z));
    m_world.addParticle( std::move(particle), fj::CollisionGroup::kRigidParticle, fj::CollisionFiltering::kRigidParticle );
    
    auto visibleParticle = GetWorld()->SpawnActor<AParticleStaticMeshActor>();
    visibleParticle->SetActorLocation( position );
    visibleParticle->SetMobility(EComponentMobility::Movable);
    m_particles.Push(visibleParticle);
}