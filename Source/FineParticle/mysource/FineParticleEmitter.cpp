// Fill out your copyright notice in the Description page of Project Settings.

#include "FineParticle.h"
#include "FineParticleEmitter.h"


// Sets default values
AFineParticleEmitter::AFineParticleEmitter()
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
    m_plane.reset( new btStaticPlaneShape( btVector3(0, 0, 1), btScalar(0.)) );
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, 0, 30));
    m_planeMotionState.reset( new btDefaultMotionState(transform) );
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, m_planeMotionState.get(), m_plane.get(),localInertia);
    std::unique_ptr<btRigidBody> plane(new btRigidBody(rbInfo));
    
    m_world.addRigidBody(std::move(plane), int(fj::CollisionFiltering::kRigid), int(fj::CollisionFiltering::kRigid) | int(fj::CollisionFiltering::kParticle));
}

// Called every frame
void AFineParticleEmitter::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
    
    m_world.stepSimulation(DeltaTime);
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
    }
    
}

void AFineParticleEmitter::CreateParticle(const FVector& position)
{
    auto particle = fj::Particle::generateParticle(position.X, position.Y, position.Z);
    m_world.addParticle( std::move(particle), int(fj::CollisionFiltering::kParticle), int(fj::CollisionFiltering::kRigid) | int(fj::CollisionFiltering::kParticle)) ;
    
    auto visibleParticle = GetWorld()->SpawnActor<AParticleStaticMeshActor>();
    visibleParticle->SetActorLocation( position );
    visibleParticle->SetMobility(EComponentMobility::Movable);
    m_particles.Push(visibleParticle);
}