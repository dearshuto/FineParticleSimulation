// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ParticleStaticMeshActor.h"
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/particle/particle.hpp"

#include "GameFramework/Actor.h"
#include "FineParticleEmitter.generated.h"

UCLASS()
class FINEPARTICLE_API AFineParticleEmitter : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFineParticleEmitter();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

    UFUNCTION(BlueprintCallable, Category = "Actor")
    void CreateParticle(const FVector& Position);
    
	UFUNCTION(BlueprintCallable, Category = "Actor")
	void Terminate();

public:
    UFUNCTION(BlueprintCallable, Category = "Actor")
    void SetSimulationSpringK(const float SimulationSpringK);
    
	UFUNCTION(BlueprintCallable, Category = "Actor")
	void SetDashpodEnvelop(const float DashpodEnvelop);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Edit")
    float SimulationTimeStep;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Edit")
    int SimulationCycle;
    
private:
    void synchronizeRenderParticlePosition();
    
    TArray<AParticleStaticMeshActor*> m_particles;
    
    fj::FineParticleWorld m_world;
    
    std::unique_ptr<btCollisionShape> m_plane;
    std::unique_ptr<btMotionState> m_planeMotionState;
    std::shared_ptr<fj::Particle::CollapseDetector> m_collapseDetector;
};
