// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ParticleStaticMeshActor.h"
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

	
    TArray<AParticleStaticMeshActor*> m_particles;
};
