// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Engine/StaticMeshActor.h"
#include "ParticleStaticMeshActor.generated.h"

/**
 * 
 */
UCLASS()
class FINEPARTICLE_API AParticleStaticMeshActor : public AStaticMeshActor
{
	GENERATED_BODY()
	
    AParticleStaticMeshActor();
	
public:
    void setRadius(const float radius);
};
