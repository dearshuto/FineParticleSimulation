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
	
}

// Called every frame
void AFineParticleEmitter::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

