// Fill out your copyright notice in the Description page of Project Settings.

#include "FineParticle.h"
#include "ParticleStaticMeshActor.h"


AParticleStaticMeshActor::AParticleStaticMeshActor()
{
    // Create and position a mesh component so we can see where our sphere is
    UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
    SphereVisual->SetupAttachment(RootComponent);
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_Sphere.Shape_Sphere"));
    if (SphereVisualAsset.Succeeded())
    {
        // 直径100の球体なので, 直径1で中心が原点に来るようにしておく
        SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
        SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, -0.5f));
        SphereVisual->SetWorldScale3D(FVector(0.1f));
    }
}