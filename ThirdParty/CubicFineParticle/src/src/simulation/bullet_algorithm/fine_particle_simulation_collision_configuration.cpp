//
//  fine_particle_simulation_collision_configuration.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/15.
//
//

#include "fine_particle/simulation/bullet_algorithm/fine_particle_simulation_collision_configuration.hpp"

btCollisionAlgorithmCreateFunc* fj::FineParticleSimulationCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1)
{
    // 粉体粒子どうしの衝突には独自のアルゴリズムを適用する
    // それ以外はBullet Physicsが提供する衝突アルゴリズムを利用するので, Proxyフラグを調節する
    
    // 両方粉体粒子の場合
    if ( (proxyType0 == CUSTOM_POLYHEDRAL_SHAPE_TYPE) || (proxyType1 == CUSTOM_POLYHEDRAL_SHAPE_TYPE) )
    {
        return m_finefine;
    }
    
    // proxy0が粉体粒子, proxyType1がオブジェクトの場合
    if ( (proxyType0 == CUSTOM_POLYHEDRAL_SHAPE_TYPE) || (proxyType1 != CUSTOM_POLYHEDRAL_SHAPE_TYPE) )
    {
        const int kProxyType0 = SPHERE_SHAPE_PROXYTYPE;
        return Super::getCollisionAlgorithmCreateFunc(kProxyType0, proxyType1);
    }

    // proxy0がオブジェクト, proxyType1粉体粒子の場合
    if ( (proxyType0 != CUSTOM_POLYHEDRAL_SHAPE_TYPE) || (proxyType1 == CUSTOM_POLYHEDRAL_SHAPE_TYPE) )
    {
        const int kProxyType1 = SPHERE_SHAPE_PROXYTYPE;
        return Super::getCollisionAlgorithmCreateFunc(proxyType0, kProxyType1);
    }

    return Super::getCollisionAlgorithmCreateFunc(proxyType0, proxyType1);
}