//
//  discritized_particle_shape.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/10.
//
//

#include <array>
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include "fine_particle/simulation/particle/discritized_particle_shape.hpp"

std::unique_ptr<fj::DiscritizedParticleShape::NormalContainer> fj::DiscritizedParticleShape::GetDiscretizedParticleShapeNormal(const ShapeType type)
{
    //---------- 離散化形状を表すクラスの派生の定義 ------------------------------------------------------------------
    
    class CubeNormalContainer : public fj::DiscritizedParticleShape::NormalContainer
    {
    public:
        // コンストラクタ
        CubeNormalContainer()
        : m_normalArray({{
            {1, 0, 0},
            {-1, 0, 0},
            {0, 1, 0},
            {0, -1, 0},
            {0, 0, 1},
            {0, 0, -1}
        }})
        {}
        
        ~CubeNormalContainer() = default;
        
        unsigned int size()const override
        {
            return static_cast<unsigned int>(m_normalArray.size());
        }
        
        const btVector3& get(unsigned int index)const override
        {
            return m_normalArray[index];
        }
        
        void rotate(const btMatrix3x3& matrix)override
        {
            std::for_each(m_normalArray.begin(), m_normalArray.end()
                          , [matrix](btVector3& vector){
                              vector = matrix * vector;
                          });
        }
    private:
        std::array<btVector3, 6> m_normalArray;
    };
    
    
    
    
    //---------- インスタンス選択 ------------------------------------------------------------------------------------

    return std::unique_ptr<CubeNormalContainer>(new CubeNormalContainer());
}
