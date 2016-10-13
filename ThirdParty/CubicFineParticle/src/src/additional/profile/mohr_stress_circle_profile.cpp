//
//  mohr_stress_circle_profile.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#include <iostream>
#include <fstream>
#include <string>
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/additional/profile/mohr_stress_circle_profile.hpp"
#include "fine_particle/simulation/particle/particle.hpp"

void fj::MohrStressCircleProfile::startSimulationProfile()
{
    
}

void fj::MohrStressCircleProfile::endSimulationProfile()
{
    // 必ずインクリメントできるように先頭に配置する
    static unsigned int frameCount = -1;
    frameCount++;
    
    std::function<double()> func;
    
    const auto& world = getFineParticleWorld();
    
    for (int i = 0; i < world.getParticles().size(); i++)
    {
        // フィルターにかける
        if (m_filterFunction)
        {
            if ( !m_filterFunction(i) )
            {
                continue;
            }

        }
        
        
        const std::string kFilename = "./particle" + std::to_string(i) + "_" + std::to_string(frameCount);
        std::ofstream output(kFilename + ".gnuplot");
        
        if (output.fail())
        {
            std::cout << "MohrStressCircleProfile not run" << std::endl;
            break;
        }
        
        const auto& particle = world.getParticles()[i];
        const auto& kMohrStressCircle = particle->getFineParticleCollapseFactor().MohrStressCircle;
        const auto& kWarrenSpringParameter = particle->getWarrenSpringCurve().getParameter();
        
        const auto kCenter = kMohrStressCircle.getCenter();
        const auto kRadius = kMohrStressCircle.getRadius();
        
        output << "reset" << std::endl;
        output << "set parametric" << std::endl;
        output << "set terminal png" << std::endl;
        output << "set ytics 1.0" << std::endl;
        output << "set size ratio 1.0 1.0" << std::endl;
        output << "set output \"" << kFilename << ".png\"" << std::endl;
        output << "set grid" << std::endl; //gridを表示させる
        output << "unset border; set xtics axis; set ytics axis; set zeroaxis ls -1;" << std::endl;  //軸を交差させる
        output << "set xrange[-3:8]" << std::endl;
        output << "set yrange[-3:8]" << std::endl;
        output << "set trange[-3:8]" << std::endl;

        // ワーレン・スプリング線を描画. 媒介変数を使用するときはまとめて書かないといけないらしい
        output << "plot cos(t)*" << kRadius <<  "+" << kCenter.X << "," << kRadius << "*sin(t), t, " << kWarrenSpringParameter.Adhesion << "*(" << "(t+" << kWarrenSpringParameter.Collapsibility << ")/" << kWarrenSpringParameter.Collapsibility << ")**(1.0/ " << kWarrenSpringParameter.SheerIndex << ")" << std::endl;
    }
    
}

void fj::MohrStressCircleProfile::terminate()
{
    
}
