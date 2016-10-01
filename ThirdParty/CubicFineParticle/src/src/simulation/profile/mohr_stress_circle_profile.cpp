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
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/profile/mohr_stress_circle_profile.hpp"

void fj::MohrStressCircleProfile::startSimulationProfile()
{
    
}

void fj::MohrStressCircleProfile::endSimulationProfile()
{
    // 必ずインクリメントできるように先頭に配置する
    static unsigned int frameCount = -1;
    frameCount++;
    
    std::function<double()> func;
    
    const auto& world = getWorld();
    
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
        
        
        const std::string kFilename = "./log/particle" + std::to_string(i) + "_" + std::to_string(frameCount);
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
        output << "set terminal png" << std::endl;
        output << "set ytics 1.0" << std::endl;
        output << "set size ratio 1.0 1.0" << std::endl;
        output << "set output \"" << kFilename << ".png\"" << std::endl;
        output << "set grid" << std::endl; //gridを表示させる
        output << "unset border; set xtics axis; set ytics axis; set zeroaxis ls -1;" << std::endl;  //軸を交差させる
        output << "set object circle at "<< kCenter.X << "," << kCenter.Y << " size graph " <<  kRadius*0.165 << "fc rgb \"#ff0000\"" << std::endl; //モール応力円
        output << "plot [-3:5][-3:5]" << kWarrenSpringParameter.Adhesion << "*(" << "(x+" << kWarrenSpringParameter.Collapsibility << ")/" << kWarrenSpringParameter.Collapsibility << ")**(1.0/ " << kWarrenSpringParameter.SheerIndex << ")" << std::endl; // ワーレン・スプリング線を描画
        output << "set arrow 2 from -0,-3 to 0,10 nohead lc rgb \"#000000\"" << std::endl; // y軸描画
        output << "replot 0 lc rgb \"#000000\" notitle" << std::endl; //x軸描画
        
        // 原文
        //            reset
        //            set terminal png
        //            set ytics 1.0 # x方向に対するアスペクト比を設定する
        //            set size ratio 1.0 1.0
        //            set output "test.png"
        //            set grid   #gridを表示させる
        //            unset border; set xtics axis; set ytics axis; set zeroaxis ls -1;    #軸を交差させる
        //            set object circle at 0,0 size graph 0.165 fc rgb "#ff0000" #モール応力円
        //            plot [-1:5][-1:5]  x
        //            #plot [-3:5][-3:5]  x#<< kWarrenSpringParameter.Adhesion <<  *(  <<  (x+  << kWarrenSpringParameter.Collapsibility <<  )/  << kWarrenSpringParameter.Collapsibility <<  )**(1.0/   << kWarrenSpringParameter.SheerIndex <<  )   # ワーレン・スプリング線を描画
        //            set arrow 2 from -0,-3 to 0,10 nohead lc rgb  "#000000"    # y軸描画
        //            replot 0 lc rgb "#000000"  notitle   #x軸描画
        
    }
    
}

void fj::MohrStressCircleProfile::terminate()
{
    
}
