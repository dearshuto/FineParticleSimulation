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
    static unsigned int frameCount = 0;
    
    const auto world = getWorld().lock();
    
    if (world)
    {
        for (int i = 0; i < world->getParticles().size(); i++)
        {
            const std::string kFilename = "./log/particle" + std::to_string(i) + "_" + std::to_string(frameCount);
            std::ofstream output(kFilename + ".gnuplot");
            
            if (output.fail())
            {
                std::cout << "MohrStressCircleProfile not run" << std::endl;
                break;
            }
            
            const auto& particle = world->getParticles()[i];
            const auto& kMohrStressCircle = particle->getFineParticleCollapseFactor().MohrStressCircle;
            const auto& kWarrenSpringParameter = particle->getWarrenSpringCurve().getParameter();
            
            const auto kCenter = kMohrStressCircle.getCenter();
            const auto kRadius = kMohrStressCircle.getRadius();
            
            output << "reset" << std::endl;
            output << "set terminal postscript eps enhanced color" << std::endl;
            output << "set output \"" << kFilename << ".eps\"" << std::endl;
            output << "unset border; set xtics axis; set ytics axis; set zeroaxis ls -1;" << std::endl;  //軸を交差させる
            output << "set grid" << std::endl; //gridを表示させる
            output << "set object circle at "<< kCenter.X << "," << kCenter.Y << " size scr " <<  kRadius << std::endl; //モール応力円
            output << "plot [-3:10][-3:10]" << kWarrenSpringParameter.Adhesion << "*(" << "(x+" << kWarrenSpringParameter.Collapsibility << ")/" << kWarrenSpringParameter.Collapsibility << ")**(1.0/ " << kWarrenSpringParameter.SheerIndex << ")" << std::endl; // ワーレン・スプリング線を描画
            output << "set arrow 2 from -0,-3 to 0,10 nohead lc rgb \"#000000\"" << std::endl; // y軸描画
            output << "replot 0 lc rgb \"#000000\" notitle" << std::endl; //x軸描画
            output << "replot" << std::endl;
        }
    }
    
    frameCount++;
}

void fj::MohrStressCircleProfile::terminate()
{
    
}
