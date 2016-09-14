//
//  gnuplot_visualizing_collapse_detector.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#include <fstream>
#include <string>
#include "fine_particle/simulation/particle/gnuplot_visualizing_collapse_detector.hpp"

bool fj::GnuplotVisualizingCollapseDetector::shouldCallapse(const fj::Particle &particle)const
{
    const auto kMohrStressCircle = generateMohrStressCircle(particle);
    
    saveToFile(kMohrStressCircle, particle.getWarrenSpringParameter());
    
    return kMohrStressCircle.hasIntersectionPoint(particle.getWarrenSpringParameter());
}

void fj::GnuplotVisualizingCollapseDetector::saveToFile(const fj::MohrStressCircle &mohrStressCircle, const fj::WarrenSpringParameter& warrenSpringParameter)const
{
    static unsigned int tickCount = 0;
    
    std::ofstream output(std::to_string(tickCount) + ".plot");
    
//    output << "set terminal aqua" << std::endl;
    output << "set terminal postscript eps enhanced color" << std::endl;
    output << "set object circle at "
        << std::to_string(mohrStressCircle.getCenter()[0]) << ", " << std::to_string(mohrStressCircle.getCenter()[1])
        << " size " << std::to_string(mohrStressCircle.getRadius())
        << std::endl;
    output << "plot " << std::to_string(warrenSpringParameter.Adhesion) << "*" << "(" << "x / " << std::to_string(warrenSpringParameter.Collapsibility) << " + 1)" << "**(" << std::to_string(warrenSpringParameter.SheerIndex) << ")" << std::endl;
    
    
    output << "set output" << "\"" << std::to_string(tickCount) << ".eps\"" << std::endl;
    output << "replot" << std::endl;
    
    tickCount++;
}