//
//  gnuplot_visualizing_collapse_detector.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#ifndef gnuplot_visualizing_collapse_detector_hpp
#define gnuplot_visualizing_collapse_detector_hpp

#include "collapse_detector.hpp"
#include "fine_particle/shape_2d/warren_spring_curve.hpp"

namespace fj{
    class GnuplotVisualizingCollapseDetector;
}

/**
 * 崩壊判定するたびにgnuplotでレンダリングできる形式で崩壊状態を出力する
 * @warning 出力ファイル名が同じだと上書きされるので, 常に最後に判定された粒子の情報しか出力できない
 */
class fj::GnuplotVisualizingCollapseDetector : public fj::Particle::CollapseDetector
{
public:
    GnuplotVisualizingCollapseDetector() = default;
    ~GnuplotVisualizingCollapseDetector() = default;
    
    bool shouldCallapse(const fj::Particle& particle)const override;
    
private:
    void saveToFile(const fj::MohrStressCircle& mohrStressCircle, const fj::WarrenSpringCurve& warrenSpringParameter)const;
};

#endif /* gnuplot_visualizing_collapse_detector_hpp */
