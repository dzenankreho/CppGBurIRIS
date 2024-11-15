#include "generalized_bur.hpp"


GBurIRIS::GBur::GeneralizedBur::GeneralizedBur(
    const Eigen::VectorXd& qCenter,
    const GeneralizedBurConfig& generalizedBurConfig,
    const robots::Robot& robot,
    const std::function<Eigen::VectorXd ()>& randomConfigGenerator
) : qCenter{ qCenter },
    generalizedBurConfig{ generalizedBurConfig },
    robot{ robot },
    randomConfigGenerator{ randomConfigGenerator } {


}


void GBurIRIS::GBur::GeneralizedBur::approximateObstaclesWithPlanes() {

}
