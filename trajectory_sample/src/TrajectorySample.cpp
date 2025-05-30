#include "TrajectorySample.hpp"
// #include <geometry/curvilinear_coordinate_system.h>
// #include "CoordinateSystemWrapper.hpp"

const TrajectorySample::LongitudinalTrajectory::OrderVectorX0 TrajectorySample::LongitudinalX0Order {0,1,2};
const TrajectorySample::LongitudinalTrajectory::OrderVectorXD TrajectorySample::LongitudinalXDOrder {1,2};

template class PolynomialTrajectory<4, 3, 2>;
template class PolynomialTrajectory<5, 3, 3>;

TrajectorySample::TrajectorySample(double dT,
                                   TrajectorySample::LongitudinalTrajectory trajectoryLongitudinal,
                                   TrajectorySample::LateralTrajectory trajectoryLateral,
                                   int uniqueId,
                                   Eigen::VectorXd samplingParameters)
    : m_dT (dT)
    , m_harm_occ_module (0)
    , m_uniqueId (uniqueId)
    , m_samplingParameters (samplingParameters)
    , m_trajectoryLongitudinal (std::make_shared<TrajectorySample::LongitudinalTrajectory>(trajectoryLongitudinal))
    , m_trajectoryLateral (std::make_shared<TrajectorySample::LateralTrajectory>(trajectoryLateral))
{

}

TrajectorySample::TrajectorySample(CartesianSample m_cartesianSample, CurviLinearSample m_curvilinearSample, size_t m_size, size_t m_actualSize, double m_dT)
: m_cartesianSample(m_cartesianSample), m_curvilinearSample(m_curvilinearSample), m_size(m_size), m_actualSize(m_actualSize), m_dT(m_dT) {}

void TrajectorySample::addCostValueToList(std::string costFunctionName, double cost, double costWeighted)
{
    m_cost += costWeighted;
    m_costMap[costFunctionName] = std::make_pair(cost, costWeighted);
}


void TrajectorySample::addFeasabilityValueToList(std::string feasabilityFunctionsName, double value)
{
    if(value) m_feasible = false;
    m_feasabilityMap[feasabilityFunctionsName] = value;
}

size_t TrajectorySample::size()
{
    return m_size;
}

