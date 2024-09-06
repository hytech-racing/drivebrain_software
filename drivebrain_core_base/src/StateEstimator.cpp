#include <StateEstimator.hpp>

using namespace core;

void StateEstimator::start_recv_thread()
{
    _recv_thread = std::thread([this]()
    {
        
    });
}

core::VehicleState StateEstimator::get_latest_state()
{
    return _vehicle_state;
}