#include "mpc_planner_types/data_types.h"
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <ros_tools/logging.h>

int main(int argc, char *argv[])
{
    MPCPlanner::Trajectory traj_1(0.2, 5);
    MPCPlanner::Trajectory traj_2(0.2, 5);
    MPCPlanner::Trajectory traj_3(0.2, 5);

    std::vector<Eigen::Vector2d> v1{
        {0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}, {3.0, 4.0}};
    std::vector<Eigen::Vector2d> v2{
        {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}, {4.0, 4.0}, {5.0, 4.0}};
    std::vector<Eigen::Vector2d> v3{
        {1.5, 1.5}, {2.5, 2.5}, {3.5, 3.5}, {4.5, 4.5}, {5.5, 5.5}};

    for (size_t i = 0; i < v1.size(); i++)
    {
        traj_1.add(v1[i]);
        traj_2.add(v2[i]);
        traj_3.add(v3[i]);
    }

    const double result_gaus_12 = traj_1.calcCollisionMaskGK(traj_2, 0.8);
    const double resutl_gaus_13 = traj_1.calcCollisionMaskGK(traj_3, 0.8);

    LOG_INFO(result_gaus_12);
    LOG_INFO(resutl_gaus_13);
    return 0;
}
