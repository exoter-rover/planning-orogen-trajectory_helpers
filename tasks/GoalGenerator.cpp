/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for goal pose generation from a 2D position 
 *  and heading information provided in a .yaml config file
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Oct 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include "GoalGenerator.hpp"

using namespace trajectory_helpers;

GoalGenerator::GoalGenerator(std::string const& name)
    : GoalGeneratorBase(name)
{
}

GoalGenerator::GoalGenerator(std::string const& name, RTT::ExecutionEngine* engine)
    : GoalGeneratorBase(name, engine)
{
}

GoalGenerator::~GoalGenerator()
{
}


bool GoalGenerator::configureHook()
{
    if (! GoalGeneratorBase::configureHook())
        return false;
    base::Vector2d position2d = _goal_position.value();     
    goalPose.position    = Eigen::Vector3d(position2d(0),position2d(1),0);
    goalPose.orientation = Eigen::Quaterniond(
                     Eigen::AngleAxisd(_goal_heading.value()/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    std::cout << "GoalGenerator::configureHook()"
                << std::endl << "Goal  = ("
                << goalPose.position.x() <<","
                << goalPose.position.y() <<","
                << goalPose.position.z() <<"), "
                << "yaw = "<<  goalPose.getYaw()*180/M_PI << " deg."
                << std::endl;
    return true;
}
bool GoalGenerator::startHook()
{
    if (! GoalGeneratorBase::startHook())
        return false;
    return true;
}
void GoalGenerator::updateHook()
{
    GoalGeneratorBase::updateHook();
    std::cout << "GoalGenerator::updateHook(): writing goal pose out." << std::endl;
    _goal_pose.write(goalPose);
}
void GoalGenerator::errorHook()
{
    GoalGeneratorBase::errorHook();
}
void GoalGenerator::stopHook()
{
    GoalGeneratorBase::stopHook();
}
void GoalGenerator::cleanupHook()
{
    GoalGeneratorBase::cleanupHook();
}
