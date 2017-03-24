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
    : GoalGeneratorBase(name),
      initialized(false),
      newGoal(false),
      counter(0)
{
}

GoalGenerator::GoalGenerator(std::string const& name, RTT::ExecutionEngine* engine)
    : GoalGeneratorBase(name, engine),
      initialized(false),
      newGoal(false),
      counter(0)
{
}

GoalGenerator::~GoalGenerator()
{
}


bool GoalGenerator::configureHook()
{
    if (! GoalGeneratorBase::configureHook())
        return false;

    currentGoal = _goal_index.value();
    heading = _goal_heading.value()/180.0*M_PI;
    joystickDeadbandSq = _joystick_deadband.value();
    joystickDeadbandSq *= joystickDeadbandSq;
    xpos = _xpos.get();
    ypos = _ypos.get();
    

    // Sizes of available waypoint lists are consistent
    if ( xpos.size() != ypos.size()){
        std::cerr << "\nError in GoalGenerator::configureHook()\n" 
                << "xpos size ("<< xpos.size() << ") does not match ypos ("<< ypos.size() << ") size!\n" 
                << "Check config/trajectory_helpers::GoalGenerator.yml.\n"
                << std::endl;
        return false;
    } else {
        numPoints = xpos.size();
    }

    // Selected goal index is valid:
    if ( currentGoal > xpos.size()-1 || currentGoal < 0){
        std::cerr << "\nError in GoalGenerator::configureHook()\n"
                << "Index: " << currentGoal << " is out of bounds: {0, ..., " << xpos.size() << "}.\n"
                << "Check goal_index in config/trajectory_helpers::GoalGenerator.yml.\n" 
                << std::endl;
        return false;
    }

    // Create initial goal position
    goalPose.position    = Eigen::Vector3d(xpos[currentGoal],ypos[currentGoal],0);
    goalPose.orientation = Eigen::Quaterniond(
                     Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    std::cout << "GoalGenerator::configureHook()"
                << std::endl << "Goal  = ("
                << goalPose.position.x() <<","
                << goalPose.position.y() <<"), "
                << "yaw = "<<  goalPose.getYaw()*180/M_PI << " deg."
                << std::endl;
    newGoal = true;
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
    if (!_raw_command.connected()){
        //if(counter == 50){
            newGoal = true;
        //}
        //counter = counter < 100? counter+1: 0;
        //std::cout << "GoalGenerator::updateHook: Joystick not connected. [" << counter << "]." << std::endl;
    } else {
        controldev::RawCommand joystick_command_prev;
        joystick_command_prev = joystick_command;
        
        // NEW JOYSTICK COMMAND HANDLING
        if(_raw_command.read(joystick_command) == RTT::NewData)
        {
            // Goal incrementation
            if(  joystick_command.buttons["BTN_START"]  &&    // Button is pressed down now
                !joystick_command_prev.buttons["BTN_START"])  // and was not pressed previously 
            {
                if (initialized){
                    // First button press - output the initial, no increment
                    // consecutivly increment the current goal on button press
                    currentGoal = currentGoal < numPoints-1 ? currentGoal+1 : 0;
                    newGoal = true;
                } else {
                    initialized = true;
                }
            }
            
            // Goal decrementation
            if(  joystick_command.buttons["BTN_BACK"]  &&    // Button is pressed down now
                !joystick_command_prev.buttons["BTN_BACK"])  // and was not pressed previously 
            {
                if (initialized){
                    currentGoal = currentGoal > 0 ? currentGoal-1 : (numPoints-1 );
                    newGoal = true;
                }
            }

            // Goal heading change
            double xaxis, yaxis, displacementNorm; 
            xaxis = -joystick_command.axes["ABS_Z"];
            yaxis = -joystick_command.axes["ABS_RZ"];
            displacementNorm = xaxis*xaxis + yaxis*yaxis;
            if( displacementNorm > joystickDeadbandSq ){
                heading = atan2(yaxis,xaxis);
                newGoal = true;
            }
        
            if(newGoal){
                // Update the goal
                goalPose.position    = Eigen::Vector3d(xpos[currentGoal],ypos[currentGoal],0);
                goalPose.orientation = Eigen::Quaterniond(
                         Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
            }
        }
    }

    // Output the goal - only if it has changed
    if (newGoal){
        std::cout << "GoalGenerator::updateHook() writing out:"
                    << std::endl << "Goal #"
                        << currentGoal<< "  = ("
                        << goalPose.position.x() <<","
                        << goalPose.position.y() <<"), "
                        << " yaw = "<<  goalPose.getYaw()*180/M_PI << " deg."
                    << std::endl;
        _goal_pose.write(goalPose);
        newGoal = false;
    }
    // Propagate the goal number to the output
    _current_goal.write(currentGoal);

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
