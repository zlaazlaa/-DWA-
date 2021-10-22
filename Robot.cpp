//
// Created by zlaa on 2021/10/22.
//

#include "Robot.h"


Robot::Robot(double maxVelocity, double maxAngularVelocity, double maxVelocityAcceleration,
             double maxAngularAcceleration) : Max_Velocity(maxVelocity), Max_Angular_Velocity(maxAngularVelocity),
                                              Max_Velocity_Acceleration(maxVelocityAcceleration),
                                              Max_Angular_Acceleration(maxAngularAcceleration) {}

Node::Node(double distToObstacle, double distToGoal, double velocity, double angularVelocity, double velocity1,
           double angularVelocity1, double distToAStar) : Dist_To_Obstacle(distToObstacle), Dist_To_Goal(distToGoal),
                                                          Velocity(velocity), Angular_Velocity(angularVelocity),
                                                          VELOCITY(velocity1), ANGULAR_VELOCITY(angularVelocity1),
                                                          Dist_To_A_Star(distToAStar) {}

Coordinate::Coordinate(int x, int y) : x(x), y(y) {}

Coordinate::Coordinate() {

}

Pair::Pair(double targetVelocity, double targetAngularVelocity) : Target_Velocity(targetVelocity),
                                                                  Target_Angular_Velocity(targetAngularVelocity) {}
