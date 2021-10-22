//
// Created by zlaa on 2021/10/22.
//

#ifndef REDO_DWA_ROBOT_H
#define REDO_DWA_ROBOT_H


class Robot {
public:
    double Max_Velocity;
    double Max_Angular_Velocity;
    double Max_Velocity_Acceleration;
    double Max_Angular_Acceleration;

    Robot(double maxVelocity, double maxAngularVelocity, double maxVelocityAcceleration, double maxAngularAcceleration);
};

class Node {
public:
    double Dist_To_Obstacle;
    double Dist_To_Goal;
    double Velocity;
    double Angular_Velocity;
    double VELOCITY;
    double ANGULAR_VELOCITY;
    double Dist_To_A_Star;


    Node(double distToObstacle, double distToGoal, double velocity, double angularVelocity, double velocity1,
         double angularVelocity1, double distToAStar);
};

class Coordinate {
public:
    int x{};
    int y{};

    Coordinate(int x, int y);
    Coordinate();
    bool operator<(const Coordinate b) const {
        if (this->x != b.x)
            return this->x < b.x;
        else
            return this->y < b.y;
    }
};

class Pair {
public:
    double Target_Velocity;
    double Target_Angular_Velocity;

    Pair(double targetVelocity, double targetAngularVelocity);

};

#endif //REDO_DWA_ROBOT_H
