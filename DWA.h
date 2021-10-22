//
// Created by zlaa on 2021/10/22.
//

#ifndef REDO_DWA_DWA_H
#define REDO_DWA_DWA_H

#include <iostream>
#include<vector>
#include<cmath>
#include<cstring>
#include<map>
#include<algorithm>
#include "DWA.h"
#include "Robot.h"

#define Max_Range 120
#define Delta 0.1
#define Predict_Delta 1.0
#define Velocity_Accuracy 0.2
#define Angular_Velocity_Accuracy 0.5
#define One_Block 0.05
#define Safe_Distance 2
#define Alpha 1 //Obstacle
#define Beta (-1.0) //Goal
#define Gamma 1.0 //Velocity
#define Delta2 (-5) //Dist_To_A_Star

double MIN(double a, double b);

double MAX(double a, double b);

double Calc_Dist(Coordinate a, Coordinate b);

bool Legal_Coordinate(Coordinate x);

bool Get_Trajectory(Coordinate Car_Coordinate, double Now_Velocity, double Now_Angle, double Now_Angular_Velocity);

double Get_Dist_To_Obstacle();

double Get_Dist_To_Goal(Coordinate Car_Destination);

Pair DWA(char Map[Max_Range][Max_Range], Coordinate Car_Coordinate, double Now_Angle, double Now_Velocity,
         double Now_Angular_Velocity, Coordinate Car_Destination, Robot Model);

#endif //REDO_DWA_DWA_H
