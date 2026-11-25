//
// Created by 32827 on 2026/4/1.
//

#ifndef EMBEDDEDMACHINELEARN_ROBOT_DATA_H
#define EMBEDDEDMACHINELEARN_ROBOT_DATA_H

#include "gsl/gsl_matrix.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_blas.h"
#include "math.h"

typedef struct
{
    double x, y, z;
} Pos;
typedef struct
{
    double a1,a2,a3,a4;
} Rot;

typedef struct
{
    Pos position;
    Rot rotation;
} ArmDataLine;

void InitBase(gsl_matrix *base,double a1,double a2, double a3,double a4);

void RotateX(gsl_matrix *mat, double angle);

void RotateY(gsl_matrix *mat, double angle);

void RotateZ(gsl_matrix *mat, double angle);

void PrintPosition(gsl_matrix *mat);

void MovePositionDataToMem(gsl_matrix *mat, Pos *position);

void InitData(ArmDataLine *data,int count);

void RobotDataTest();


#endif //EMBEDDEDMACHINELEARN_ROBOT_DATA_H