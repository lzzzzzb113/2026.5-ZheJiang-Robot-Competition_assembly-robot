//
// Created by 32827 on 2026/4/1.
//

#include "robot_data.h"

void InitBase(gsl_matrix *base,double a1,double a2, double a3,double a4)
{
    static double len[4] = {0.0,250,175.0,0.0};//长度默认在x轴,0在z轴
    gsl_matrix_set_zero(base);
    gsl_matrix_set(base,0,3,len[3]);//x
    gsl_matrix_set(base,1,3,0);//y
    gsl_matrix_set(base,2,3,0);//z
    gsl_matrix_set(base,3,3,1);

    RotateY(base,a4);//末端开始

    gsl_matrix_set(base,0,3,gsl_matrix_get(base,0,3)+len[2]);
    RotateY(base,a3);
    gsl_matrix_set(base,0,3,gsl_matrix_get(base,0,3)+len[1]);
    RotateY(base,a2);
    gsl_matrix_set(base,2,3,gsl_matrix_get(base,2,3)+len[0]);
    RotateZ(base,a1);
}

void RotateX(gsl_matrix *mat, double angle)
{
    gsl_matrix *rotate = gsl_matrix_alloc(4,4);
    gsl_matrix *mat_copy = gsl_matrix_alloc(4,4);
    gsl_matrix_set(mat_copy,0,3,gsl_matrix_get(mat,0,3));
    gsl_matrix_set(mat_copy,1,3,gsl_matrix_get(mat,1,3));
    gsl_matrix_set(mat_copy,2,3,gsl_matrix_get(mat,2,3));
    gsl_matrix_set(mat_copy,3,3,1);
    gsl_matrix_set_zero(rotate);
    gsl_matrix_set(rotate,0,0,1);
    gsl_matrix_set(rotate,1,1,cos(angle));
    gsl_matrix_set(rotate,1,2,-sin(angle));
    gsl_matrix_set(rotate,2,1,sin(angle));
    gsl_matrix_set(rotate,2,2,cos(angle));
    gsl_matrix_set(rotate,3,3,1);

    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,rotate,mat_copy,0.0,mat);
    gsl_matrix_free(rotate);
    gsl_matrix_free(mat_copy);
}

void RotateY(gsl_matrix *mat, double angle)
{
    gsl_matrix *rotate = gsl_matrix_alloc(4,4);
    gsl_matrix *mat_copy = gsl_matrix_alloc(4,4);
    gsl_matrix_set(mat_copy,0,3,gsl_matrix_get(mat,0,3));
    gsl_matrix_set(mat_copy,1,3,gsl_matrix_get(mat,1,3));
    gsl_matrix_set(mat_copy,2,3,gsl_matrix_get(mat,2,3));
    gsl_matrix_set(mat_copy,3,3,1);
    gsl_matrix_set_zero(rotate);
    gsl_matrix_set(rotate,0,0,cos(angle));
    gsl_matrix_set(rotate,0,2,sin(angle));
    gsl_matrix_set(rotate,1,1,1);
    gsl_matrix_set(rotate,2,0,-sin(angle));
    gsl_matrix_set(rotate,2,2,cos(angle));
    gsl_matrix_set(rotate,3,3,1);

    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,rotate,mat_copy,0.0,mat);
    gsl_matrix_free(rotate);
    gsl_matrix_free(mat_copy);
}

void RotateZ(gsl_matrix *mat, double angle)
{
    gsl_matrix *rotate = gsl_matrix_alloc(4,4);
    gsl_matrix *mat_copy = gsl_matrix_alloc(4,4);
    gsl_matrix_set_zero(mat_copy);
    gsl_matrix_set(mat_copy,0,3,gsl_matrix_get(mat,0,3));
    gsl_matrix_set(mat_copy,1,3,gsl_matrix_get(mat,1,3));
    gsl_matrix_set(mat_copy,2,3,gsl_matrix_get(mat,2,3));
    gsl_matrix_set(mat_copy,3,3,1);
    gsl_matrix_set_zero(rotate);
    gsl_matrix_set(rotate,0,0,cos(angle));
    gsl_matrix_set(rotate,0,1,-sin(angle));
    gsl_matrix_set(rotate,1,0,sin(angle));
    gsl_matrix_set(rotate,1,1,cos(angle));
    gsl_matrix_set(rotate,2,2,1);
    gsl_matrix_set(rotate,3,3,1);

    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,rotate,mat_copy,0.0,mat);
    gsl_matrix_free(rotate);
    gsl_matrix_free(mat_copy);
}

void PrintPosition(gsl_matrix *mat)
{
    printf("x:%f,y:%f,z:%f\n",gsl_matrix_get(mat,0,3),gsl_matrix_get(mat,1,3),gsl_matrix_get(mat,2,3));
}

void MovePositionDataToMem(gsl_matrix *mat, Pos *position)
{
    position->x = gsl_matrix_get(mat,0,3);
    position->y = gsl_matrix_get(mat,1,3);
    position->z = gsl_matrix_get(mat,2,3);
}

void InitData(ArmDataLine *data,int count)
{
    double a4_t;
    double a1_t;
    gsl_matrix *base = gsl_matrix_alloc(4,4);
    for (int i = 0; i < count; i++)
    {
        a4_t = M_PI*(((double)(rand()%95000)+5000)/50000.0);
        a1_t = M_PI*2*(((double)(rand()%95000)+5000)/50000.0);
        InitBase(base,a1_t,0,0,a4_t);
        data[i].rotation.a1 = a1_t;
        data[i].rotation.a2 = 0;
        data[i].rotation.a3 = 0;
        data[i].rotation.a4 = a4_t;
        MovePositionDataToMem(base,&data[i].position);
    }
}
void RobotDataTest()
{
    gsl_matrix *base = gsl_matrix_alloc(4,4);
    InitBase(base,1,1,1,M_PI/2);
    PrintPosition(base);
}