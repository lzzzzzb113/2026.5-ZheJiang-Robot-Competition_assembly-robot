#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "genann.h"
#include "robot_data.h"
int count = 20000;
ArmDataLine *data_line;
ArmDataLine p;
int main(int argc, char *argv[])
{
    data_line = malloc(sizeof(ArmDataLine) * count);
    srand(time(0));
    gsl_matrix *base = gsl_matrix_alloc(4,4);
    //44 41
    InitBase(base,0,M_PI*3/2 + (55-44)/180.0*M_PI,M_PI/2+(63-41)/180.0*M_PI,0.0);
    PrintPosition(base);
    // int mod = 0; //0训练 1测试
    //
    // if (mod == 0)
    // {
    //     genann *ann = genann_init(3,3,48,4);
    //     double *input = malloc(sizeof(double) * 3);
    //     double *output = malloc(sizeof(double) * 4);
    //
    //     for (int j = 0; j < 320; j++)
    //     {
    //         if (j%32==0)
    //         {
    //             InitData(data_line,count);
    //         }
    //         for (int i = 0; i < count; i++) {
    //             input[0] = data_line[i].position.x;
    //             input[1] = data_line[i].position.y;
    //             input[2] = data_line[i].position.z;
    //             output[0] = data_line[i].rotation.a1;
    //             output[1] = data_line[i].rotation.a2;
    //             output[2] = data_line[i].rotation.a3;
    //             output[3] = data_line[i].rotation.a4;
    //
    //             // output[0] = data_line[i].rotation.a4;
    //             //归一化 input 最大 -50 - 50 -> -1 - 1
    //             input[0] = (input[0] + 50) / 100;
    //             input[1] = (input[1] + 50) / 100;
    //             input[2] = (input[2] + 50) / 100;
    //             //归一化 output 最大 -3.14 - 3.14 -> -1 - 1
    //             output[0] = (output[0]) / (M_PI*2);
    //             output[1] = (output[1]) / (M_PI);
    //             output[2] = (output[2]) / (M_PI);
    //             output[3] = (output[3]) / (M_PI);
    //             // output[0] = (output[0]) / (M_PI);
    //             genann_train(ann,input,output,10);
    //         }
    //         double *output_t = genann_run(ann,input);
    //         //反变换
    //         // output_t[0] = output_t[0] * (M_PI*2) - M_PI;
    //         // output_t[1] = output_t[1] * (M_PI*2) - M_PI;
    //         // output_t[2] = output_t[2] * (M_PI*2) - M_PI;
    //         // output_t[3] = output_t[3] * (M_PI*2) - M_PI;
    //         // double error_t = fabs(output_t[0] - output[0]) + fabs(output_t[1] - output[1]) + fabs(output_t[2] - output[2]) + fabs(output_t[3] - output[3]);
    //         // printf("[%d] %.6lf %.6lf error:%f\n",j,output_t[3],output[3],error_t);
    //
    //         double error_t = fabs(output_t[3] - output[3]);
    //         printf("[%d] %.6lf %.6lf error:%f\n",j,output_t[3],output[3],error_t);
    //     }
    //     RobotDataTest();
    //     FILE *out = fopen("E:/GitDown/RobotArmANN/ann_t2.dat","wb");
    //     genann_write(ann,out);
    //     fclose(out);
    //     genann_free(ann);
    // }else
    // {
    //     FILE *in = fopen("E:/GitDown/RobotArmANN/ann_one.dat","rb");
    //     genann *ann = genann_read(in);
    //     double true_out = M_PI*1/5;
    //     double *input = malloc(sizeof(double) * 3);
    //     gsl_matrix *base = gsl_matrix_alloc(4,4);
    //     InitBase(base,0,0,0,true_out);
    //     p.rotation.a1 = 0;
    //     p.rotation.a2 = 0;
    //     p.rotation.a3 = 0;
    //     p.rotation.a4 = true_out;
    //     MovePositionDataToMem(base,&p.position);
    //     input[0] = (p.position.x + 50) / 100;
    //     input[1] = (p.position.y + 50) / 100;
    //     double *output_t = genann_run(ann,input);
    //     printf("true_out:%f\n",(true_out)/(M_PI));
    //     printf("output_t:%f\n",output_t[0]);
    // }
    return 0;
}