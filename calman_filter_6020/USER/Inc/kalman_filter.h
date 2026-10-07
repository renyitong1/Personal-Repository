#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

//控制维度
#define KAL_F_C 1
//状态
#define KAL_F_S 2
//观测
#define KAL_F_M 1

typedef struct
{
    //误差协方差矩阵
    float P_bar[KAL_F_S][KAL_F_S];
    float P_tilde[KAL_F_S][KAL_F_S];

    //过程模型噪声协方差矩阵
    float Q[KAL_F_S][KAL_F_S];
    //测量模型噪声协方差矩阵
    float R[KAL_F_M][KAL_F_M];
    
    //kalman增益
    float K[KAL_F_S][KAL_F_M];

    //转移矩阵
    float H[KAL_F_M][KAL_F_S];
    //状态转移矩阵
    float A[KAL_F_S][KAL_F_S];
    //控制矩阵
    float B[KAL_F_S][KAL_F_C];

    //state
    float s_bar[KAL_F_S];
    float s_tilde[KAL_F_S];
    float s_hat[KAL_F_S];
    
}kalman_filter_t;

void kalman_filter_Init(kalman_filter_t *kal);
void kalman_filter_Calc(kalman_filter_t *kal, float m[KAL_F_M] , float c[KAL_F_C], float dt);
void matrix_multiple(int m ,int n, int k, float A[m][k], float B[k][n], float C[m][n]);
void matrix_transpose(int m, int n,float A[m][n], float AT[n][m]);


#endif
