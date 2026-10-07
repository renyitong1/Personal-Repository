#include "kalman_filter.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

/***********************************************************
*@fuction	:kalman_filter_Init
*@brief		:
*@param		:--
*@return	:void
*@author	:--renyitong
*@date		:2026-10-06
***********************************************************/

static void kalman_assignment(kalman_filter_t *kal, float dt);
static int matrix_inverse(int n, float A[n][n], float Ainv[n][n]);

void kalman_filter_Init(kalman_filter_t *kal)
{
	if (kal == NULL) return;
	
	//给定Q R
	for(int i = 0; i < KAL_F_S; i ++)
	{
		kal->A[i][i] = 0.0f;
		kal->P_bar[i][i] = 1.0f;
		kal->Q[i][i] = 0.0f;
		kal->s_bar[i] = 0.0f;
	}
	for(int i = 0; i < KAL_F_M; i ++)
	{
		kal->R[i][i] = 0.0f;
	}
	for(int i = 0; i < KAL_F_S; i ++)
	{
		for(int j = 0; j < KAL_F_C; j ++)
		{
			kal->B[i][j] = 0.0f;
		}
	}
	for(int i = 0; i < KAL_F_M; i ++)
	{
		for(int j = 0; j < KAL_F_S; j ++)
		{
			kal->H[i][j] = 0.0f;
		}
	}
}


/***********************************************************
*@fuction	:kalman_filter_Calc
*@brief		: m : measure; s : state; c : control
*@param		:--
*@return	:void
*@author	:--renyitong
*@date		:2026-10-06
***********************************************************/
void kalman_filter_Calc(kalman_filter_t *kal, float m[KAL_F_M] , float c[KAL_F_C], float dt)
{
	if (kal == NULL) return;
	
	kalman_assignment(kal,dt);

	/***************  predict  ***************/
	//s_tilde = As_late_bar + Bc
	float s_late_bar[KAL_F_S] = {0.0f};
	float P_late_bar[KAL_F_S][KAL_F_S] = {0.0f};
	for(int i = 0; i < KAL_F_S; i++)
	{
		s_late_bar[i] = kal->s_bar[i];
		for(int j = 0; j < KAL_F_S; j ++)
		{
			P_late_bar[i][j] = kal->P_bar[i][j];
		}
	}

	//1. s_tilde
	for(int i = 0; i < KAL_F_S; i++)
	{
		float sum1 = 0.0f;
		for(int j = 0; j < KAL_F_S; j ++) {sum1 += kal->A[i][j] * s_late_bar[j];}
		for(int j = 0; j < KAL_F_C; j ++) {sum1 += kal->B[i][j] * c[j];}
		kal->s_tilde[i] = sum1;
	}
	
	//2. P_tilde
	float AP[KAL_F_S][KAL_F_S];
	float A_T[KAL_F_S][KAL_F_S];
	float APA_T[KAL_F_S][KAL_F_S];

	matrix_multiple(KAL_F_S, KAL_F_S, KAL_F_S, kal->A, P_late_bar, AP);
	matrix_transpose(KAL_F_S, KAL_F_S, kal->A, A_T);
	matrix_multiple(KAL_F_S, KAL_F_S, KAL_F_S, AP, A_T, APA_T);

	for(int i = 0; i < KAL_F_S; i ++)
	{
		for(int j = 0; j < KAL_F_S; j ++) {kal->P_tilde[i][j] = APA_T[i][j] + kal->Q[i][j];}
	} 
	
	/***************  update  ***************/	
	/* ---- 3. y = m - H * s_tilde ---- */
    float y[KAL_F_M];
    for (int i = 0; i < KAL_F_M; i++) {
        float sum = 0.0f;
        for (int j = 0; j < KAL_F_S; j++)
            sum += kal->H[i][j] * kal->s_tilde[j];
        y[i] = m[i] - sum;
    }

    /* ---- 4. S = H * P_tilde * H^T + R ---- */
    float HP [KAL_F_M][KAL_F_S];
    float HT [KAL_F_S][KAL_F_M];
    float HPH[KAL_F_M][KAL_F_M];
    float S  [KAL_F_M][KAL_F_M];

    matrix_multiple(KAL_F_M, KAL_F_S, KAL_F_S, kal->H, kal->P_tilde, HP);
    matrix_transpose(KAL_F_M, KAL_F_S, kal->H, HT);
    matrix_multiple(KAL_F_M, KAL_F_M, KAL_F_S, HP, HT, HPH);

    for (int i = 0; i < KAL_F_M; i++)
        for (int j = 0; j < KAL_F_M; j++)
            S[i][j] = HPH[i][j] + kal->R[i][j];

    /* ---- 5. 求 S 的逆（不是 H 的伪逆！） ---- */
    float Sinv[KAL_F_M][KAL_F_M];
    if (matrix_inverse(KAL_F_M, S, Sinv) != 0) {
        /* S 奇异，跳过更新，直接输出先验 */
        for (int i = 0; i < KAL_F_S; i++) kal->s_bar[i] = kal->s_tilde[i];
		for (int i = 0; i < KAL_F_S; i++)
        for (int j = 0; j < KAL_F_S; j++)
            kal->P_bar[i][j] = kal->P_tilde[i][j];
        return;
    }

    /* ---- 6. K = P_tilde * H^T * Sinv ---- */
    float PHT[KAL_F_S][KAL_F_M];
    matrix_multiple(KAL_F_S, KAL_F_M, KAL_F_S, kal->P_tilde, HT, PHT);
    matrix_multiple(KAL_F_S, KAL_F_M, KAL_F_M, PHT, Sinv, kal->K);

    /* ---- 7. s_hat = s_tilde + K * y ---- */
    for (int i = 0; i < KAL_F_S; i++) {
        kal->s_hat[i] = kal->s_tilde[i];
        for (int j = 0; j < KAL_F_M; j++)
            kal->s_hat[i] += kal->K[i][j] * y[j];
    }

    /* ---- 8. P_bar = (I - K*H) * P_tilde ---- */
    float KH  [KAL_F_S][KAL_F_S];
    float I_KH[KAL_F_S][KAL_F_S];
    float P_new[KAL_F_S][KAL_F_S];

    matrix_multiple(KAL_F_S, KAL_F_S, KAL_F_M, kal->K, kal->H, KH);

    for (int i = 0; i < KAL_F_S; i++)
        for (int j = 0; j < KAL_F_S; j++)
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    matrix_multiple(KAL_F_S, KAL_F_S, KAL_F_S, I_KH, kal->P_tilde, P_new);


    for (int i = 0; i < KAL_F_S; i++) {
        kal->s_bar[i] = kal->s_hat[i];       /* 当前后验状态，最终输出 */
    }
    for (int i = 0; i < KAL_F_S; i++) {
        for (int j = 0; j < KAL_F_S; j++) {
            kal->P_bar[i][j] = P_new[i][j];  /* 当前后验误差协方差 */
        }
    }
}

//A(m,k)B(k,n) = C(m,n); Cij = sum(Aik * bkj)
void matrix_multiple(int m ,int n, int k, float A[m][k], float B[k][n], float C[m][n])
{
	for(int i = 0; i < m; i ++)
	{
		for(int j = 0; j < n; j ++)
		{
			C[i][j] = 0;
			for(int t = 0; t < k; t++)
			{
				C[i][j] += A[i][t] * B[t][j];
			}
		}
	}
}

//AT[n][m] = A[m][n]
void matrix_transpose(int m, int n, float A[m][n], float AT[n][m])
{
	for(int i = 0; i < m; i ++)
	{
		for(int j = 0 ; j < n; j ++)
		{
			AT[j][i] = A[i][j];
		}
	}
}

/***********************************************************
* @function : matrix_inverse
* @brief    : A(n×n) -> Ainv(n×n)，高斯-约当，返回 0 成功，-1 奇异
* @param    : n - 维度；A - 输入；Ainv - 输出
* @return   : 0 成功，-1 奇异
* @author   : --renyitong
* @date     : 2026-10-06
***********************************************************/
static int matrix_inverse(int n, float A[n][n], float Ainv[n][n])
{
    double M[n][2 * n];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            M[i][j]     = A[i][j];
            M[i][n + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int col = 0; col < n; col++) {
        /* 选主元 */
        int piv = col;
        double mx = fabs(M[col][col]);
        for (int r = col + 1; r < n; r++) {
            if (fabs(M[r][col]) > mx) { mx = fabs(M[r][col]); piv = r; }
        }
        if (mx < 1e-12) return -1;

        /* 交换行 */
        if (piv != col) {
            for (int j = 0; j < 2 * n; j++) {
                double t = M[col][j];
                M[col][j] = M[piv][j];
                M[piv][j] = t;
            }
        }

        /* 归一化主元行 */
        double d = M[col][col];
        for (int j = 0; j < 2 * n; j++) M[col][j] /= d;

        /* 消元 */
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            double f = M[r][col];
            if (f == 0.0) continue;
            for (int j = 0; j < 2 * n; j++)
                M[r][j] -= f * M[col][j];
        }
    }

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Ainv[i][j] = (float)M[i][n + j];

    return 0;
}

static void kalman_assignment(kalman_filter_t *kal, float dt)
{
	//赋值,这里控制矩阵不好测，先置0
	kal->A[0][0] = 1.0f; kal->A[0][1] = dt;
	kal->A[1][0] = 0.0f; kal->A[1][1] = 1.0f;

	kal->H[0][0] = 1.0f; kal->H[0][1] = 0.0f;

	kal->R[0][0] = 10.0f;

	kal->Q[0][0] = 0.01f; kal->Q[0][1] = 0.0f;
	kal->Q[1][0] = 0.0f; kal->Q[1][1] = 0.1f;
}
