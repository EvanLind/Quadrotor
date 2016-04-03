#include "rcs.h"
#include "bsp.h"


// rx1,  ry1,  rxN,  ryN,分别是首端和末端的切向量
// px[],py[]是跑点坐标，u是张力参数，N_POINT是坐标个数,在.h中定义
// ppx[][],ppy[][]是处理后的坐标,ppx[0][3]表示第一段中的第4个点

void Hermite_Init(float rx1, float ry1, float rxN, float ryN,
                  float px[], float py[], float u, 
                  float ppx[][N_POINT], float ppy[][N_POINT])
{
    float t, t2, t3, express, px0, py0, px1, py1,h0,h1,h2,h3;
    float rxf,ryf,rxl,ryl;
    int n, count;

    for (n = 1; n <= N_POINT - 3; n++)
    {
        px0 = px[n];
        py0 = py[n];
        px1 = px[n + 1];
        py1 = py[n + 1];
        rxf = (1 - u) * (px[n + 1] - px[n - 1]);
        ryf = (1 - u) * (py[n + 1] - py[n - 1]);
        rxl = (1 - u) * (px[n + 2] - px[n]);
        ryl = (1 - u) * (py[n + 2] - py[n]);

        for (t = 0, count = 0; count < N_POINT; t += 1.0f/N_POINT, count ++)
        {
            t2 = t * t;
            t3 = t2 * t;
            express = 3 * t2 - 2 * t3;
            h0 = 1 - express;
            h1 = express;
            h2 = t - 2 * t2 + t3;
            h3 = t3 - t2;
            if (n == 1)
            {
                ppx[n - 1][count] = px[0] * h0 + px[1] * h1 + rx1 * h2 + rxf * h3;
                ppy[n - 1][count] = py[0] * h0 + py[1] * h1 + ry1 * h2 + ryf * h3;

            }
            else if (n == N_POINT - 3)
            {
                ppx[n - 1][count] = px[N_POINT - 2] * h0 + px[N_POINT - 1] * h1 + rxl * h2 + rxN * h3;
                ppy[n - 1][count] = py[N_POINT - 2] * h0 + py[N_POINT - 1] * h1 + ryl * h2 + ryN * h3;
            }
            ppx[n - 1][count] = px0 * h0 + px1 * h1 + rxf * h2 + rxl * h3;
            ppy[n - 1][count] = py0 * h0 + py1 * h1 + ryf * h2 + ryl * h3;
        }
    }
}