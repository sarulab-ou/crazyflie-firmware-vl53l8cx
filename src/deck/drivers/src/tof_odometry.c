/**
 * 超低密度ToF点群からの移動量・回転量の幾何学的抽出 (usd/tof_odometry.py のオンボード版)。
 *
 * 数値計算は 3x3 の対称行列に閉じるように整理してあるので、SVD ルーチンは不要:
 *   - PCA           : 共分散 C (対称3x3) の Jacobi 固有値分解
 *   - Kabsch        : H^T H (対称3x3) の固有値分解から V を得て U = H V S^-1
 *   - Point-to-Plane: 正規方程式 (Σ w n n^T) Δt = -Σ w Δd n を固有値分解で
 *                     truncated SVD 相当に解く (退化方向は 0 とする)
 */
#include "tof_odometry.h"

#include <math.h>
#include <string.h>

#include "log.h"
#include "param.h"
#include "usec_time.h"
#include "vl53l8cx_api.h" /* vl53l8cxToFDist[][] */

/* ---- VL53L8CX の仕様 (tof_odometry.py と同じ) ---- */
#define FOV_DEG    45.0f  /* 正方FOV (対角65度 ≒ 片辺45度) */
#define GRID       4      /* 4x4 = 16 ゾーン */
#define NZONE      (GRID * GRID)
#define D_MIN_MM   30.0f
#define D_MAX_MM   4000.0f

/* ---- 平面抽出パラメータ ---- */
#define RANSAC_ITERS     60
#define RANSAC_THRESH_MM 20.0f
#define MIN_INLIERS      6
#define PLANARITY_EPS    0.02f

/* ---- 移動量推定の打ち切り閾値 (python の rcond) ---- */
#define TRANS_RCOND 0.05f

#define DEG2RAD (float)(M_PI / 180.0)
#define RAD2DEG (float)(180.0 / M_PI)
#define NANF    ((float)NAN)

/* オドメトリに使うセンサー番号。搭載している 11 個すべてを使う。 */
static const uint8_t TOFODO_SENSORS[TOFODO_NUM_SENSORS] = {0, 1, 2, 3, 4, 5,
                                                           6, 7, 8, 9, 10};

/* センサー取り付け (docs/vl53l8cx-position.md, 位置[mm] / yaw-pitch-roll[deg])。 */
static const float SENSOR_MOUNT_T[TOFODO_NUM_SENSORS][3] = {
    {56.6f, -9.5f, 0.0f},   /* 0 */
    {59.2f, 0.0f, 0.0f},    /* 1 */
    {56.6f, 9.5f, 0.0f},    /* 2 */
    {56.6f, 0.0f, 9.5f},    /* 3 */
    {56.6f, 0.0f, -9.5f},   /* 4 */
    {0.0f, -21.7f, 0.0f},   /* 5 */
    {0.0f, 21.7f, 0.0f},    /* 6 */
    {-59.2f, 0.0f, 0.0f},   /* 7 */
    {-2.0f, 0.0f, 4.2f},    /* 8 */
    {28.5f, 0.0f, -4.2f},   /* 9 */
    {-37.5f, 0.0f, -4.2f},  /* 10 */
};
static const float SENSOR_MOUNT_YPR[TOFODO_NUM_SENSORS][3] = {
    {-22.5f, 0.0f, 0.0f},   /* 0 */
    {0.0f, 0.0f, 0.0f},     /* 1 */
    {22.5f, 0.0f, 0.0f},    /* 2 */
    {0.0f, 22.5f, 0.0f},    /* 3 */
    {0.0f, -22.5f, 0.0f},   /* 4 */
    {-90.0f, 0.0f, 0.0f},   /* 5 */
    {90.0f, 0.0f, 0.0f},    /* 6 */
    {180.0f, 0.0f, 0.0f},   /* 7 */
    {0.0f, -90.0f, 0.0f},   /* 8 */
    {0.0f, 90.0f, 0.0f},    /* 9 */
    {0.0f, 90.0f, 0.0f},    /* 10 */
};

typedef struct
{
    float n[3];      /* 単位法線 (ボディ座標系, 原点→平面向き) */
    float d;         /* 原点から平面までの垂直距離 [mm] */
    float planarity; /* λ3 / Σλ (小さいほど良い平面) */
    bool valid;
} Plane;

/* ボディ座標系での各ゾーンの光線方向 (取り付け回転を適用済み)。 */
static float s_ray[TOFODO_NUM_SENSORS][NZONE][3];
static bool s_initialized = false;

/* 累積姿勢・位置と前フレームの平面。 */
static float s_Rcum[3][3];
static float s_pcum[3];
static Plane s_prevPlane[TOFODO_NUM_SENSORS];
static bool s_havePrev = false;

/* ---- log 変数 (microSD card deck で記録する最小セット) ---- */
static float s_logPlaneD[TOFODO_NUM_SENSORS];   /* 平面距離 d_i [mm] */
static float s_logPlanarity[TOFODO_NUM_SENSORS];/* 平面性スコア */
static float s_logAzimuth[TOFODO_NUM_SENSORS];  /* 法線の方位角 [deg] */
static float s_logDypr[3];                      /* ΔR (yaw,pitch,roll) [deg/frame] */
static float s_logDt[3];                        /* Δt (x,y,z) [mm/frame] */
static float s_logYpr[3];                       /* 累積回転 [deg] */
static float s_logPos[3];                       /* 累積移動 [mm] */
static uint8_t s_logRankR = 0;
static uint8_t s_logRankT = 0;
static uint8_t s_logNPairs = 0;
static uint32_t s_logSeq = 0;   /* 処理したフレーム数 */
static uint32_t s_logCalcUs = 0;/* 1フレームの計算時間 [us] */

/* 推定を止めたいときのパラメータ (既定=有効)。 */
static uint8_t s_enable = 1;

/* ============================================================
 * 小さな線形代数ユーティリティ
 * ============================================================ */
static void mat3Identity(float R[3][3])
{
    memset(R, 0, 9 * sizeof(float));
    R[0][0] = R[1][1] = R[2][2] = 1.0f;
}

/* out = A * B */
static void mat3Mul(const float A[3][3], const float B[3][3], float out[3][3])
{
    float tmp[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            tmp[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

/* out = A * v */
static void mat3MulVec(const float A[3][3], const float v[3], float out[3])
{
    float tmp[3];
    for (int i = 0; i < 3; i++)
    {
        tmp[i] = A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2];
    }
    memcpy(out, tmp, sizeof(tmp));
}

static void vec3Cross(const float a[3], const float b[3], float out[3])
{
    float tmp[3] = {a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]};
    memcpy(out, tmp, sizeof(tmp));
}

static float vec3Dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static bool vec3Normalize(float v[3])
{
    float n = sqrtf(vec3Dot(v, v));
    if (n < 1e-9f)
    {
        return false;
    }
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return true;
}

/* Z-Y-X (yaw→pitch→roll) 順の回転行列。 */
static void rotYpr(float yawDeg, float pitchDeg, float rollDeg, float R[3][3])
{
    float y = yawDeg * DEG2RAD, p = pitchDeg * DEG2RAD, r = rollDeg * DEG2RAD;
    float cy = cosf(y), sy = sinf(y);
    float cp = cosf(p), sp = sinf(p);
    float cr = cosf(r), sr = sinf(r);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

/* 回転行列 → (yaw, pitch, roll) [deg] (Z-Y-X)。 */
static void rotToYpr(const float R[3][3], float ypr[3])
{
    float s = -R[2][0];
    s = (s > 1.0f) ? 1.0f : ((s < -1.0f) ? -1.0f : s);
    float pitch = asinf(s);
    float yaw, roll;
    if (cosf(pitch) > 1e-6f)
    {
        yaw = atan2f(R[1][0], R[0][0]);
        roll = atan2f(R[2][1], R[2][2]);
    }
    else /* ジンバルロック */
    {
        yaw = 0.0f;
        roll = atan2f(-R[1][2], R[1][1]);
    }
    ypr[0] = yaw * RAD2DEG;
    ypr[1] = pitch * RAD2DEG;
    ypr[2] = roll * RAD2DEG;
}

/* Gram-Schmidt で回転行列の直交性を回復する (累積積の数値ドリフト対策)。 */
static void mat3Reorthonormalize(float R[3][3])
{
    float c0[3] = {R[0][0], R[1][0], R[2][0]};
    float c1[3] = {R[0][1], R[1][1], R[2][1]};
    float c2[3];

    if (!vec3Normalize(c0))
    {
        mat3Identity(R);
        return;
    }
    float proj = vec3Dot(c0, c1);
    for (int i = 0; i < 3; i++)
    {
        c1[i] -= proj * c0[i];
    }
    if (!vec3Normalize(c1))
    {
        mat3Identity(R);
        return;
    }
    vec3Cross(c0, c1, c2);

    for (int i = 0; i < 3; i++)
    {
        R[i][0] = c0[i];
        R[i][1] = c1[i];
        R[i][2] = c2[i];
    }
}

/**
 * 対称3x3行列の固有値分解 (循環 Jacobi 法)。
 * w[] は昇順、V の列 k が w[k] に対応する固有ベクトル (A = V diag(w) V^T)。
 */
static void eigenSym3(const float Ain[3][3], float w[3], float V[3][3])
{
    float A[3][3];
    memcpy(A, Ain, sizeof(A));
    mat3Identity(V);

    static const int PQ[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    for (int sweep = 0; sweep < 12; sweep++)
    {
        float off = fabsf(A[0][1]) + fabsf(A[0][2]) + fabsf(A[1][2]);
        if (off < 1e-14f * (fabsf(A[0][0]) + fabsf(A[1][1]) + fabsf(A[2][2]) + 1e-30f))
        {
            break;
        }
        for (int k = 0; k < 3; k++)
        {
            int p = PQ[k][0], q = PQ[k][1];
            if (fabsf(A[p][q]) < 1e-20f)
            {
                continue;
            }
            /* A[p][q] を消す Givens 回転 J: J[p][p]=J[q][q]=c, J[p][q]=s, J[q][p]=-s */
            float theta = (A[q][q] - A[p][p]) / (2.0f * A[p][q]);
            float t = ((theta >= 0.0f) ? 1.0f : -1.0f) /
                      (fabsf(theta) + sqrtf(theta * theta + 1.0f));
            float c = 1.0f / sqrtf(t * t + 1.0f);
            float s = t * c;

            for (int i = 0; i < 3; i++) /* A <- A J */
            {
                float aip = A[i][p], aiq = A[i][q];
                A[i][p] = c * aip - s * aiq;
                A[i][q] = s * aip + c * aiq;
            }
            for (int i = 0; i < 3; i++) /* A <- J^T A */
            {
                float api = A[p][i], aqi = A[q][i];
                A[p][i] = c * api - s * aqi;
                A[q][i] = s * api + c * aqi;
            }
            for (int i = 0; i < 3; i++) /* V <- V J */
            {
                float vip = V[i][p], viq = V[i][q];
                V[i][p] = c * vip - s * viq;
                V[i][q] = s * vip + c * viq;
            }
        }
    }

    w[0] = A[0][0];
    w[1] = A[1][1];
    w[2] = A[2][2];

    /* 昇順にソート (固有ベクトルの列も一緒に入れ替える) */
    for (int i = 0; i < 2; i++)
    {
        int m = i;
        for (int j = i + 1; j < 3; j++)
        {
            if (w[j] < w[m])
            {
                m = j;
            }
        }
        if (m != i)
        {
            float tw = w[i];
            w[i] = w[m];
            w[m] = tw;
            for (int r = 0; r < 3; r++)
            {
                float tv = V[r][i];
                V[r][i] = V[r][m];
                V[r][m] = tv;
            }
        }
    }
}

/* ============================================================
 * 3.1 PCA + RANSAC 平面抽出
 * ============================================================ */

/**
 * 点群 P (idx で選ばれた n 点) から PCA で平面を求める。
 * n は原点側を向く単位法線、d = n^T p̄ > 0。
 */
static void pcaPlane(const float P[NZONE][3], const uint8_t *idx, int n,
                     float normal[3], float *dOut, float *planarityOut)
{
    float pbar[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < n; i++)
    {
        for (int a = 0; a < 3; a++)
        {
            pbar[a] += P[idx[i]][a];
        }
    }
    for (int a = 0; a < 3; a++)
    {
        pbar[a] /= (float)n;
    }

    float C[3][3] = {{0}};
    for (int i = 0; i < n; i++)
    {
        float q[3];
        for (int a = 0; a < 3; a++)
        {
            q[a] = P[idx[i]][a] - pbar[a];
        }
        for (int a = 0; a < 3; a++)
        {
            for (int b = a; b < 3; b++)
            {
                C[a][b] += q[a] * q[b];
            }
        }
    }
    for (int a = 0; a < 3; a++)
    {
        for (int b = a; b < 3; b++)
        {
            C[a][b] /= (float)n;
            C[b][a] = C[a][b];
        }
    }

    float w[3], V[3][3];
    eigenSym3(C, w, V); /* 昇順 */

    normal[0] = V[0][0];
    normal[1] = V[1][0];
    normal[2] = V[2][0]; /* 最小固有値の固有ベクトル */

    float sum = w[0] + w[1] + w[2];
    *planarityOut = w[0] / ((sum > 1e-12f) ? sum : 1e-12f);

    float d = vec3Dot(normal, pbar);
    if (d < 0.0f) /* 法線の向きを「原点→平面」に統一 */
    {
        normal[0] = -normal[0];
        normal[1] = -normal[1];
        normal[2] = -normal[2];
        d = -d;
    }
    *dOut = d;
}

/* RANSAC 用の軽量な擬似乱数 (xorshift32)。 */
static uint32_t s_rngState = 0x12345678u;
static uint32_t rngNext(void)
{
    uint32_t x = s_rngState;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rngState = x;
    return x;
}

/** RANSAC で外れ値を除いてから PCA 平面フィット。失敗時は false。 */
static bool fitPlaneRansac(const float P[NZONE][3], int N, Plane *out)
{
    if (N < MIN_INLIERS)
    {
        return false;
    }

    uint8_t bestInl[NZONE];
    int bestCount = 0;

    for (int it = 0; it < RANSAC_ITERS; it++)
    {
        /* 相異なる3点を選ぶ (N>=MIN_INLIERS>=3)。衝突したら捨てるのではなく
         * 残りの候補から選び直すことで、試行回数を無駄にしない。 */
        int i0 = (int)(rngNext() % (uint32_t)N);
        int i1 = (int)(rngNext() % (uint32_t)(N - 1));
        if (i1 >= i0)
        {
            i1++;
        }
        int i2 = (int)(rngNext() % (uint32_t)(N - 2));
        int lo = (i0 < i1) ? i0 : i1;
        int hi = (i0 < i1) ? i1 : i0;
        if (i2 >= lo)
        {
            i2++;
        }
        if (i2 >= hi)
        {
            i2++;
        }

        float e1[3], e2[3], nrm[3];
        for (int a = 0; a < 3; a++)
        {
            e1[a] = P[i1][a] - P[i0][a];
            e2[a] = P[i2][a] - P[i0][a];
        }
        vec3Cross(e1, e2, nrm);
        if (!vec3Normalize(nrm))
        {
            continue;
        }

        uint8_t inl[NZONE];
        int count = 0;
        for (int k = 0; k < N; k++)
        {
            float diff[3];
            for (int a = 0; a < 3; a++)
            {
                diff[a] = P[k][a] - P[i0][a];
            }
            if (fabsf(vec3Dot(diff, nrm)) < RANSAC_THRESH_MM)
            {
                inl[count++] = (uint8_t)k;
            }
        }
        if (count > bestCount)
        {
            bestCount = count;
            memcpy(bestInl, inl, (size_t)count * sizeof(uint8_t));
            if (count == N)
            {
                break;
            }
        }
    }

    if (bestCount < MIN_INLIERS)
    {
        return false;
    }

    /* インライアで最終 PCA (2回リファイン) */
    float normal[3], d, planarity;
    for (int r = 0; r < 2; r++)
    {
        pcaPlane(P, bestInl, bestCount, normal, &d, &planarity);

        uint8_t newInl[NZONE];
        int newCount = 0;
        for (int k = 0; k < N; k++)
        {
            if (fabsf(vec3Dot(P[k], normal) - d) < RANSAC_THRESH_MM)
            {
                newInl[newCount++] = (uint8_t)k;
            }
        }
        if (newCount < MIN_INLIERS)
        {
            break;
        }
        bestCount = newCount;
        memcpy(bestInl, newInl, (size_t)newCount * sizeof(uint8_t));
    }

    pcaPlane(P, bestInl, bestCount, normal, &d, &planarity);
    if (planarity > PLANARITY_EPS)
    {
        return false;
    }

    memcpy(out->n, normal, sizeof(normal));
    out->d = d;
    out->planarity = planarity;
    out->valid = true;
    return true;
}

/** 1センサー16ゾーンの距離 → ボディ座標系の点群 (有効点のみ)。戻り値は点数。 */
static int pointsBody(int si, float P[NZONE][3])
{
    uint8_t sensor = TOFODO_SENSORS[si];
    int n = 0;
    for (int z = 0; z < NZONE; z++)
    {
        float d = (float)vl53l8cxToFDist[sensor][z];
        if (!(d > D_MIN_MM) || !(d < D_MAX_MM))
        {
            continue;
        }
        for (int a = 0; a < 3; a++)
        {
            P[n][a] = s_ray[si][z][a] * d + SENSOR_MOUNT_T[si][a];
        }
        n++;
    }
    return n;
}

/* ============================================================
 * 3.2 ΔR (Kabsch), 3.3 Δt (Point-to-Plane 最小二乗)
 * ============================================================ */

/**
 * ΔR = argmin_R Σ w_i ||n'_i - R n_i||^2。
 * H = Σ w n_prev n_curr^T の SVD の代わりに H^T H の固有値分解を使い、
 * U/V をともに右手系に揃えることで反射補正 (det) を自動的に織り込む。
 * 戻り値は H の特異値のランク (2未満なら回転は一意でない)。
 */
static int kabschNormals(const float Nprev[TOFODO_NUM_SENSORS][3],
                         const float Ncurr[TOFODO_NUM_SENSORS][3],
                         const float *wgt, int M, float R[3][3])
{
    float H[3][3] = {{0}};
    for (int k = 0; k < M; k++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                H[i][j] += wgt[k] * Nprev[k][i] * Ncurr[k][j];
            }
        }
    }

    float HtH[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            HtH[i][j] = H[0][i] * H[0][j] + H[1][i] * H[1][j] + H[2][i] * H[2][j];
        }
    }

    float lam[3], Vasc[3][3];
    eigenSym3(HtH, lam, Vasc); /* 昇順 */

    /* 降順に並べ替え、V を右手系に揃える */
    float sv[3], V[3][3];
    for (int k = 0; k < 3; k++)
    {
        float l = lam[2 - k];
        sv[k] = (l > 0.0f) ? sqrtf(l) : 0.0f;
        for (int i = 0; i < 3; i++)
        {
            V[i][k] = Vasc[i][2 - k];
        }
    }
    float v0[3] = {V[0][0], V[1][0], V[2][0]};
    float v1[3] = {V[0][1], V[1][1], V[2][1]};
    float v2[3];
    vec3Cross(v0, v1, v2);
    for (int i = 0; i < 3; i++)
    {
        V[i][2] = v2[i];
    }

    float tol = 1e-6f * ((sv[0] > 1e-12f) ? sv[0] : 1e-12f);
    int rank = 0;
    for (int k = 0; k < 3; k++)
    {
        if (sv[k] > tol)
        {
            rank++;
        }
    }
    if (rank < 2)
    {
        mat3Identity(R);
        return rank;
    }

    /* u_k = H v_k / s_k (k=0,1)、u_2 は右手系になるよう外積で補う */
    float u0[3], u1[3], u2[3];
    float vk[3];
    for (int i = 0; i < 3; i++)
    {
        vk[i] = V[i][0];
    }
    mat3MulVec(H, vk, u0);
    if (!vec3Normalize(u0))
    {
        mat3Identity(R);
        return rank;
    }
    for (int i = 0; i < 3; i++)
    {
        vk[i] = V[i][1];
    }
    mat3MulVec(H, vk, u1);
    float proj = vec3Dot(u0, u1); /* 数値誤差の直交化 */
    for (int i = 0; i < 3; i++)
    {
        u1[i] -= proj * u0[i];
    }
    if (!vec3Normalize(u1))
    {
        mat3Identity(R);
        return rank;
    }
    vec3Cross(u0, u1, u2);

    float U[3][3];
    for (int i = 0; i < 3; i++)
    {
        U[i][0] = u0[i];
        U[i][1] = u1[i];
        U[i][2] = u2[i];
    }

    /* R = V U^T */
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = V[i][0] * U[j][0] + V[i][1] * U[j][1] + V[i][2] * U[j][2];
        }
    }
    return rank;
}

/**
 * Σ w_i (Δd_i + n_i^T Δt)^2 の最小化。
 * 正規方程式 M Δt = b (M = Σ w n n^T, b = -Σ w Δd n) を固有値分解し、
 * 特異値が小さい (= 退化した) 方向は 0 とする (truncated SVD 相当)。
 * 戻り値は観測可能な方向の数 (ランク)。
 */
static int solveTranslation(const float Nprev[TOFODO_NUM_SENSORS][3],
                            const float *dd, const float *wgt, int M, float dt[3])
{
    float A[3][3] = {{0}};
    float b[3] = {0.0f, 0.0f, 0.0f};
    for (int k = 0; k < M; k++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                A[i][j] += wgt[k] * Nprev[k][i] * Nprev[k][j];
            }
            b[i] -= wgt[k] * dd[k] * Nprev[k][i];
        }
    }

    float lam[3], V[3][3];
    eigenSym3(A, lam, V); /* 昇順。lam = 特異値の2乗 */

    float lamMax = lam[2];
    /* 特異値比 s_i/s_max > rcond ⟺ λ_i/λ_max > rcond^2 */
    float thresh = TRANS_RCOND * TRANS_RCOND * ((lamMax > 1e-12f) ? lamMax : 1e-12f);

    dt[0] = dt[1] = dt[2] = 0.0f;
    int rank = 0;
    for (int k = 0; k < 3; k++)
    {
        if (lam[k] <= thresh)
        {
            continue;
        }
        float v[3] = {V[0][k], V[1][k], V[2][k]};
        float coeff = vec3Dot(v, b) / lam[k];
        for (int i = 0; i < 3; i++)
        {
            dt[i] += coeff * v[i];
        }
        rank++;
    }
    return rank;
}

/* ============================================================
 * 公開 API
 * ============================================================ */
void tofOdometryInit(void)
{
    /* 各ゾーン中心方向の単位ベクトル (センサー座標系) を取り付け回転でボディ系へ。
     * zone i → row = i/4, col = i%4。depth_map.py と同じく上下左右を反転して
     * 物理的な向き (row↑ = 上, col↑ = 左) に直してから角度を割り当てる。 */
    const float step = (FOV_DEG * DEG2RAD) / (float)GRID;

    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        float Rm[3][3];
        rotYpr(SENSOR_MOUNT_YPR[si][0], SENSOR_MOUNT_YPR[si][1],
               SENSOR_MOUNT_YPR[si][2], Rm);

        for (int i = 0; i < NZONE; i++)
        {
            int row = i / GRID, col = i % GRID;
            float r = (float)((GRID - 1) - row); /* 反転後の行 (大きいほど上) */
            float c = (float)((GRID - 1) - col); /* 反転後の列 (大きいほど左) */
            float az = (c - (GRID - 1) / 2.0f) * step;
            float el = (r - (GRID - 1) / 2.0f) * step;

            float ray[3] = {cosf(el) * cosf(az), cosf(el) * sinf(az), sinf(el)};
            mat3MulVec(Rm, ray, s_ray[si][i]);
        }
    }

    mat3Identity(s_Rcum);
    s_pcum[0] = s_pcum[1] = s_pcum[2] = 0.0f;
    s_havePrev = false;
    s_rngState = 0x12345678u;

    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        s_prevPlane[si].valid = false;
        s_logPlaneD[si] = NANF;
        s_logPlanarity[si] = NANF;
        s_logAzimuth[si] = NANF;
    }
    for (int a = 0; a < 3; a++)
    {
        s_logDypr[a] = NANF;
        s_logDt[a] = NANF;
        s_logYpr[a] = 0.0f;
        s_logPos[a] = 0.0f;
    }
    s_logRankR = s_logRankT = s_logNPairs = 0;
    s_logSeq = 0;
    s_initialized = true;
}

void tofOdometryUpdate(void)
{
    if (!s_initialized)
    {
        tofOdometryInit();
    }
    if (!s_enable)
    {
        return;
    }

    uint64_t tStart = usecTimestamp();

    /* --- 3.1 各センサーの平面抽出 --- */
    /* 点群バッファはタスクスタックを節約するため static (このタスク専用)。 */
    static float P[NZONE][3];
    Plane curr[TOFODO_NUM_SENSORS];
    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        int n = pointsBody(si, P);

        curr[si].valid = false;
        if (n >= MIN_INLIERS)
        {
            fitPlaneRansac(P, n, &curr[si]);
        }

        if (curr[si].valid)
        {
            s_logPlaneD[si] = curr[si].d;
            s_logPlanarity[si] = curr[si].planarity;
            s_logAzimuth[si] = atan2f(curr[si].n[1], curr[si].n[0]) * RAD2DEG;
        }
        else
        {
            s_logPlaneD[si] = NANF;
            s_logPlanarity[si] = NANF;
            s_logAzimuth[si] = NANF;
        }
    }

    /* --- 前フレームと両方で平面が取れたセンサーだけを対応付ける --- */
    /* センサー11個分あるのでこれらもタスクスタックではなく static に置く。 */
    static float Nprev[TOFODO_NUM_SENSORS][3], Ncurr[TOFODO_NUM_SENSORS][3];
    static float wgt[TOFODO_NUM_SENSORS], dd[TOFODO_NUM_SENSORS];
    int M = 0;

    if (s_havePrev)
    {
        for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
        {
            if (!s_prevPlane[si].valid || !curr[si].valid)
            {
                continue;
            }
            memcpy(Nprev[M], s_prevPlane[si].n, sizeof(Nprev[M]));
            memcpy(Ncurr[M], curr[si].n, sizeof(Ncurr[M]));
            /* 重み: 平面性が良いほど (λ3 が小さいほど) 大きい */
            float pl = curr[si].planarity;
            wgt[M] = 1.0f / ((pl > 1e-6f) ? pl : 1e-6f);
            dd[M] = curr[si].d - s_prevPlane[si].d;
            M++;
        }
    }
    s_logNPairs = (uint8_t)M;

    if (M > 0)
    {
        float wsum = 0.0f;
        for (int k = 0; k < M; k++)
        {
            wsum += wgt[k];
        }
        for (int k = 0; k < M; k++)
        {
            wgt[k] /= wsum;
        }

        /* --- 3.2 回転量 ΔR --- */
        float dR[3][3];
        int rankR = kabschNormals(Nprev, Ncurr, wgt, M, dR);
        if (rankR < 2) /* 法線が1方向のみ → 回転は不定なので採用しない */
        {
            mat3Identity(dR);
        }
        s_logRankR = (uint8_t)rankR;
        rotToYpr(dR, s_logDypr);

        /* --- 3.3 移動量 Δt (平面までの垂直距離は原点まわりの回転で不変) --- */
        float dt[3];
        int rankT = solveTranslation(Nprev, dd, wgt, M, dt);
        s_logRankT = (uint8_t)rankT;
        memcpy(s_logDt, dt, sizeof(dt));

        /* --- 累積 --- */
        mat3Mul(s_Rcum, dR, s_Rcum);
        mat3Reorthonormalize(s_Rcum);

        float dtWorld[3];
        mat3MulVec(s_Rcum, dt, dtWorld);
        for (int a = 0; a < 3; a++)
        {
            s_pcum[a] += dtWorld[a];
        }
    }
    else
    {
        s_logRankR = 0;
        s_logRankT = 0;
        for (int a = 0; a < 3; a++)
        {
            s_logDypr[a] = NANF;
            s_logDt[a] = NANF;
        }
    }

    rotToYpr(s_Rcum, s_logYpr);
    memcpy(s_logPos, s_pcum, sizeof(s_pcum));

    memcpy(s_prevPlane, curr, sizeof(curr));
    s_havePrev = true;
    s_logSeq++;
    s_logCalcUs = (uint32_t)(usecTimestamp() - tStart);
}

/**
 * ToF オドメトリの推定結果。usd/tof_odometry.py が描く8枚のグラフを再現するのに
 * 必要な最小セット。累積値 (yaw..pz) は Δ の積分で復元できるが、SD への
 * 書き込みが1フレーム欠けても以降がズレないよう実機側の値もそのまま出す。
 */
LOG_GROUP_START(tofodo)
/** @brief sensor0 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d0, &s_logPlaneD[0])
/** @brief sensor1 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d1, &s_logPlaneD[1])
/** @brief sensor2 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d2, &s_logPlaneD[2])
/** @brief sensor3 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d3, &s_logPlaneD[3])
/** @brief sensor4 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d4, &s_logPlaneD[4])
/** @brief sensor5 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d5, &s_logPlaneD[5])
/** @brief sensor6 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d6, &s_logPlaneD[6])
/** @brief sensor7 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d7, &s_logPlaneD[7])
/** @brief sensor8 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d8, &s_logPlaneD[8])
/** @brief sensor9 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d9, &s_logPlaneD[9])
/** @brief sensor10 の平面距離 d [mm] (抽出失敗時は NaN) */
LOG_ADD(LOG_FLOAT, d10, &s_logPlaneD[10])
/** @brief sensor0 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl0, &s_logPlanarity[0])
/** @brief sensor1 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl1, &s_logPlanarity[1])
/** @brief sensor2 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl2, &s_logPlanarity[2])
/** @brief sensor3 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl3, &s_logPlanarity[3])
/** @brief sensor4 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl4, &s_logPlanarity[4])
/** @brief sensor5 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl5, &s_logPlanarity[5])
/** @brief sensor6 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl6, &s_logPlanarity[6])
/** @brief sensor7 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl7, &s_logPlanarity[7])
/** @brief sensor8 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl8, &s_logPlanarity[8])
/** @brief sensor9 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl9, &s_logPlanarity[9])
/** @brief sensor10 の平面性スコア λ3/Σλ */
LOG_ADD(LOG_FLOAT, pl10, &s_logPlanarity[10])
/** @brief sensor0 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az0, &s_logAzimuth[0])
/** @brief sensor1 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az1, &s_logAzimuth[1])
/** @brief sensor2 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az2, &s_logAzimuth[2])
/** @brief sensor3 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az3, &s_logAzimuth[3])
/** @brief sensor4 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az4, &s_logAzimuth[4])
/** @brief sensor5 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az5, &s_logAzimuth[5])
/** @brief sensor6 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az6, &s_logAzimuth[6])
/** @brief sensor7 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az7, &s_logAzimuth[7])
/** @brief sensor8 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az8, &s_logAzimuth[8])
/** @brief sensor9 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az9, &s_logAzimuth[9])
/** @brief sensor10 の法線の方位角 [deg] (ボディ座標系) */
LOG_ADD(LOG_FLOAT, az10, &s_logAzimuth[10])
/** @brief フレーム間回転量 Δyaw [deg/frame] */
LOG_ADD(LOG_FLOAT, dyaw, &s_logDypr[0])
/** @brief フレーム間回転量 Δpitch [deg/frame] */
LOG_ADD(LOG_FLOAT, dpitch, &s_logDypr[1])
/** @brief フレーム間回転量 Δroll [deg/frame] */
LOG_ADD(LOG_FLOAT, droll, &s_logDypr[2])
/** @brief フレーム間移動量 Δx [mm/frame] */
LOG_ADD(LOG_FLOAT, dx, &s_logDt[0])
/** @brief フレーム間移動量 Δy [mm/frame] */
LOG_ADD(LOG_FLOAT, dy, &s_logDt[1])
/** @brief フレーム間移動量 Δz [mm/frame] */
LOG_ADD(LOG_FLOAT, dz, &s_logDt[2])
/** @brief 累積回転 yaw [deg] */
LOG_ADD(LOG_FLOAT, yaw, &s_logYpr[0])
/** @brief 累積回転 pitch [deg] */
LOG_ADD(LOG_FLOAT, pitch, &s_logYpr[1])
/** @brief 累積回転 roll [deg] */
LOG_ADD(LOG_FLOAT, roll, &s_logYpr[2])
/** @brief 累積移動 x [mm] */
LOG_ADD(LOG_FLOAT, px, &s_logPos[0])
/** @brief 累積移動 y [mm] */
LOG_ADD(LOG_FLOAT, py, &s_logPos[1])
/** @brief 累積移動 z [mm] */
LOG_ADD(LOG_FLOAT, pz, &s_logPos[2])
/** @brief ΔR の観測可能な方向数 (2未満なら回転は不定) */
LOG_ADD(LOG_UINT8, rankR, &s_logRankR)
/** @brief Δt の観測可能な方向数 (3未満ならその軸は不定) */
LOG_ADD(LOG_UINT8, rankT, &s_logRankT)
/** @brief 前フレームと対応が取れたセンサー数 */
LOG_ADD(LOG_UINT8, npair, &s_logNPairs)
/** @brief 処理済みフレーム数 (SD の欠落検出用) */
LOG_ADD(LOG_UINT32, seq, &s_logSeq)
/** @brief 1フレームの計算時間 [us] */
LOG_ADD(LOG_UINT32, calcUs, &s_logCalcUs)
LOG_GROUP_STOP(tofodo)

PARAM_GROUP_START(tofodo)
/** @brief 0 にすると推定を停止する */
PARAM_ADD(PARAM_UINT8, enable, &s_enable)
PARAM_GROUP_STOP(tofodo)
