/**
 * 直方体チャンバー内での「壁に対する機体のヨーずれ」を ToF だけで推定する。
 * 詳細は tof_wall_angle.h を参照。usd/tof_wall_angle.py のオンボード実装。
 */
#include "tof_wall_angle.h"

#include <math.h>
#include <string.h>

#include "log.h"
#include "param.h"
#include "tof_odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (float)(M_PI / 180.0)
#define RAD2DEG (float)(180.0 / M_PI)
#define NANF    ((float)NAN)

/* 直方体なので壁は 90 度おき。4 倍角にして円環平均すると、どの壁を見ているか
 * を識別せずに融合できる。 */
#define WALL_PERIOD_DEG 90.0f
#define WALL_K          4.0f

/* 法線がこれ以上鉛直に近い面は「垂直な壁ではない」(天井/床) として使わない。 */
#define NZ_MAX 0.5f

/* 融合平均からこの角度以上ずれた面は外れ値として棄却する [deg]。 */
#define REJECT_DEG   15.0f
#define REJECT_ITERS 2

/* sensor0 と sensor2 が別々の壁を見ていると判定する角度差 [deg]。
 * 斜め 22.5 度を向いた 0/2 は狭いチャンバーでは隣接2面の角に差し掛かりやすく、
 * 実測 (log54/57/58) では 13〜35% のフレームで発生した。 */
#define CONFLICT_DEG   30.0f
/* 2つの面を「同じ壁」とみなす角度差 [deg]。 */
#define SAME_PLANE_DEG 10.0f

/* 指数移動平均の係数。測距 5Hz で 0.4 ≒ 時定数 0.5s 程度。 */
#define EMA_ALPHA_DEFAULT 0.4f

/* この集中度を下回る推定は信用しない。 */
#define CONF_MIN_DEFAULT 0.70f

/* 推定が途切れてこのフレーム数を超えたら平滑値を無効に戻す。 */
#define STALE_FRAMES 10

/* ---- 内部状態 ---- */
static tofWallAngle_t s_res;
static float s_emaRe = 0.0f; /* 平滑化した exp(4ja) の実部・虚部 */
static float s_emaIm = 0.0f;
static bool s_emaOk = false;
static uint16_t s_staleCount = 0;
static bool s_initialized = false;

/* ---- パラメータ (param グループ tofwall) ---- */
static uint8_t s_enable = 1;
static uint8_t s_frontReject = 1; /* sensor0/2 の食い違い棄却を行うか */
static float s_emaAlpha = EMA_ALPHA_DEFAULT;
static float s_confMin = CONF_MIN_DEFAULT;

/* ---- log 用 ---- */
static float s_logSensorDeg[TOFODO_NUM_SENSORS];
static float s_logAzimuth[TOFODO_NUM_SENSORS];
static uint8_t s_logUsedMask = 0; /* bit si = 融合に採用された */
static uint32_t s_logSeq = 0;

/* ============================================================
 * 角度ユーティリティ
 * ============================================================ */

/** 角度を [-180,180) deg に畳む。 */
static float wrap180(float deg)
{
    while (deg >= 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

/** 角度を [-45,+45) deg (壁の周期 90 度の代表値) に畳む。 */
static float wrapWall(float deg)
{
    while (deg >= WALL_PERIOD_DEG / 2.0f)
    {
        deg -= WALL_PERIOD_DEG;
    }
    while (deg < -WALL_PERIOD_DEG / 2.0f)
    {
        deg += WALL_PERIOD_DEG;
    }
    return deg;
}

/** 2つの方位角の差の絶対値 [0,180] deg。同じ壁なら 0 付近、隣の壁なら 90 付近。 */
static float angDiff(float a, float b)
{
    return fabsf(wrap180(a - b));
}

/* ============================================================
 * sensor0 / sensor2 が別々の壁を見たときの棄却
 * ============================================================ */

/** TOFODO_SENSORS[] の中から物理センサー番号 id の添字を探す。無ければ -1。 */
static int indexOfSensor(uint8_t id)
{
    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        if (tofOdometryGetSensorId(si) == id)
        {
            return si;
        }
    }
    return -1;
}

/**
 * sensor0 と sensor2 が CONFLICT_DEG 以上ずれている場合に、誤った側を棄却する。
 *
 *   - sensor1 と異なる面を見ている (差が SAME_PLANE_DEG 以上)、または
 *   - sensor1 ではない方の隣 (0→5, 2→6) と同じ面を見ている (差が
 *     SAME_PLANE_DEG 未満。横のセンサーと同じ壁 = 正面ではなく横を見ている)
 *   に当てはまる方を落とす。どちらも判定できなければ両方落とす。
 *
 * ok[] を書き換え、食い違いが起きていれば true を返す。
 */
static bool rejectFrontConflict(const float *az, bool *ok)
{
    const uint8_t pair[2] = {0, 2};
    const uint8_t neighbor[2] = {5, 6}; /* sensor1 ではない方の隣 */

    int i0 = indexOfSensor(pair[0]);
    int i2 = indexOfSensor(pair[1]);
    int i1 = indexOfSensor(1);
    if (i0 < 0 || i2 < 0 || !ok[i0] || !ok[i2])
    {
        return false;
    }
    if (angDiff(az[i0], az[i2]) < CONFLICT_DEG)
    {
        return false;
    }

    bool drop[2] = {false, false};
    for (int k = 0; k < 2; k++)
    {
        int si = (k == 0) ? i0 : i2;
        /* sensor1 と異なる面を見ている */
        if (i1 >= 0 && ok[i1] && angDiff(az[si], az[i1]) >= SAME_PLANE_DEG)
        {
            drop[k] = true;
            continue;
        }
        /* 外側の隣と同じ面を見ている */
        int nb = indexOfSensor(neighbor[k]);
        if (nb >= 0 && ok[nb] && angDiff(az[si], az[nb]) < SAME_PLANE_DEG)
        {
            drop[k] = true;
        }
    }

    if (!drop[0] && !drop[1])
    {
        /* どちらが正しいか判断できないので両方捨てる */
        drop[0] = drop[1] = true;
    }
    if (drop[0])
    {
        ok[i0] = false;
    }
    if (drop[1])
    {
        ok[i2] = false;
    }
    return true;
}

/* ============================================================
 * 4倍角の円環平均による融合
 * ============================================================ */

/**
 * ok[] が立っている面の方位角を 4倍角の円環平均で融合し、外れ値を反復除去する。
 *
 * angleDeg : 融合結果 [-45,+45) deg
 * conf     : 集中度 |z| (0〜1)
 * 戻り値   : 採用した面数 (0 なら推定不能)
 */
static int fuseAngles(const float *az, bool *ok, float *angleDeg, float *conf)
{
    float mean = 0.0f;
    float magnitude = 0.0f;
    int n = 0;

    for (int iter = 0; iter <= REJECT_ITERS; iter++)
    {
        float sre = 0.0f, sim = 0.0f;
        n = 0;
        for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
        {
            if (!ok[si])
            {
                continue;
            }
            float a4 = WALL_K * az[si] * DEG2RAD;
            sre += cosf(a4);
            sim += sinf(a4);
            n++;
        }
        if (n == 0)
        {
            *angleDeg = NANF;
            *conf = 0.0f;
            return 0;
        }

        sre /= (float)n;
        sim /= (float)n;
        magnitude = sqrtf(sre * sre + sim * sim);
        mean = atan2f(sim, sre) * RAD2DEG / WALL_K;

        if (iter == REJECT_ITERS || n <= 2)
        {
            break;
        }

        /* 平均から REJECT_DEG 以上外れた面を落とす */
        bool changed = false;
        int remaining = 0;
        for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
        {
            if (!ok[si])
            {
                continue;
            }
            if (fabsf(wrapWall(az[si] - mean)) > REJECT_DEG)
            {
                changed = true;
            }
            else
            {
                remaining++;
            }
        }
        if (!changed || remaining < 2)
        {
            break;
        }
        for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
        {
            if (ok[si] && fabsf(wrapWall(az[si] - mean)) > REJECT_DEG)
            {
                ok[si] = false;
            }
        }
    }

    *angleDeg = wrapWall(mean);
    *conf = magnitude;
    return n;
}

/* ============================================================
 * 公開 API
 * ============================================================ */
void tofWallAngleInit(void)
{
    memset(&s_res, 0, sizeof(s_res));
    s_res.bodyDeg = NANF;
    s_res.bodyFiltDeg = NANF;
    s_res.conf = 0.0f;
    s_res.valid = false;
    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        s_res.sensorDeg[si] = NANF;
        s_res.sensorAzimuthDeg[si] = NANF;
        s_logSensorDeg[si] = NANF;
        s_logAzimuth[si] = NANF;
    }
    s_emaRe = s_emaIm = 0.0f;
    s_emaOk = false;
    s_staleCount = 0;
    s_logUsedMask = 0;
    s_logSeq = 0;
    s_initialized = true;
}

void tofWallAngleUpdate(void)
{
    if (!s_initialized)
    {
        tofWallAngleInit();
    }
    if (!s_enable)
    {
        return;
    }

    float az[TOFODO_NUM_SENSORS];
    bool ok[TOFODO_NUM_SENSORS];
    int nPlanes = 0;

    /* --- 1. 各センサーの平面から法線の方位角を求める --- */
    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        az[si] = NANF;
        ok[si] = false;
        s_res.sensorDeg[si] = NANF;
        s_res.sensorAzimuthDeg[si] = NANF;
        s_res.sensorValid[si] = false;
        s_res.sensorUsed[si] = false;

        float n[3], d, planarity;
        if (!tofOdometryFitSensorPlane(si, n, &d, &planarity))
        {
            continue;
        }
        if (fabsf(n[2]) > NZ_MAX)
        {
            continue; /* 天井/床の平面は方位角を持たない */
        }

        az[si] = atan2f(n[1], n[0]) * RAD2DEG;
        ok[si] = true;
        nPlanes++;

        /* そのセンサーの光軸から見たずれ。取り付け yaw を差し引くので、
         * sensor0 (-22.5) / sensor2 (+22.5) も sensor1/5/6/7 と同じ意味
         * 「そのセンサーが壁と正対していれば 0」になる。 */
        s_res.sensorAzimuthDeg[si] = az[si];
        s_res.sensorDeg[si] = wrap180(az[si] - tofOdometryGetMountYawDeg(si));
        s_res.sensorValid[si] = true;
    }

    /* --- 2. sensor0/2 が別々の壁を見ているフレームの棄却 --- */
    s_res.frontConflict = false;
    if (s_frontReject)
    {
        s_res.frontConflict = rejectFrontConflict(az, ok);
    }

    /* --- 3. 4倍角の円環平均で融合 --- */
    float angle, conf;
    int nUsed = fuseAngles(az, ok, &angle, &conf);

    s_res.nPlanes = (uint8_t)nPlanes;
    s_res.nUsed = (uint8_t)nUsed;
    s_res.bodyDeg = (nUsed > 0) ? angle : NANF;
    s_res.conf = conf;
    s_res.valid = (nUsed > 0);

    s_logUsedMask = 0;
    for (int si = 0; si < TOFODO_NUM_SENSORS; si++)
    {
        s_res.sensorUsed[si] = ok[si];
        if (ok[si])
        {
            s_logUsedMask |= (uint8_t)(1u << si);
        }
        s_logSensorDeg[si] = s_res.sensorDeg[si];
        s_logAzimuth[si] = s_res.sensorAzimuthDeg[si];
    }

    /* --- 4. 複素平面 (4倍角) のまま指数移動平均で平滑化 --- */
    if (nUsed > 0 && conf >= s_confMin)
    {
        float a4 = WALL_K * angle * DEG2RAD;
        float re = cosf(a4), im = sinf(a4);
        if (!s_emaOk)
        {
            s_emaRe = re;
            s_emaIm = im;
            s_emaOk = true;
        }
        else
        {
            s_emaRe += s_emaAlpha * (re - s_emaRe);
            s_emaIm += s_emaAlpha * (im - s_emaIm);
        }
        s_staleCount = 0;
    }
    else if (s_emaOk)
    {
        s_staleCount++;
        if (s_staleCount > STALE_FRAMES)
        {
            s_emaOk = false; /* 長く更新が無いので平滑値を捨てる */
        }
    }

    s_res.bodyFiltDeg =
        s_emaOk ? wrapWall(atan2f(s_emaIm, s_emaRe) * RAD2DEG / WALL_K) : NANF;
    s_logSeq++;
}

bool tofWallAngleGet(tofWallAngle_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    *out = s_res;
    return s_emaOk;
}

float tofWallAngleGetBodyDeg(void)
{
    return s_emaOk ? s_res.bodyFiltDeg : NANF;
}

bool tofWallAngleIsValid(void)
{
    return s_emaOk;
}

/* ============================================================
 * log / param
 * ============================================================ */
LOG_GROUP_START(tofwall)
/** @brief 壁に対する機体のヨーずれ [-45,45] deg (このフレームの生値)。0 = 正対 */
LOG_ADD(LOG_FLOAT, angle, &s_res.bodyDeg)
/** @brief 上を指数移動平均で平滑化した値 [deg]。制御にはこちらを使う */
LOG_ADD(LOG_FLOAT, angleFilt, &s_res.bodyFiltDeg)
/** @brief 面同士の整合度 0〜1 (1 に近いほど全ての面が一致) */
LOG_ADD(LOG_FLOAT, conf, &s_res.conf)
/** @brief 融合に採用した面数 */
LOG_ADD(LOG_UINT8, nUsed, &s_res.nUsed)
/** @brief 平面が取れた面数 */
LOG_ADD(LOG_UINT8, nPlanes, &s_res.nPlanes)
/** @brief 融合に採用した面のビットマスク (bit0=s0, bit1=s1, bit2=s2, bit3=s5, bit4=s6, bit5=s7) */
LOG_ADD(LOG_UINT8, usedMask, &s_logUsedMask)
/** @brief sensor0/2 が別々の壁を見ていると判定されたフレームか */
LOG_ADD(LOG_UINT8, conflict, &s_res.frontConflict)
/** @brief 処理したフレーム数 */
LOG_ADD(LOG_UINT32, seq, &s_logSeq)
/** @brief sensor0 の光軸と壁法線のずれ [deg] (取り付け -22.5 度を差し引き済み) */
LOG_ADD(LOG_FLOAT, a0, &s_logSensorDeg[0])
/** @brief sensor1 の光軸と壁法線のずれ [deg] */
LOG_ADD(LOG_FLOAT, a1, &s_logSensorDeg[1])
/** @brief sensor2 の光軸と壁法線のずれ [deg] (取り付け +22.5 度を差し引き済み) */
LOG_ADD(LOG_FLOAT, a2, &s_logSensorDeg[2])
/** @brief sensor5 の光軸と壁法線のずれ [deg] */
LOG_ADD(LOG_FLOAT, a5, &s_logSensorDeg[3])
/** @brief sensor6 の光軸と壁法線のずれ [deg] */
LOG_ADD(LOG_FLOAT, a6, &s_logSensorDeg[4])
/** @brief sensor7 の光軸と壁法線のずれ [deg] */
LOG_ADD(LOG_FLOAT, a7, &s_logSensorDeg[5])
LOG_GROUP_STOP(tofwall)

PARAM_GROUP_START(tofwall)
/** @brief 0 にすると壁角推定を停止する */
PARAM_ADD(PARAM_UINT8, enable, &s_enable)
/** @brief 0 にすると sensor0/2 の食い違い棄却を行わない */
PARAM_ADD(PARAM_UINT8, frontRej, &s_frontReject)
/** @brief 平滑化の指数移動平均係数 (大きいほど追従が速い) */
PARAM_ADD(PARAM_FLOAT, emaAlpha, &s_emaAlpha)
/** @brief この集中度を下回るフレームは平滑値に取り込まない */
PARAM_ADD(PARAM_FLOAT, confMin, &s_confMin)
PARAM_GROUP_STOP(tofwall)
