/**
 * 直方体チャンバー内で、機体が壁に対して何度ずれているかを ToF だけで求める。
 *
 * usd/tof_wall_angle.py のオンボード実装。ジャイロ・EKF は一切使わないので
 * 積分ドリフトが無く、絶対角がそのまま得られる。
 *
 *   1. 各センサーの 16 ゾーンを PCA+RANSAC で平面フィットし、壁の法線を得る
 *      (tof_odometry.c の tofOdometryFitSensorPlane() を再利用)
 *   2. ボディ座標での法線方位角 a = atan2(n_y, n_x) を求める
 *   3. 直方体の壁は 90 度おきなので、exp(4ja) の円環平均で「どの壁を見ている
 *      か」を識別せずに全センサーの観測を1つの角度に融合する
 *      → 結果は [-45,+45) 度 = 最も近い壁面に対する機体のヨーずれ
 *   4. 平均から大きく外れた面は棄却。sensor0/2 が別々の壁を見ている場合は
 *      sensor1 および隣 (0→5, 2→6) との比較で誤った側を落とす
 *   5. 複素平面のまま指数移動平均で平滑化する
 *
 * tofWallAngleUpdate() を新しい測距フレームごとに1回呼ぶ。
 * 結果は log グループ "tofwall" と tofWallAngleGet() で参照できる。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tof_odometry.h" /* TOFODO_NUM_SENSORS */

/* 0 にすると本モジュールをビルドしても何もしない (既存動作のまま)。 */
#ifndef TOF_WALL_ANGLE_ENABLED
#define TOF_WALL_ANGLE_ENABLED 1
#endif

/** 壁角推定の結果。tofWallAngleGet() でまとめて取得する。 */
typedef struct
{
    /* --- 機体全体の推定 --- */
    /** 機体前方 (+x) と最も近い壁の法線とのヨーずれ [-45,+45) deg。
     *  0 = 壁と正対。無効なら NaN。 */
    float bodyDeg;
    /** bodyDeg を指数移動平均で平滑化した値 [deg]。制御にはこちらを使う。 */
    float bodyFiltDeg;
    /** 面同士の整合度 0〜1 (1 に近いほど全ての面が同じ答えを出している)。 */
    float conf;
    /** このフレームで bodyDeg が求まったか。 */
    bool valid;
    /** 融合に採用した面数 / 平面が取れた面数。 */
    uint8_t nUsed;
    uint8_t nPlanes;
    /** sensor0/2 が別々の壁を見ていると判定されたフレームか。 */
    bool frontConflict;

    /* --- センサーごとの推定 (添字は TOFODO_SENSORS[] の並び 0,1,2,5,6,7) --- */
    /** そのセンサーの光軸と、見ている壁の法線とのずれ [deg]。
     *  0 = そのセンサーが壁と正対。取り付け角 (sensor0 = -22.5, sensor2 = +22.5)
     *  は差し引き済みなので、6 個すべて同じ意味で読める。無効なら NaN。 */
    float sensorDeg[TOFODO_NUM_SENSORS];
    /** そのセンサーの法線のボディ座標系での方位角 [deg] (取り付け角込み)。 */
    float sensorAzimuthDeg[TOFODO_NUM_SENSORS];
    /** そのセンサーで平面が取れ、かつ融合に採用されたか。 */
    bool sensorValid[TOFODO_NUM_SENSORS];
    bool sensorUsed[TOFODO_NUM_SENSORS];
} tofWallAngle_t;

/** 内部状態 (平滑化フィルタ) を初期化する。 */
void tofWallAngleInit(void);

/** 1測距フレーム分の推定を行う。最新の距離データを読んで結果を更新する。 */
void tofWallAngleUpdate(void);

/** 最新の結果をコピーする。まだ有効な推定が無ければ false。 */
bool tofWallAngleGet(tofWallAngle_t *out);

/** 平滑化後の機体のヨーずれ [deg] を返す。無効なら NaN。 */
float tofWallAngleGetBodyDeg(void);

/** 平滑化後の推定が使える状態か (集中度が閾値以上で、直近に更新がある)。 */
bool tofWallAngleIsValid(void);
