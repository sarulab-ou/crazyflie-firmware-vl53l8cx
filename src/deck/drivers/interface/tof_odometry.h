/**
 * 超低密度ToF点群 (VL53L8CX 4x4 = 16点/センサー) からの
 * 移動量 (Δt) ・回転量 (ΔR) の幾何学的抽出。
 *
 * usd/tof_odometry.py のオンボード実装。アルゴリズムは同論文 3章:
 *   3.1 PCA + RANSAC による平面抽出
 *   3.2 法線ベクトル群からの回転量 ΔR の推定 (Kabsch / SVD)
 *   3.3 Point-to-Plane 誤差モデルによる移動量 Δt の推定 (truncated SVD)
 *
 * tofOdometryUpdate() を新しい測距フレームごとに1回呼ぶ。結果は log グループ
 * "tofodo" として公開され、microSD card deck で記録できる。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 基板に搭載されている VL53L8CX の数 (取り付けテーブルの行数)。 */
#define VL53L8CX_MAX_SENSORS 11

/* オドメトリに使うセンサー数 (tof_odometry.c の TOFODO_SENSORS と要一致)。 */
#define TOFODO_NUM_SENSORS 6

/** 内部状態 (累積姿勢・位置・前フレームの平面) を初期化する。 */
void tofOdometryInit(void);

/**
 * 1測距フレーム分の推定を行う。
 * vl53l8cxToFDist[] / vl53l8cxToFStatus[] の最新値を読み、log 変数を更新する。
 */
void tofOdometryUpdate(void);
