# -*- coding: utf-8 -*-
"""
log_data1.csv の加速度・ジャーク・速度解析

このスクリプトは以下を実施します：
1. log_data1.csvを読み込み（Shift-JIS対応）
2. ジョイント電流値から加速度・ジャークを計算
3. 速度プロファイルを計算
4. 結果をCSVに出力
"""

import pandas as pd
import numpy as np
from pathlib import Path
import json
from scipy import signal, integrate

# パス設定
CSV_FILE = Path(__file__).parent.parent / "log" / "log_data1.csv"
RESULTS_DIR = Path(__file__).parent.parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)

def load_log_data():
    """
    log_data1.csvをロード

    Row 0: メタデータ（日時、ロボット型式、モード等）
    Row 1: 日本語ヘッダー
    Row 2: ユニット行（Arms, %, deg等）
    Row 3以降: データ
    """
    # utf-8-sigで読み込み（日本語ヘッダーは文字化けのため、ユニット行をカラム名として使用）
    df = pd.read_csv(CSV_FILE, encoding='utf-8-sig', skiprows=3)

    # カラム名を生成（時刻と各ユニットで構成）
    units = ['time', 'unknown', 'unknown',
             'current_J1', 'current_J2', 'current_J3', 'current_J4',
             'current_J5', 'current_J6', 'current_J7', 'current_J8',
             'current_J1_max', 'current_J2_max', 'current_J3_max', 'current_J4_max',
             'current_J5_max', 'current_J6_max', 'current_J7_max', 'current_J8_max',
             'current_accel_J1', 'current_accel_J2', 'current_accel_J3', 'current_accel_J4',
             'current_accel_J5', 'current_accel_J6', 'current_accel_J7', 'current_accel_J8']

    # カラム名を設定
    for i in range(len(units)):
        if i < len(df.columns):
            df.columns.values[i] = units[i]

    return df

def identify_joint_columns(df):
    """
    ジョイント関連の列を特定

    パターン：
    - J1-J8: 8個のジョイント
    - Arms: 電流値（FB：フィードバック）
    - Pulse: パルス値
    - deg: 角度値
    """
    joint_columns = {
        'time': 'time',
        'current_fb': [col for col in df.columns if 'current_J' in col and '_max' not in col and '_accel' not in col],
        'current_max': [col for col in df.columns if 'current_J' in col and '_max' in col],
        'current_accel': [col for col in df.columns if '_accel' in col],
        'pulse': [col for col in df.columns if 'Pulse' in str(col)],
        'angle': [col for col in df.columns if 'deg' in str(col)]
    }

    return joint_columns

def calculate_velocity_from_current(current_values, dt=0.005031, smoothing_window=5):
    """
    電流値から速度を推定

    簡易的には、電流値の時間変化から加速度を推定し、
    それを積分して速度を計算

    Args:
        current_values: 電流値配列
        dt: サンプリング時間間隔（秒）
        smoothing_window: 平滑化ウィンドウサイズ

    Returns:
        velocity: 速度配列
    """
    # NaN値を補間
    current_clean = pd.Series(current_values).interpolate(method='linear').ffill().bfill()

    # 加速度を計算（1階差分）
    acceleration = np.gradient(current_clean.values, dt)

    # 平滑化（移動平均）
    if smoothing_window > 1:
        acceleration_smooth = pd.Series(acceleration).rolling(window=smoothing_window, center=True).mean().values
    else:
        acceleration_smooth = acceleration

    # 速度を計算（積分）
    # 累積台形則で積分
    velocity = np.cumsum(acceleration_smooth) * dt

    return velocity, acceleration_smooth

def calculate_jerk(acceleration_values, dt=0.005031, smoothing_window=3):
    """
    加速度値からジャークを計算

    ジャーク = d(加速度)/dt

    Args:
        acceleration_values: 加速度配列
        dt: サンプリング時間間隔
        smoothing_window: 平滑化ウィンドウ

    Returns:
        jerk: ジャーク配列
    """
    # NaN値を補間
    accel_clean = pd.Series(acceleration_values).interpolate(method='linear').ffill().bfill()

    # ジャークを計算（1階差分）
    jerk = np.gradient(accel_clean.values, dt)

    # 平滑化
    if smoothing_window > 1:
        jerk_smooth = pd.Series(jerk).rolling(window=smoothing_window, center=True).mean().values
    else:
        jerk_smooth = jerk

    return jerk_smooth

def main():
    print("=" * 70)
    print(" log_data1.csv 加速度・ジャーク・速度 解析")
    print("=" * 70)

    # [1] CSVを読み込み
    print("\n[1] ログファイルを読み込み中...")
    df = load_log_data()
    print(f"✓ データを読み込み: {len(df)} 行 × {len(df.columns)} 列")

    # 時刻列を取得
    time_col = df.columns[0]
    time_values = df[time_col].values / 1000.0  # ミリ秒から秒に変換
    dt_mean = np.mean(np.diff(time_values))
    print(f"✓ 時刻範囲: {time_values[0]:.3f} - {time_values[-1]:.3f} 秒")
    print(f"✓ サンプリング間隔: {dt_mean:.6f} 秒 ({1/dt_mean:.1f} Hz)")

    # [2] ジョイント列を特定
    print("\n[2] ジョイント関連の列を特定中...")
    joint_info = identify_joint_columns(df)
    print(f"✓ 電流値（FB）: {len(joint_info['current_fb'])} 列")
    print(f"✓ パルス値: {len(joint_info['pulse'])} 列")

    # [3] 解析対象を選定（最初のジョイント電流を使用）
    if joint_info['current_fb']:
        target_column = joint_info['current_fb'][0]
        print(f"\n[3] 解析対象: {target_column}")

        current_raw = df[target_column].values.astype(float)
        print(f"✓ 電流値: min={np.nanmin(current_raw):.3f}, max={np.nanmax(current_raw):.3f}, mean={np.nanmean(current_raw):.3f}")

        # [4] 加速度・ジャーク・速度を計算
        print(f"\n[4] 加速度・ジャーク・速度を計算中...")
        velocity, acceleration = calculate_velocity_from_current(current_raw, dt=dt_mean)
        jerk = calculate_jerk(acceleration, dt=dt_mean)

        print(f"✓ 速度: min={np.nanmin(velocity):.6f}, max={np.nanmax(velocity):.6f}, mean={np.nanmean(velocity):.6f}")
        print(f"✓ 加速度: min={np.nanmin(acceleration):.6f}, max={np.nanmax(acceleration):.6f}, mean={np.nanmean(acceleration):.6f}")
        print(f"✓ ジャーク: min={np.nanmin(jerk):.6f}, max={np.nanmax(jerk):.6f}, mean={np.nanmean(jerk):.6f}")

        # [5] 結果をCSVに出力
        print(f"\n[5] 結果をCSVに出力中...")
        results_df = pd.DataFrame({
            'time(s)': time_values,
            'current(Arms)': current_raw,
            'velocity(m/s)': velocity,
            'acceleration(m/s^2)': acceleration,
            'jerk(m/s^3)': jerk
        })

        output_file = RESULTS_DIR / "02_acceleration_analysis.csv"
        results_df.to_csv(output_file, index=False, encoding='utf-8')
        print(f"✓ 結果を保存: {output_file}")

        # [6] 統計情報をJSON出力
        stats = {
            "analysis_date": pd.Timestamp.now().isoformat(),
            "source_file": str(CSV_FILE),
            "target_column": target_column,
            "time_range": {
                "start_s": float(time_values[0]),
                "end_s": float(time_values[-1]),
                "duration_s": float(time_values[-1] - time_values[0])
            },
            "sampling": {
                "samples": int(len(df)),
                "interval_s": float(dt_mean),
                "frequency_hz": float(1 / dt_mean)
            },
            "current": {
                "min_Arms": float(np.nanmin(current_raw)),
                "max_Arms": float(np.nanmax(current_raw)),
                "mean_Arms": float(np.nanmean(current_raw)),
                "std_Arms": float(np.nanstd(current_raw))
            },
            "velocity": {
                "min_m/s": float(np.nanmin(velocity)),
                "max_m/s": float(np.nanmax(velocity)),
                "mean_m/s": float(np.nanmean(velocity)),
                "std_m/s": float(np.nanstd(velocity))
            },
            "acceleration": {
                "min_m/s^2": float(np.nanmin(acceleration)),
                "max_m/s^2": float(np.nanmax(acceleration)),
                "mean_m/s^2": float(np.nanmean(acceleration)),
                "std_m/s^2": float(np.nanstd(acceleration))
            },
            "jerk": {
                "min_m/s^3": float(np.nanmin(jerk)),
                "max_m/s^3": float(np.nanmax(jerk)),
                "mean_m/s^3": float(np.nanmean(jerk)),
                "std_m/s^3": float(np.nanstd(jerk))
            }
        }

        stats_file = RESULTS_DIR / "02_acceleration_analysis_stats.json"
        with open(stats_file, 'w', encoding='utf-8') as f:
            json.dump(stats, f, indent=2, ensure_ascii=False)
        print(f"✓ 統計情報を保存: {stats_file}")

    else:
        print("✗ 電流値の列が見つかりません")

    print("\n" + "=" * 70)
    print(" 解析完了")
    print("=" * 70)

if __name__ == "__main__":
    main()

