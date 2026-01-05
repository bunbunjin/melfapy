# -*- coding: utf-8 -*-
"""
log_data1.csv - 全ジョイント（J1-J8）の加速度・ジャーク・速度 詳細解析

このスクリプトは以下を実施します：
1. 全8つのジョイントの電流値を抽出
2. 各ジョイントの加速度・ジャーク・速度を計算
3. 全ジョイント統合CSV、個別CSVを出力
4. 各ジョイントの統計情報を出力
"""

import pandas as pd
import numpy as np
from pathlib import Path
import json
import matplotlib.pyplot as plt

# パス設定
CSV_FILE = Path(__file__).parent.parent / "log" / "log_data1.csv"
RESULTS_DIR = Path(__file__).parent.parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)

def load_log_data():
    """log_data1.csvをロード"""
    df = pd.read_csv(CSV_FILE, encoding='utf-8-sig', skiprows=3)
    return df

def get_joint_columns():
    """ジョイント電流値の列を定義"""
    return {
        'J1': 3, 'J2': 4, 'J3': 5, 'J4': 6,
        'J5': 7, 'J6': 8, 'J7': 9, 'J8': 10
    }

def calculate_acceleration_jerk(current_values, dt=5.052):
    """
    電流値から加速度・ジャーク・速度を計算
    
    Args:
        current_values: 電流値配列
        dt: サンプリング時間間隔（秒）
    
    Returns:
        dict: acceleration, jerk, velocity, current_filtered
    """
    # NaN値を補間
    current_clean = pd.Series(current_values).interpolate(method='linear').ffill().bfill().values
    
    # 加速度を計算（1階差分）
    acceleration = np.gradient(current_clean, dt)
    
    # ジャークを計算（2階差分）
    jerk = np.gradient(acceleration, dt)
    
    # 速度を計算（積分）
    velocity = np.cumsum(acceleration) * dt
    
    return {
        'current': current_clean,
        'acceleration': acceleration,
        'jerk': jerk,
        'velocity': velocity
    }

def main():
    print("=" * 80)
    print(" log_data1.csv - 全ジョイント加速度・ジャーク・速度詳細解析")
    print("=" * 80)
    
    # [1] CSVを読み込み
    print("\n[1] ログファイルを読み込み中...")
    df = load_log_data()
    print(f"✓ データを読み込み: {len(df)} 行 × {len(df.columns)} 列")
    
    # 時刻列を取得
    time_col = df.columns[0]
    time_values = df[time_col].values / 1000.0  # ミリ秒から秒に変換
    dt_mean = np.mean(np.diff(time_values))
    print(f"✓ 時刻範囲: {time_values[0]:.3f} - {time_values[-1]:.3f} 秒")
    print(f"✓ サンプリング間隔: {dt_mean:.3f} 秒")
    
    # [2] ジョイント列を特定
    print("\n[2] ジョイント電流値を抽出中...")
    joint_cols = get_joint_columns()
    
    # 全ジョイント統合DataFrame
    all_joints_df = pd.DataFrame({'time(s)': time_values})
    
    # 各ジョイントの統計情報
    all_stats = {
        'analysis_date': pd.Timestamp.now().isoformat(),
        'source_file': str(CSV_FILE),
        'time_range': {
            'start_s': float(time_values[0]),
            'end_s': float(time_values[-1]),
            'duration_s': float(time_values[-1] - time_values[0])
        },
        'sampling': {
            'samples': int(len(df)),
            'interval_s': float(dt_mean),
            'frequency_hz': float(1 / dt_mean)
        },
        'joints': {}
    }
    
    # [3] 各ジョイントを解析
    print("\n[3] 各ジョイントを解析中...")
    for joint_name, col_idx in joint_cols.items():
        print(f"  {joint_name}: ", end='')
        
        # 列データを取得
        current_raw = df.iloc[:, col_idx].values.astype(float)
        
        # 計算実行
        results = calculate_acceleration_jerk(current_raw, dt=dt_mean)
        
        # 統計情報を計算
        joint_stats = {
            'column_index': col_idx,
            'current': {
                'min_Arms': float(np.nanmin(current_raw)),
                'max_Arms': float(np.nanmax(current_raw)),
                'mean_Arms': float(np.nanmean(current_raw)),
                'std_Arms': float(np.nanstd(current_raw))
            },
            'acceleration': {
                'min_m/s^2': float(np.nanmin(results['acceleration'])),
                'max_m/s^2': float(np.nanmax(results['acceleration'])),
                'mean_m/s^2': float(np.nanmean(results['acceleration'])),
                'std_m/s^2': float(np.nanstd(results['acceleration']))
            },
            'jerk': {
                'min_m/s^3': float(np.nanmin(results['jerk'])),
                'max_m/s^3': float(np.nanmax(results['jerk'])),
                'mean_m/s^3': float(np.nanmean(results['jerk'])),
                'std_m/s^3': float(np.nanstd(results['jerk']))
            },
            'velocity': {
                'min_m/s': float(np.nanmin(results['velocity'])),
                'max_m/s': float(np.nanmax(results['velocity'])),
                'mean_m/s': float(np.nanmean(results['velocity'])),
                'std_m/s': float(np.nanstd(results['velocity']))
            }
        }
        
        all_stats['joints'][joint_name] = joint_stats
        
        # 全ジョイント統合DataFrameに追加
        all_joints_df[f'{joint_name}_current(Arms)'] = results['current']
        all_joints_df[f'{joint_name}_accel(m/s^2)'] = results['acceleration']
        all_joints_df[f'{joint_name}_jerk(m/s^3)'] = results['jerk']
        all_joints_df[f'{joint_name}_velocity(m/s)'] = results['velocity']
        
        # 個別CSVを出力
        joint_df = pd.DataFrame({
            'time(s)': time_values,
            f'{joint_name}_current(Arms)': results['current'],
            f'{joint_name}_acceleration(m/s^2)': results['acceleration'],
            f'{joint_name}_jerk(m/s^3)': results['jerk'],
            f'{joint_name}_velocity(m/s)': results['velocity']
        })
        
        joint_file = RESULTS_DIR / f"03_joint_{joint_name}_profile.csv"
        joint_df.to_csv(joint_file, index=False, encoding='utf-8')
        
        print(f"✓ (I:{joint_stats['current']['mean_Arms']:+.3f}A, "
              f"A:{joint_stats['acceleration']['max_m/s^2']:.3f}m/s², "
              f"J:{joint_stats['jerk']['max_m/s^3']:.6f}m/s³)")
    
    # [4] 全ジョイント統合CSVを出力
    print("\n[4] 全ジョイント統合CSVを出力中...")
    all_joints_file = RESULTS_DIR / "03_all_joints_profile.csv"
    all_joints_df.to_csv(all_joints_file, index=False, encoding='utf-8')
    print(f"✓ 全ジョイント統合CSV: {all_joints_file}")
    
    # [5] 統計情報をJSON出力
    print("\n[5] 統計情報をJSON出力中...")
    stats_file = RESULTS_DIR / "03_all_joints_analysis_stats.json"
    with open(stats_file, 'w', encoding='utf-8') as f:
        json.dump(all_stats, f, indent=2, ensure_ascii=False)
    print(f"✓ 統計情報JSON: {stats_file}")
    
    # [6] サマリーレポート
    print("\n[6] サマリーレポート")
    print("\n  加速度（最大値）:")
    for joint, stats in sorted(all_stats['joints'].items()):
        max_accel = stats['acceleration']['max_m/s^2']
        print(f"    {joint}: {max_accel:+.6f} m/s²")
    
    print("\n  ジャーク（最大値）:")
    for joint, stats in sorted(all_stats['joints'].items()):
        max_jerk = stats['jerk']['max_m/s^3']
        print(f"    {joint}: {max_jerk:+.6f} m/s³")
    
    print("\n" + "=" * 80)
    print(" 解析完了")
    print("=" * 80)

if __name__ == "__main__":
    main()

