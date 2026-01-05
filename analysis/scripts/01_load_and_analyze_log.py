"""
log_data1.csvの読み込みと基本解析スクリプト

このスクリプトは以下を実施します：
1. log_data1.csvを読み込み
2. データ構造を確認
3. 時刻、電流値などの基本統計情報を出力
4. 加速度・ジャーク・速度の計算に必要な列を特定
"""

import pandas as pd
import numpy as np
from pathlib import Path
import json

# パスを定義
CSV_FILE = Path(__file__).parent.parent / "log" / "log_data1.csv"
RESULTS_DIR = Path(__file__).parent.parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)

def load_csv():
    """CSVファイルを読み込む"""
    # ヘッダーは最初の2行をスキップ
    df = pd.read_csv(CSV_FILE, encoding='utf-8', skiprows=2)
    return df

def analyze_structure(df):
    """データ構造を分析"""
    info = {
        "total_rows": len(df),
        "total_columns": len(df.columns),
        "timestamp_column": df.columns[0],  # 最初の列は時刻
        "columns": df.columns.tolist()
    }
    return info

def identify_relevant_columns(df):
    """解析に関連する列を特定"""
    # 可能性のある列名パターンを検索
    relevant_cols = {
        "time": None,
        "current": [],  # 電流値
        "velocity": [],  # 速度
        "position": [],  # 位置
        "torque": []  # トルク
    }

    for col in df.columns:
        col_lower = col.lower()

        if "time" in col_lower or col_lower.strip() == "-" or col_lower.strip() == "":
            continue

        # 列名から推測して分類
        if "j" in col_lower or "joint" in col_lower or "arms" in str(df[col].dtype):
            if relevant_cols["time"] is None:
                relevant_cols["time"] = col
            elif "arms" in col_lower or "fb" in col_lower:
                relevant_cols["current"].append(col)

        if "speed" in col_lower or "velocity" in col_lower or "vel" in col_lower:
            relevant_cols["velocity"].append(col)

        if "pos" in col_lower or "position" in col_lower:
            relevant_cols["position"].append(col)

        if "torque" in col_lower or "trq" in col_lower:
            relevant_cols["torque"].append(col)

    return relevant_cols

def compute_statistics(df):
    """基本統計情報を計算"""
    # 数値列のみを取得
    numeric_df = df.select_dtypes(include=[np.number])

    stats = {
        "time_range": {
            "min": numeric_df.iloc[:, 0].min(),
            "max": numeric_df.iloc[:, 0].max(),
            "duration": numeric_df.iloc[:, 0].max() - numeric_df.iloc[:, 0].min()
        },
        "sampling": {
            "total_samples": len(df),
            "mean_interval": numeric_df.iloc[:, 0].diff().mean() if len(df) > 1 else 0
        }
    }

    return stats

def main():
    print("=" * 60)
    print("log_data1.csv 解析スクリプト")
    print("=" * 60)

    # CSVを読み込み
    print("\n[1] CSVファイルを読み込み中...")
    df = load_csv()
    print(f"✓ 読み込み完了: {len(df)} 行 × {len(df.columns)} 列")

    # データ構造を分析
    print("\n[2] データ構造を分析中...")
    info = analyze_structure(df)
    print(f"✓ 時刻列: {info['timestamp_column']}")
    print(f"✓ データ開始時刻: {df.iloc[0, 0]}")
    print(f"✓ データ終了時刻: {df.iloc[-1, 0]}")

    # 関連列を特定
    print("\n[3] 関連列を特定中...")
    relevant = identify_relevant_columns(df)
    print(f"✓ 時刻列: {relevant['time']}")
    print(f"✓ 電流値列数: {len(relevant['current'])}")
    if relevant['current']:
        print(f"  - {relevant['current'][:3]}...")

    # 統計情報を計算
    print("\n[4] 統計情報を計算中...")
    stats = compute_statistics(df)
    print(f"✓ 時刻範囲: {stats['time_range']['min']} - {stats['time_range']['max']} (継続時間: {stats['time_range']['duration']:.2f})")
    print(f"✓ サンプリング間隔: {stats['sampling']['mean_interval']:.2f} (ms)")

    # 最初と最後の数行を表示
    print("\n[5] データサンプル")
    print("\n--- 最初の3行 ---")
    print(df.iloc[:3, :5])
    print("\n--- 最後の3行 ---")
    print(df.iloc[-3:, :5])

    # 全列名を表示（参考）
    print("\n[6] 全列名（30列まで表示）")
    for i, col in enumerate(df.columns[:30]):
        print(f"  [{i:3d}] {col}")
    if len(df.columns) > 30:
        print(f"  ... ({len(df.columns) - 30} more columns)")

    # 結果をJSON形式で保存
    print("\n[7] 結果を保存中...")
    results = {
        "file": str(CSV_FILE),
        "metadata": info,
        "statistics": {
            "total_rows": int(len(df)),
            "total_columns": int(len(df.columns)),
            "time_range": {
                "min": float(stats['time_range']['min']),
                "max": float(stats['time_range']['max']),
                "duration_ms": float(stats['time_range']['duration'])
            },
            "sampling": {
                "total_samples": int(stats['sampling']['total_samples']),
                "mean_interval_ms": float(stats['sampling']['mean_interval'])
            }
        },
        "column_names": df.columns.tolist()
    }

    # JSON ファイルに保存
    results_file = RESULTS_DIR / "01_log_structure_analysis.json"
    with open(results_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"✓ 結果を保存: {results_file}")

    # CSVサンプルも出力
    sample_csv = RESULTS_DIR / "01_log_structure_sample.csv"
    df.head(20).to_csv(sample_csv, index=False, encoding='utf-8')
    print(f"✓ サンプルCSVを保存: {sample_csv}")

    print("\n" + "=" * 60)
    print("解析完了")
    print("=" * 60)

if __name__ == "__main__":
    main()

