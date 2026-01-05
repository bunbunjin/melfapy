# -*- coding: utf-8 -*-
"""
log_data1.csvの詳細ヘッダー解析
"""

import csv
from pathlib import Path

csv_file = Path(__file__).parent.parent / "log" / "log_data1.csv"

# 詳細にヘッダー行を解析
with open(csv_file, 'r', encoding='utf-8-sig') as f:
    reader = csv.reader(f)

    for i, row in enumerate(reader):
        if i == 0:
            print(f"Row 0 (メタデータ): {len(row)} 項目")
            print(row[:5])
        elif i == 1:
            print(f"\nRow 1 (日本語ヘッダー): {len(row)} 項目")
            print(f"最初の5項目: {row[:5]}")
        elif i == 2:
            print(f"\nRow 2 (ユニット): {len(row)} 項目")
            print("最初の50列のユニット:")
            for j in range(0, min(50, len(row)), 5):
                print(f"  [{j:3d}-{j+4:3d}]: {row[j:j+5]}")
            print("\n全列のユニット（グループ化）:")
            # グループ化して表示
            current_unit = None
            unit_cols = []
            for j, unit in enumerate(row):
                if unit != current_unit:
                    if unit_cols:
                        print(f"  {current_unit}: {len(unit_cols)} 列 [{unit_cols[0]}-{unit_cols[-1]}]")
                    current_unit = unit
                    unit_cols = [j]
                else:
                    unit_cols.append(j)
            # 最後のグループ
            if unit_cols:
                print(f"  {current_unit}: {len(unit_cols)} 列 [{unit_cols[0]}-{unit_cols[-1]}]")
        elif i == 3:
            print(f"\nRow 3 (データ例): {len(row)} 項目")
            print(f"最初の10個のデータ: {row[:10]}")
            break

