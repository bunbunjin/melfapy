# -*- coding: utf-8 -*-
"""
log_data1.csvの詳細解析

このスクリプトは、log_data1.csvのエンコーディングと構造を分析します。
"""

import csv
from pathlib import Path

csv_file = Path(__file__).parent.parent / "log" / "log_data1.csv"

# 複数のエンコーディングを試す
encodings = ['utf-8-sig', 'shift_jis', 'cp932', 'iso-8859-1', 'utf-8']

for encoding in encodings:
    try:
        with open(csv_file, 'r', encoding=encoding) as f:
            print(f"\n=== {encoding} デコード ===")
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                if i == 0:
                    print(f"Row 0 (メタデータ): {row[0][:100]}...")
                elif i == 1:
                    print(f"Row 1 (ヘッダー): {row[0][:50]}... [{len(row)} 列]")
                elif i == 2:
                    print(f"Row 2 (ユニット): 最初の20列 = {row[:20]}")
                    print(f"Total columns: {len(row)}")
                    break
            print(f"✓ {encoding} で正常に読み込めました")
            break

    except Exception as e:
        print(f"✗ {encoding} で失敗: {str(e)[:80]}")

