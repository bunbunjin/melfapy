# melpy

## Overview
Mitsubishi MELFAをPythonで動作させるためのライブラリ。

1. ToolBox3で座標初期化
2. MelafaPoseで目的の座標を指定
3. MelfaPacketでコマンド、送信データタイプ指定、返信データタイプ指定、位置データを指定する
データ指定
4. send_packet()メソッドと使用して動き始める

> Melfa座標指定フォーマット
> 
> [x, y, z, a, b, c f1, f2]
