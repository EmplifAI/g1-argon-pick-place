# G1カメラ・接続メモ

確認日: 2026-10-14

## 確認できた情報

- PCの有線インターフェース: enp12s0、192.168.123.99/24。
- G1制御ボード: 192.168.123.161。ping応答確認。
- G1 Jetson: 192.168.123.164。ping応答・SSH接続確認。
- Unitree SDK VideoClient.GetImageSample()からJPEGを取得。
- 実際の取得画像をデコードして確認したRGB解像度: 1920×1080。
- Jetsonのpyrealsense2で実機の機器情報・対応プロファイルを列挙。型番: Intel RealSense D435I。
- 実機が報告するRGB最大解像度: 1920×1080、対応FPS: 6/15/30。
- 実機が報告するデプス最大解像度: 1280×720、対応FPS: 6/15/30、形式Z16。
- 上記FPSは対応プロファイルの値で、現在の配信FPSやRGB/デプス同時取得の実測値ではない。
- Jetsonのpython3からpyrealsense2を利用可能。プロファイル列挙のみ実施し、新たなカメラストリームは開始していない。
- PCプレビュー: http://127.0.0.1:8765/
- 既存起動スクリプト: /home/soneo/xr_teleoperate/local/start-camera.sh
- 既存映像受信実装: /home/soneo/xr_teleoperate/local/g1-realsense-preview.py
- 保存画像: ../captures/g1-current.jpg
- Jetsonに/dev/video0〜/dev/video5あり。
- v4l2-ctlとrs-enumerate-devicesはPATH上で確認できず。

## 未確認の情報

- RGB/デプスの同時取得確認、実測FPS。
- デプス取得方法、解像度、単位、内部パラメータ、RGBへの位置合わせ。

## デプス使用方針

ボトル把持とBへの配置にデプスを使う。RGBでボトルとBを認識し、
位置合わせしたデプスとカメラ内部パラメータで3D座標を復元する。
Bはテーブル平面に投影し、把持点はボトルの姿勢・形状から選ぶ。
透明なPET表面の深度は欠損しうるので、周辺テーブル平面、複数フレーム、
ボトル寸法と組み合わせて推定する。信頼できる位置が得られないときは動作を開始しない。
現行JPEGプレビューにはデプスと内部パラメータの取得処理はない。別途受信を実装する。
