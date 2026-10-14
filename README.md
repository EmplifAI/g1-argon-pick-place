# G1 Argon Pick & Place

目的: Gemini ArgonでペットボトルとテーブルのBを認識し、G1の左腕・左Inspire Handで把持・運搬・配置する。

実機は吊り具で支持し、動作対象は左腕・左RH56DFQだけ。
Argonが操作を判断し、G1側が関節補間・停止・通信断監視を担当する。
実機で左腕の小さい移動と指の開放を実施した。ボトルの把持・Bへの配置は未成功。

## 操作API

`observe`、`open_hand`、`move_left_hand`、`grasp_bottle`、`verify_grasp`、
`place_at`、`release_at_B`、`stop`。
手先の位置と姿勢から左腕7関節のIKを計算し、一つの操作呼び出しで連続軌道を実行する。
7関節の軌道はG1側で更新し、モデルの応答待ちで制御周期を止めない。

Cartesian操作は、実機のカメラ・手先座標校正が検証済みで、RGB-Dシーンが新鮮な場合だけ許可する。
未検証のURDFカメラ位置をそのまま実機動作に使わない。
再起動のboot ID、停止の世代番号、期限、速度、関節限界を確認する。
コードの実装と、実機での把持成功は別の状態として扱う。

## 環境

PCにはPython、NumPy、SciPy、Pinocchio、Unitree SDK2 PythonとCyclone DDSを用意。
既存の`xr-teleop`環境を利用できる。G1側にはSDK2 Python、NumPy、RealSense SDKを用意。
URDFとメッシュは`scripts/fetch_robot_description.sh`で公式リポジトリから固定コミットを取得する。
ソースにAPIキーを埋め込まない。`scripts/start_controller.py`はキーを匿名パイプでメモリーだけに渡す。

## 検証

SDK/Pinocchio環境で `python -m unittest discover -s tests -v`。
停止・通信断・古い指令・左右の範囲・速度・7関節補間・IK・未校正拒否を検証する。
実機投入は停止状態で行い、開始ボタンから明示的に起動する。

動画、操作ログ、センサー画像、ランタイムのPIDや停止状態、認証情報はGitに含めない。

カメラと接続の確認情報はdocs/camera.md、実機制約はAGENTS.mdに記録。

残る実機検証: RGB-Dの対応と手先校正 → 経路プレビュー → 左腕の接近 → 把持/持ち上げ/配置の確認。

既存参考: /home/soneo/xr_teleoperate/teleop/robot_control/robot_arm.py と robot_hand_inspire.py。
Inspire公式参考: https://github.com/unitreerobotics/dfx_inspire_service
標準Inspire DDSはrt/inspire/cmdとrt/inspire/state、右手0〜5・左手6〜11。q以外のコマンドフィールドは予約扱い。
右側を実際に操作しない保証は、サービス側の部分更新対応を確認するまで未解決。
Argon接続とモデル利用権限は未確認。認証情報は保存しない。
