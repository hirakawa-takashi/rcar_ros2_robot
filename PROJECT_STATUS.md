# PROJECT_STATUS.md

## 最終更新
- 更新日: 2026/09/13
- 更新概要: ai_car_web に system_monitor_node（Raspberry Pi 5 のCPU・メモリ・電源、AI HAT+ 情報）を追加し、ダッシュボードで表示。実機で動作確認済み。
- 更新担当: Devin

## システム構成
- 実行環境: Raspberry Pi 5
- ROS: ROS2 Jazzy（ros-base）
- ナビゲーション: Nav2（未導入）
- センサー: LiDAR（RPLIDAR A1M8）、カメラ（v3）、IMU（BNO055）
- Web: FastAPI + uvicorn
- 開発環境: VS Code Remote-SSH
- AI: Cline + LM Studio + DeepSeek 32B Q4_K_M

## 実装済み機能
- ai_car_description
  - URDF/Xacro（`urdf/ai_car.xacro`）: base_footprint / base_link / メカナム4輪 / laser / camera_link / camera_optical_frame / imu_link（計10リンク）
  - Launch（`launch/view_robot.launch.py`）: robot_state_publisher、joint_state_publisher、use_gui / use_rviz / use_sim_time 引数
  - RViz2設定（`rviz/ai_car.rviz`）: RobotModel / TF / LaserScan
- ai_car_web
  - `dashboard_node`: FastAPI サーバー（別スレッド）+ rclpy ノード
  - Publish: `/cmd_vel`（geometry_msgs/Twist）、`~/status`
  - Subscribe: `/scan`、`/imu/data`、`/odom`、`/camera/image_raw/compressed`
  - REST: `GET /api/status`、`POST /api/cmd_vel`、`POST /api/stop`、`GET /api/camera/snapshot`、`GET /api/camera/stream`（MJPEG）、WebSocket `/ws`（テレメトリ配信）
  - カメラカード: MJPEG 映像と受信フレーム数・最終受信時刻を表示（未受信時は待機表示）
  - LiDAR カード: `/scan` の360度スキャンを上面視の点群マップ（Canvas、最大720点に間引き、表示範囲は自動スケール）として描画。カード順は CPU → AI HAT+ → カメラ → LiDAR
  - LiDAR ノード: `rplidar_ros`（`rplidar_composition`）を `dashboard.launch.py` の `use_lidar` で起動。ポートは by-id パス、115200bps、`frame_id: laser`
  - カメラノード: `camera_ros`（libcamera）を `dashboard.launch.py` の `use_camera` で起動。`~/opt/rpicam` の Raspberry Pi 版 libcamera を `LD_LIBRARY_PATH` に自動追加
  - 障害物判定ノード（`perception_node`）: LiDAR を主として前方 ±30° を左/中央/右セクターで評価し、停止 0.35m / 減速 0.8m で 停止・減速・安全・不明 を判定。カメラ画像は AI HAT+（Hailo-8, `yolov8n.hef`）で推論して物体名を補助情報として付与。結果は `/obstacle_status`（JSON, 5Hz）
  - 検出物体の距離: 画像の横位置を `camera_hfov_deg`（66°）で方位角に変換し、LiDAR の同方位の最近距離を採用。カメラ映像に検出枠（`danger_distance` 0.3m 以内は赤、それ以外は緑）と距離を重畳し、LiDAR 点群マップでは前方 ±30°（`front_angle_deg` 60°）内かつ 0.3m 以内の点を赤点で表示。LiDAR の取り付け向きは `scan_angle_offset_deg`（既定180°）で補正し、マップ上方＝ロボット前方
  - サーマル制御: CPU 70℃ / AI HAT+ 75℃ で推論レート 40%、CPU 78℃ / AI HAT+ 85℃ で推論停止（LiDAR 判定は継続）。`dashboard_node` は `speed_scale` を前進指令に適用する（`obstacle_guard`）
  - 画面（`static/index.html`）: メカナム方向操作（前後・平行移動・旋回）、出力ゲイン、テレメトリ表示
  - 安全機構: `cmd_timeout`（既定0.7秒）無指令で自動停止
  - パラメータ: `config/dashboard.yaml`（host/port/各トピック名/最大速度/タイムアウト/配信レート）
  - `system_monitor_node`: Raspberry Pi 5 / AI HAT+ の状態監視（読み取りのみ）
    - Publish: `/system_status`（std_msgs/String, JSON、1Hz）
    - 取得内容: CPU使用率（全体・コア別）・クロック・温度・ロードアベレージ、メモリ/Swap/ディスク、PMIC レール別電圧・電流・電力、スロットリング状態、Hailo-8 検出状態
    - パラメータ: `publish_rate` / `topic` / `enable_pmic` / `enable_hailo`
    - 依存: `psutil`、`vcgencmd`、`lspci`、`hailortcli`（あれば利用）

## 現在作業中
- なし

## 未実装機能
- IMUノード（BNO055）
- モーター制御ノード（Adafruit Motor HAT / メカナム逆運動学）
- オドメトリ発行
- SLAM
- Nav2自律走行

## 既知の問題
- Pi には rviz2 が未インストール（ros-base のみ）。RViz2 は開発PC側での表示を想定
- `/imu/data`・`/odom` の各ノードが未実装のため、それらのテレメトリは現状 null
- LiDAR は `ros-jazzy-rplidar-ros` の導入と `super` の `dialout` グループ所属が必要
- AI HAT+ は `hailo_pci` 4.24.0 と HailoRT 4.24.0 をソース導入済み（Ubuntu 24.04 に `hailo-all` は存在しない。手順は README 参照）。温度は HailoRT C API `hailo_get_chip_temperature()` で取得済み。NPU 使用率と電力はダッシュボードでは扱わない（使用率は `HAILO_MONITOR=1` の推論アプリがある間のみ `hailortcli monitor` で参照可）。電力は未取得（`measure-power` は `UNSUPPORTED_OPCODE`、`query_health_stats()`/`query_performance_stats()` は HAILO8 非対応、`hatctl` は Ubuntu に存在せず Hailo 専用 hwmon も無し、PMIC に HAT 専用レール無し。外付け INA219 等が必要）
- 検出物体の距離は LiDAR の方位角対応による概算で、カメラとLiDARの外部キャリブレーションは未実施（設置が同軸・同方向である前提）
- `perception_node` の推論には HailoRT の Python バインディング（`hailo_platform`）が必要。apt には無いため HailoRT 4.24.0 のソースからビルドして導入する（手順は README 参照）。未導入の場合は LiDAR 判定のみで動作し、`/obstacle_status` の `note` に理由を表示する
- カメラは Ubuntu 標準の libcamera 0.7.2 だと raspi カーネル 6.8 のエンティティ名不一致で `no cameras available` となる。`~/opt/rpicam` の Raspberry Pi 版 libcamera を使う必要がある（手順は README 参照）
- `vcgencmd get_throttled` が `0x50000` = 過去に低電圧/スロットリングを検出（現在は正常）
- Pi に websockets/wsproto が未導入のため WebSocket が使えず、画面は `/api/status` の1秒ポーリングで動作中（`sudo apt install -y python3-websockets` で WebSocket 配信に戻る）

## テスト結果
- `colcon build --symlink-install`: 2パッケージ成功
- `xacro ai_car.xacro`: URDF 生成成功（10リンク）
- `ros2 launch ai_car_web dashboard.launch.py`: 起動成功
  - `GET /` → 200
  - `GET /api/status` → JSON 応答
  - `POST /api/cmd_vel {linear_x:1.0, angular_z:0.5}` → `{linear_x:0.3, angular_z:0.5}`（最大速度でスケール）
- `system_monitor_node`: `/system_status` の JSON 取得成功（CPU 48.3℃ / 合計 2.17W / EXT5V 4.84V / Hailo-8 PCIe 検出）
- HailoRT 導入後の `/api/status`: `driver_ready: true` / `hailo_pci 4.24.0` / FW 4.24.0 / HAILO8 / PCIe 8.0 GT/s x1 を取得
- AI HAT+ 温度: `/api/status` の `system.hailo.temperature_c` で 49.6℃ を取得（ts0/ts1 平均、取得時間 約30ms）
- `GET /api/status` に `system` フィールドが含まれることを確認
- ブラウザ表示確認: CPU / メモリ / 電源 / AI HAT+ の各カードが実値で更新されることを確認（ポーリングフォールバック経由）
  - `/cmd_vel` トピック publish と指令タイムアウト停止のログを確認
- カメラ（IMX708）: Raspberry Pi 版 libcamera v0.7.2+rpt20260817 / libpisp v1.7.0 を `~/opt/rpicam` にビルドし、`cam -l` でカメラ認識を確認
- `ros2 launch ai_car_web dashboard.launch.py`（カメラ含む）: `/camera/image_raw/compressed` 30Hz、`/api/status` の `camera.available: true`、`GET /api/camera/snapshot` → 200（約74KB JPEG）を確認
- LiDAR（RPLIDAR, CP2102 USB）: `rplidar_composition` 起動で `/scan` を 約8Hz で受信。ダッシュボードの点群マップに720点（正面 0.17m / 最大 3.05m）が描画されることをブラウザで確認
- 障害物判定: `/obstacle_status` で `level: stop`（前方 0.177m）を確認。Hailo-8 推論は約 8ms、YOLOv8n で物体検出（例: bed 0.61 / sink 0.50）を確認。温度閾値を一時的に下げて warn（推論 40%）→ critical（推論停止、LiDAR 判定は継続）→ 復帰を確認
- `ros2 launch ai_car_description view_robot.launch.py`: 起動成功（`/robot_description`・`/joint_states`・`/tf` 発行を確認）

## カメラ仕様（実測）
- 解像度: 640x480（IMX708 / Camera Module v3、`config/dashboard.yaml` の `width`/`height`）
- ROS 配信レート: 約 25〜30Hz（`/camera/image_raw/compressed`）
- ダッシュボード MJPEG: 10fps（`camera_stream_rate`）
- AI HAT+ 推論: 640x640 にリサイズして 4Hz（`inference_rate`、高温時は自動低下）

## 次回作業
- モーター制御ノード（`/cmd_vel` 購読 → Motor HAT 駆動）を実装する
- IMU ノードを実装しダッシュボードのテレメトリを実データで確認する

## 変更ファイル
- `/home/super/AI-CAR_ws/src/ai_car_description/` - 新規作成（package.xml, CMakeLists.txt, urdf/, launch/, rviz/, config/, meshes/）
- `/home/super/AI-CAR_ws/src/ai_car_web/` - 新規作成（package.xml, setup.py, setup.cfg, ai_car_web/dashboard_node.py, launch/, config/, static/）
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/system_monitor_node.py` - 新規作成
- `/home/super/AI-CAR_ws/src/ai_car_web/{setup.py, package.xml, config/dashboard.yaml, launch/dashboard.launch.py, static/index.html, ai_car_web/dashboard_node.py}` - システム監視対応で更新
- `/home/super/AI-CAR_ws/PROJECT_STATUS.md` - 本ファイル
- `/home/super/AI-CAR_ws/CHANGELOG.md` - 更新
