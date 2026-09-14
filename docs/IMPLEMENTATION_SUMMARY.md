# AI-CAR ロボット開発プロジェクト - 実装概要

## プロジェクト概要
- **プロジェクト名**: AI-CAR ロボット開発プロジェクト
- **実行環境**: Raspberry Pi 5
- **ROSバージョン**: ROS2 Jazzy（ros-base）
- **主な技術スタック**: LiDAR、カメラ、IMU、Nav2（未導入）、FastAPI

## 実装済み機能
- `ai_car_description`
  - URDF/Xacro（`urdf/ai_car.xacro`）: base_footprint、base_link、メカナム4輪、laser、camera_link、camera_optical_frame、imu_link
  - Launch（`launch/view_robot.launch.py`）: robot_state_publisher、joint_state_publisher、`use_gui` / `use_rviz` / `use_sim_time`
  - RViz2設定（`rviz/ai_car.rviz`）: RobotModel / TF / LaserScan
- `ai_car_web`
  - `dashboard_node`: FastAPI（別スレッド）+ rclpy
  - Publish `/cmd_vel`、Subscribe `/scan`・`/imu/data`・`/odom`
  - REST `/api/status`・`/api/cmd_vel`・`/api/stop`、WebSocket `/ws`
  - 手動操作UI（メカナム前後・平行移動・旋回、出力ゲイン）とテレメトリ表示
  - 無指令タイムアウト（既定0.7秒）による自動停止

## 未実装機能
- LiDARノード（RPLIDAR A1M8 ドライバ）
- カメラノード
- IMUノード（BNO055）
- モーター制御ノード（Adafruit Motor HAT、メカナム逆運動学）
- オドメトリ発行
- SLAM、Nav2自律走行

## ハードウェア構成
- Raspberry Pi 5 (8GB RAM)
- Raspberry Pi AI HAT+ (26TOPS NPU)
- Adafruit DC & Stepper Motor HAT
- Raspberry Pi カメラモジュール v3
- LiDAR（RPLIDAR A1M8）
- 9軸 IMU（GY-BNO055）
- メカナムホイール駆動系
- バッテリー（5V/12V系統）

## 次回作業計画
1. モーター制御ノードを実装し `/cmd_vel` で実走行させる
2. LiDAR / カメラ / IMU ノードを実装する
3. オドメトリと TF を整備する
4. SLAM と Nav2 を導入する
5. ダッシュボードにカメラ映像と地図表示を追加する

## プロジェクト管理
- プロジェクトルール: PROJECT_RULES.md
- 実装状況管理: PROJECT_STATUS.md
- 変更履歴: CHANGELOG.md
