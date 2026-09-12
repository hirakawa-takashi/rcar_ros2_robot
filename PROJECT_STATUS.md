# PROJECT_STATUS.md

## 最終更新
- 更新日: 2026/09/13
- 更新概要: ai_car_description（URDF/Xacro・Launch・RViz設定）と ai_car_web（FastAPI Webダッシュボード）を新規作成。実機でビルドと動作確認を実施。
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
  - Subscribe: `/scan`、`/imu/data`、`/odom`
  - REST: `GET /api/status`、`POST /api/cmd_vel`、`POST /api/stop`、WebSocket `/ws`（テレメトリ配信）
  - 画面（`static/index.html`）: メカナム方向操作（前後・平行移動・旋回）、出力ゲイン、テレメトリ表示
  - 安全機構: `cmd_timeout`（既定0.7秒）無指令で自動停止
  - パラメータ: `config/dashboard.yaml`（host/port/各トピック名/最大速度/タイムアウト/配信レート）

## 現在作業中
- なし

## 未実装機能
- LiDARノード（RPLIDAR ドライバ）
- カメラノード
- IMUノード（BNO055）
- モーター制御ノード（Adafruit Motor HAT / メカナム逆運動学）
- オドメトリ発行
- SLAM
- Nav2自律走行

## 既知の問題
- `ros-jazzy-joint-state-publisher` が未インストールのため `view_robot.launch.py` が起動不可（`sudo apt install -y ros-jazzy-joint-state-publisher` が必要）
- Pi には rviz2 が未インストール（ros-base のみ）。RViz2 は開発PC側での表示を想定
- `/scan`・`/imu/data`・`/odom` の各ノードが未実装のため、ダッシュボードのテレメトリは現状 null

## テスト結果
- `colcon build --symlink-install`: 2パッケージ成功
- `xacro ai_car.xacro`: URDF 生成成功（10リンク）
- `ros2 launch ai_car_web dashboard.launch.py`: 起動成功
  - `GET /` → 200
  - `GET /api/status` → JSON 応答
  - `POST /api/cmd_vel {linear_x:1.0, angular_z:0.5}` → `{linear_x:0.3, angular_z:0.5}`（最大速度でスケール）
  - `/cmd_vel` トピック publish と指令タイムアウト停止のログを確認
- `ros2 launch ai_car_description view_robot.launch.py`: joint_state_publisher 未インストールで失敗（未検証）

## 次回作業
- `ros-jazzy-joint-state-publisher` を導入し `view_robot.launch.py` を検証する
- モーター制御ノード（`/cmd_vel` 購読 → Motor HAT 駆動）を実装する
- LiDAR / カメラ / IMU ノードを実装しダッシュボードのテレメトリを実データで確認する

## 変更ファイル
- `/home/super/AI-CAR_ws/src/ai_car_description/` - 新規作成（package.xml, CMakeLists.txt, urdf/, launch/, rviz/, config/, meshes/）
- `/home/super/AI-CAR_ws/src/ai_car_web/` - 新規作成（package.xml, setup.py, setup.cfg, ai_car_web/dashboard_node.py, launch/, config/, static/）
- `/home/super/AI-CAR_ws/PROJECT_STATUS.md` - 本ファイル
- `/home/super/AI-CAR_ws/CHANGELOG.md` - 更新
