# CHANGELOG.md

## [Unreleased]

### Added
- ai_car_description パッケージを新規作成
  - CMakeLists.txt (ament_cmakeベース)、package.xml
  - urdf/ai_car.xacro: メカナム4輪・LiDAR・カメラ・IMU を含むロボット記述
  - launch/view_robot.launch.py: robot_state_publisher / joint_state_publisher（use_gui・use_rviz・use_sim_time 引数）
  - rviz/ai_car.rviz: RobotModel / TF / LaserScan 表示設定
  - ディレクトリ構造: config/, launch/, meshes/, rviz/, urdf/
- ai_car_web パッケージを新規作成（FastAPI Webダッシュボード）
  - dashboard_node: `/cmd_vel` publish、`/scan`・`/imu/data`・`/odom` 購読
  - REST API: `/api/status`, `/api/cmd_vel`, `/api/stop`、WebSocket `/ws`
  - static/index.html: メカナム操作UIとテレメトリ表示
  - config/dashboard.yaml、launch/dashboard.launch.py
  - 無指令タイムアウトによる自動停止
- プロジェクト管理ファイルを作成
  - PROJECT_RULES.md: 開発方針・禁止事項
  - PROJECT_STATUS.md: 実装状況・課題管理
  - CHANGELOG.md: 本ファイル
