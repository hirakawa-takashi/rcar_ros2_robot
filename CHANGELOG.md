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
- system_monitor_node を新規作成（Raspberry Pi 5 / AI HAT+ 監視）
  - CPU使用率（全体・コア別）・クロック・温度・ロードアベレージ
  - メモリ・Swap・ディスク使用量
  - `vcgencmd pmic_read_adc` による電源レール別の電圧・電流・電力と合計消費電力
  - `vcgencmd get_throttled` のスロットリング状態（現在・過去）
  - AI HAT+ (Hailo-8) の PCIe 検出状態、`hailo_pci` ドライバ版数と PCIe リンク速度・幅、HailoRT 導入済みなら FW 版数・アーキテクチャ
  - `/system_status`（std_msgs/String, JSON）を 1Hz で publish
  - dashboard_node が `/system_status` を購読し、REST / WebSocket の `system` フィールドで配信
  - index.html に CPU / メモリ / 電源 / AI HAT+ のカードを追加
- プロジェクト管理ファイルを作成
  - PROJECT_RULES.md: 開発方針・禁止事項
  - PROJECT_STATUS.md: 実装状況・課題管理
  - CHANGELOG.md: 本ファイル
- README に Ubuntu 24.04 向け AI HAT+ セットアップ手順（hailort-drivers v4.24.0 と HailoRT 4.24.0 のソース導入）を追記

### Fixed
- Hailo の状態判定を `hailortcli` の有無ではなく `/dev/hailo0` の存在で行うように変更。AI HAT+ 非対応の温度取得呼び出しを削除
- WebSocket が利用できない環境（uvicorn に websockets/wsproto 未導入）では `/api/status` の1秒ポーリングへ自動フォールバックするようにした
