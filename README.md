# rcar_ros2_robot (AT-CAR)

Raspberry Pi 5 + ROS2 Jazzy で動作するメカナム4輪自律走行ロボット AT-CAR のワークスペース。

## パッケージ

| パッケージ | 内容 |
|-----------|------|
| `ai_car_description` | URDF/Xacro モデル、表示用 Launch、RViz2 設定 |
| `ai_car_web` | FastAPI による Web ダッシュボード（テレメトリ表示・手動操作・システム監視） |

## ビルド

```bash
source /opt/ros/jazzy/setup.bash
cd ~/AI-CAR_ws
colcon build --symlink-install
source install/setup.bash
```

## 実行

ロボットモデルの表示（RViz2 は開発PC側で起動する想定）:

```bash
ros2 launch ai_car_description view_robot.launch.py          # robot_state_publisher + joint_state_publisher
ros2 launch ai_car_description view_robot.launch.py use_rviz:=true use_gui:=true
```

Web ダッシュボード:

```bash
ros2 launch ai_car_web dashboard.launch.py
# システム監視ノードを止める場合: use_system_monitor:=false
# ブラウザで http://<ラズパイのIP>:8080
```

## インターフェース

`dashboard_node`

- Publish: `/cmd_vel` (`geometry_msgs/Twist`)、`~/status` (`std_msgs/String`)
- Subscribe: `/scan` (`sensor_msgs/LaserScan`)、`/imu/data` (`sensor_msgs/Imu`)、`/odom` (`nav_msgs/Odometry`)、`/system_status` (`std_msgs/String`)
- HTTP: `GET /`、`GET /api/status`、`POST /api/cmd_vel`、`POST /api/stop`、WebSocket `/ws`
- パラメータ: `src/ai_car_web/config/dashboard.yaml`

操作コマンドは -1.0〜1.0 の正規化値で受け取り、`max_linear_speed` / `max_angular_speed` にスケールされる。
`cmd_timeout`（既定 0.7 秒）の間に新しい指令が来ない場合は自動的に停止する。

`system_monitor_node`

- Publish: `/system_status` (`std_msgs/String`, JSON, 既定 1Hz)
- 内容: CPU 使用率（全体・コア別）・クロック・温度・ロードアベレージ、メモリ/Swap/ディスク、PMIC レール別の電圧・電流・電力と合計消費電力、スロットリング状態、AI HAT+ (Hailo-8) 検出状態
- パラメータ: `publish_rate` / `topic` / `enable_pmic` / `enable_hailo`
- 依存: `python3-psutil`、`vcgencmd`（Raspberry Pi）、`lspci`
- AI HAT+ の NPU 温度・デバイス情報は HailoRT（`hailortcli`）が導入されている場合のみ取得。未導入時は PCIe 検出状態のみ表示する。

## 依存

```bash
sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro
sudo apt install -y python3-psutil
pip3 install fastapi uvicorn
```

## ドキュメント

- 開発方針: `PROJECT_RULES.md`
- 実装状況: `PROJECT_STATUS.md`
- ハードウェア構成: `HARDWARE_BOM.md`
- 変更履歴: `CHANGELOG.md`
