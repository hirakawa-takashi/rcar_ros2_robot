# PROJECT_STATUS.md

## 最終更新
- 更新日: 2026/09/15
- 更新概要: TM1637 4桁7セグメントLED用 `seg_display_node`（`/display_state`）を追加。Logitech F710 ゲームパッドによる手動操作（`joy_teleop_node`）を追加。以前: ダッシュボードに Raspberry Pi 5 の GPIO 40 ピンヘッダー図（横向き、上段 2〜40 / 下段 1〜39）と 40 ピン一覧表（使用中・配線色・接続先・信号）を追加。配線は `config/gpio_pins.yaml` で編集、`GET /api/gpio` で取得。以前: ダッシュボードの systemd 自動起動サービスと respawn 対応を追記。カメラと LiDAR のカードを先頭に移動し横幅を2列分に拡大。LiDAR の取り付け向きを 180° 補正し、カメラ検出物体の距離推定をクラスタ中央値に変更。CPU カードに入力電圧・入力電流と低電圧・スロットリング警告を追加し、CPU / AI HAT+ カードの注記行を削除。CPU カードのロードアベレージを削除し PD 対応（5A）の 〇 / × 表示に変更。
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
  - LiDAR カード: `/scan` の360度スキャンを上面視の点群マップ（Canvas、最大720点に間引き、表示範囲は自動スケール）として描画。カード順は カメラ → LiDAR → CPU → AI HAT+ で、カメラと LiDAR は横幅 2 列分（狭い画面では 1 列）
  - LiDAR ノード: `rplidar_ros`（`rplidar_composition`）を `dashboard.launch.py` の `use_lidar` で起動。ポートは by-id パス、115200bps、`frame_id: laser`
  - カメラノード: `camera_ros`（libcamera）を `dashboard.launch.py` の `use_camera` で起動。`~/opt/rpicam` の Raspberry Pi 版 libcamera を `LD_LIBRARY_PATH` に自動追加
  - 障害物判定ノード（`perception_node`）: LiDAR を主として前方 ±30° を左/中央/右セクターで評価し、停止 0.35m / 減速 0.8m で 停止・減速・安全・不明 を判定。カメラ画像は AI HAT+（Hailo-8, `yolov8n.hef`）で推論して物体名を補助情報として付与。結果は `/obstacle_status`（JSON, 5Hz）
  - 検出物体の距離: 画像の横位置を `camera_hfov_deg`（66°）で方位角に変換し、LiDAR の同方位の距離を採用。カメラ映像に検出枠（`danger_distance` 0.3m 以内は赤、それ以外は緑）と距離を重畳し、LiDAR 点群マップでは前方 ±30°（`front_angle_deg` 60°）内かつ 0.3m 以内の点を赤点で表示。点群マップには前方／後方／左／右のラベルを表示（上＝前方）。方位角範囲内の点は距離でクラスタリングし（`cluster_gap` 0.25m）、最も手前のまとまりの中央値を採用する。スキャンが `scan_max_age`（1秒）より古い場合は距離を出さない。LiDAR の取り付け向きは `scan_angle_offset_deg` で補正し、実機は前方が機体後方を向くため 180° を設定済み
  - サーマル制御: CPU 70℃ / AI HAT+ 75℃ で推論レート 40%、CPU 78℃ / AI HAT+ 85℃ で推論停止（LiDAR 判定は継続）。`dashboard_node` は `speed_scale` を前進指令に適用する（`obstacle_guard`）
  - 画面（`static/index.html`）: 手動操作カードは LiDAR カードの直下（4 列固定グリッド、幅 2 列分）に配置。メカナム方向操作（前後・平行移動・旋回）、出力ゲイン、テレメトリ表示
  - 安全機構: `cmd_timeout`（既定0.7秒）無指令で自動停止
  - ゲームパッド手動操作（`joy_teleop_node`）: Logitech F710（XInput モード、`/dev/input/js0`）を Linux joystick API で直接読み（追加の Python 依存なし）、正規化済み Twist を `/joy_cmd` に 20Hz で publish。`dashboard_node` が `/joy_cmd` を購読して Web 操作と同じ `publish_cmd_vel()` 経路で `/cmd_vel` に変換するため、最大速度・障害物ガード（減速／停止）・`cmd_timeout` 停止が同様に適用される。割り当て: 左スティック上下=前後、左スティック左右=平行移動、右スティック左右=旋回、十字キー=前後・平行移動（デジタル）、RB=全速（通常は `speed_scale` 0.5）、B=停止（押している間）、`button_enable` でデッドマンボタンも設定可（既定は無効）。F710 未接続でも起動し、抜き差しを検出して再接続（切断時は指令を出さない）。画面の手動操作カードに接続状態（未接続／接続中／操作中）を表示。launch 引数 `use_joy` で無効化可
  - `seg_display_node`: TM1637 4桁7セグメントLEDを libgpiod で駆動し、`/system_status`・`/obstacle_status`・`/joy_cmd`・`/cmd_vel` から状態を判定して `/display_state`（std_msgs/String）へ publish。`use_seg_display` で無効化可
  - パラメータ: `config/dashboard.yaml`（host/port/各トピック名/最大速度/タイムアウト/配信レート）
  - 自動起動: `systemd/ai-car-dashboard.service`（`install_service.sh` で登録、`Restart=always`、USB デバイス待ちで起動を10秒遅延）。カメラ / LiDAR / 7セグ表示ノードは launch 側で `respawn`
  - `system_monitor_node`: Raspberry Pi 5 / AI HAT+ の状態監視（読み取りのみ）
    - Publish: `/system_status`（std_msgs/String, JSON、1Hz）
    - 取得内容: CPU使用率（全体・コア別）・クロック・温度・ロードアベレージ、メモリ/Swap/ディスク、PMIC レール別電圧・電流・電力、スロットリング状態、Hailo-8 検出状態
    - パラメータ: `publish_rate` / `topic` / `enable_pmic` / `enable_hailo`
    - 依存: `psutil`、`vcgencmd`、`lspci`、`hailortcli`（あれば利用）
  - GPIO 40 ピンヘッダーカード（全幅）: `ai_car_web/gpio_pinout.py` の固定ピン定義（物理番号・名称・BCM・種別）と `config/gpio_pins.yaml` の配線定義（`pins` / `device` / `signal` / `color` / `note`）を結合し `GET /api/gpio` で返す。画面は SVG のヘッダー図（種別ごとの色、使用中ピンは緑の外枠、配線色バー、ホバーで接続先表示）と一覧表（ピン / 名称 / BCM / 使用 / 配線色 / 接続先 / 信号・備考）。同一ピンへの複数接続（I2C バス共有）は「/」区切りで併記。パラメータ `gpio_config`（空なら share 内の `gpio_pins.yaml`）。YAML 編集後は `dashboard_node` 再起動で反映

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
- 実機の電源が不足している。ダッシュボード（カメラ＋LiDAR＋Hailo 推論）起動と同時に `hwmon3: Undervoltage detected!` が連発し、2026/09/14 19:22 に shutdown シーケンスなしで電源断した（前回起動だけで 198 回発生、EXT5V は 4.75、5.11V、`get_throttled=0x50000`）。`usb_max_current_enable=0` で 5A PD として認識されておらず、Pi 5 が制限モード（USB 周辺機器 合計 600mA）で動作している。公式 27W USB-C PD 電源への交換と、RPLIDAR の別系統（セルフパワーハブ等）給電が必要
- カメラは Ubuntu 標準の libcamera 0.7.2 だと raspi カーネル 6.8 のエンティティ名不一致で `no cameras available` となる。`~/opt/rpicam` の Raspberry Pi 版 libcamera を使う必要がある（手順は README 参照）
- `vcgencmd get_throttled` が `0x50000` = 過去に低電圧/スロットリングを検出（現在は正常）
- `gpio_pins.yaml` の配線内容（Motor HAT / BNO055 の I2C・電源・GND、AI HAT+ の ID EEPROM ピン、配線色）は HARDWARE_BOM からの暫定値。実配線と照合して修正が必要。TM1637 の配線色は実配線を反映済み（オレンジ・黒・黄・緑）
- WebSocket は `pip install --user --break-system-packages "websockets>=13"` で有効化済み（apt の python3-websockets 10.4 は uvicorn が要求する `ServerProtocol` を持たず、入れると dashboard_node が ImportError で起動しない）。未導入環境では `/api/status` の 250ms ポーリングへ自動フォールバックする

## テスト結果
- `seg_display_node`（実機、2026/09/15、**TM1637 モジュール未接続の状態**）: `python3-libgpiod` を apt 導入後、`gpiochip4 (pinctrl-rp1)` の GPIO23/24 を `seg_display_node` が output で確保（`gpioinfo` で確認）。起動 1 秒で `boot` → `/system_status` 受信後 `rdy` に遷移、`/display_state` を publish。実際の LED 点灯は配線後に確認が必要
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
- F710 手動操作: 実機で `joy_teleop_node` が `Logitech Gamepad F710 (/dev/input/js0)` を検出し `/joy_cmd` 20Hz を確認。`/joy_cmd {x:0.5, y:-0.5, z:0.5}` を publish → `/cmd_vel {x:0.075, y:-0.15, z:0.5}`（最大速度 0.3/1.0 と前方障害物による減速 0.5 が適用）を確認。入力停止 → 0.7 秒後に「指令タイムアウトのため停止しました」を確認。`/api/status` の `joy.connected / active` を確認。スティックの実操作による前後・左右の向きは未確認（ユーザーによる実機確認が必要）
- GPIO ヘッダー: `build_pinout('config/gpio_pins.yaml')` で使用中 9 ピン（1,2,3,4,5,6,9,27,28）、YAML 不在時は `error` を返すことを確認。実機で `GET /api/gpio` → `used_count: 9`、ブラウザでヘッダー図（上下段ラベルの重なり無し）と 40 行の一覧表の表示を確認。`flake8 --max-line-length 100` エラー無し
- `ros2 launch ai_car_description view_robot.launch.py`: 起動成功（`/robot_description`・`/joint_states`・`/tf` 発行を確認）

## カメラ仕様（実測）
- 解像度: 1280x720 / JPEG 品質 80（IMX708 / Camera Module v3、`config/dashboard.yaml` の `width`/`height`/`jpeg_quality`）。1 枚 約100KB・約2.9MB/s、camera_ros の CPU は約 36〜45%
- ROS 配信レート: 約 30Hz（`/camera/image_raw/compressed`）
- ダッシュボード MJPEG: 30fps（`camera_stream_rate`）
- AI HAT+ 推論: 640x640 にリサイズして 10Hz 設定（実測 約5Hz、JPEG デコード＋リサイズが律速。推論自体は 約9ms、高温時は自動低下）

## 次回作業
- モーター制御ノード（`/cmd_vel` 購読 → Motor HAT 駆動）を実装する
- IMU ノードを実装しダッシュボードのテレメトリを実データで確認する

## 変更ファイル
- `/home/super/AI-CAR_ws/src/ai_car_description/` - 新規作成（package.xml, CMakeLists.txt, urdf/, launch/, rviz/, config/, meshes/）
- `/home/super/AI-CAR_ws/src/ai_car_web/` - 新規作成（package.xml, setup.py, setup.cfg, ai_car_web/dashboard_node.py, launch/, config/, static/）
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/system_monitor_node.py` - 新規作成
- `/home/super/AI-CAR_ws/src/ai_car_web/{setup.py, package.xml, config/dashboard.yaml, launch/dashboard.launch.py, static/index.html, ai_car_web/dashboard_node.py}` - システム監視対応で更新
- `/home/super/AI-CAR_ws/systemd/{ai-car-dashboard.service, install_service.sh}` - 新規作成（自動起動）
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/gpio_pinout.py`、`config/gpio_pins.yaml` - 新規作成（GPIO ピン配置）
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/dashboard_node.py, config/dashboard.yaml, package.xml, static/index.html}` - GPIO カード対応で更新
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/joy_teleop_node.py` - 新規作成（F710 手動操作）
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/dashboard_node.py, setup.py, launch/dashboard.launch.py, config/dashboard.yaml, static/index.html}` - F710 対応で更新
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/seg_display_node.py` - 新規作成（TM1637 7セグ表示）
- `/home/super/AI-CAR_ws/src/ai_car_web/{setup.py, package.xml, config/dashboard.yaml, launch/dashboard.launch.py, config/gpio_pins.yaml}` - TM1637 7セグ表示対応で更新
- `/home/super/AI-CAR_ws/PROJECT_STATUS.md` - 本ファイル
- `/home/super/AI-CAR_ws/CHANGELOG.md` - 更新
