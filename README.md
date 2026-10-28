# rcar_ros2_robot (AI-CAR)

Raspberry Pi 5 + ROS2 Jazzy で動作するメカナム4輪自律走行ロボット AI-CAR のワークスペース。

- バージョン: 1.30.7（2026-10-28。変更は `CHANGELOG.md`、上げ方は `AGENTS.md`）

## パッケージ

| パッケージ | 内容 |
|-----------|------|
| `ai_car_description` | URDF/Xacro モデル、表示用 Launch、RViz2 設定 |
| `ai_car_web` | ROS 2 ノード一式（モーター・エンコーダー・LiDAR の見張り・段差・IMU・運転モード・自律走行・知覚）と FastAPI の Web ダッシュボード |

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

起動時の自動起動（systemd）:

```bash
sudo ~/AI-CAR_ws/systemd/install_service.sh   # /etc/systemd/system/ai-car-dashboard.service を登録して起動
systemctl status ai-car-dashboard.service
sudo systemctl disable --now ai-car-dashboard.service   # 手動起動に戻す場合
```

サービスは `Restart=always`（5秒間隔）で、USB デバイスの認識を待つため起動を10秒遅延する。カメラ、LiDAR、IMU のノードは launch 側で `respawn` する。

CPU ファンの作動温度（既定 50/60/67.5/75℃）を前倒しして高温になりにくくする:

```bash
sudo cp ~/AI-CAR_ws/systemd/ai-car-fan.service /etc/systemd/system/
sudo systemctl enable --now ai-car-fan.service   # 45/50/55/62℃ を設定（起動ごとに再適用）
# 温度を変えたい場合は Environment=FAN_TRIP_TEMPS="..." をサービスに追加
```

設定値は `scripts/set_fan_curve.sh` が `thermal_zone0` の active トリップへ書き込む。ファン回転数と段階はダッシュボードの CPU カードに表示される。

## インターフェース

`dashboard_node`

- Publish: `/cmd_vel_manual` (`geometry_msgs/Twist`、Web・ゲームパッド・スタックちゃんの手動指令。`/cmd_vel` にするのは `drive_mode_node` だけ)、`/drive_output` (`std_msgs/Float32`、「出力」0.1〜1.0)、`/drive_mode_request`、`/nav_goal`（地図の行き先）、`~/status` (`std_msgs/String`)
- Subscribe: `/scan`、`/imu/data`、`/imu/mag`、`/odom`、`/map`、`/pose`、`/system_status`、`/camera/image_raw/compressed`、`/obstacle_status`、`/cliff_status`、`/drive_mode`、`/autonomy_status`、`/motor_status`、`/joy_cmd`
- HTTP: `GET /`、`GET /api/status`、`GET /api/architecture`、`GET /api/program?file=…`（プログラム一覧のファイルの中身）、`GET /api/dev_diary?days=7&before=…&tag=…`（開発日記。DB から 7 日ずつ）、`POST /api/cmd_vel`、`POST /api/stop`、`POST /api/drive_output`、`POST /api/drive_mode`、`POST /api/slam/save`（名前を付けて地図の絵と slam_toolbox の地図データを保存）、`GET /api/slam/maps`・`POST /api/slam/maps/load`（保存した地図の一覧・選んで読みこむ）、`POST /api/nav`（地図の行き先 plan / go / cancel）、`GET /api/camera/snapshot`、`GET /api/camera/stream`（MJPEG）、WebSocket `/ws` など（全部はプロジェクト説明の「Web API」タブ）
- パラメータ: `src/ai_car_web/config/dashboard.yaml`

操作コマンドは -1.0〜1.0 の正規化値で受け取り、`max_linear_speed` / `max_angular_speed` にスケールされる。
`cmd_timeout`（既定 0.7 秒）の間に新しい指令が来ない場合は自動的に停止する。
ダッシュボードの「出力」は初め 15 %（`dashboard_node.output_percent: 15.0`・`drive_mode_node.default_output: 0.15`）で、手動・ゲームパッド・自動の最大の速さにかける。
画面上部の「プロジェクト説明」ボタンから、`architecture.yaml` に基づく概要・システム構成・ハードウェア構成・3D プリント・ソフトウェア構成・運転モードと安全機構・知覚と自律走行・Web API・開発状況の 9 タブを確認できる。
`dashboard_node.api_token` または環境変数 `AI_CAR_API_TOKEN` を設定すると、POST `/api/*` に
`X-API-Token`（または Bearer）ヘッダーが必要になる。UI は `http://<PI_IP>:8080/?token=...` を
一度開くとトークンを localStorage に保存する。

ほかのノード（`dashboard.launch.py` で起動。`use_*` で個別に止められる）

| ノード | 役割 |
|--------|------|
| `drive_mode_node` | `/cmd_vel_manual` と `/cmd_vel_auto` の片方を `/cmd_vel` にする（manual / auto / stop）。最後に LiDAR 360° の見張り（車体の端から 0.25 m で、その向きの動き・回転だけを 0。手動は 0.7 m から半分の速さ）と段差ガードをかける |
| `motor_driver_node` | `/cmd_vel` → メカナムの逆運動学 → Motor HAT M1〜M4。旋回 0 のあいだ IMU の向きを保つ（`heading_hold`）。`/motor_status` |
| `encoder_odom_node` | 4 輪のエンコーダー（GPIO v2 uAPI、距離）と IMU（`/imu/data`、回った向き）→ `/odom`・TF `odom → base_footprint`（20 Hz） |
| `cliff_node` | 前後の VL53L1X（0x30 / 0x31）→ `/cliff_status`（20 Hz） |
| `imu_node` | BNO055（0x29）→ `/imu/data`（20 Hz）・`/imu/mag` |
| `perception_node` | `/scan` → `/obstacle_status`。カメラ画像の AI（YOLOv8m・Hailo-8、10 回/秒）で物の名前を出す（表示だけ。速さには使わない） |
| `autonomy_node` | Level 1 自律走行（LiDAR の反応で前進・旋回）と Level 2（`/nav_goal` の行き先へ Nav2 で向かい、`/cmd_vel_nav` を中継）→ `/cmd_vel_auto` |
| Nav2（`nav2.launch.py`） | `controller_server`（Regulated Pure Pursuit）・`planner_server`（NavFn）・`behavior_server`・`bt_navigator`。slam_toolbox の `/map` と map → odom の TF を使い、速度は `/cmd_vel_nav`。`use_nav2`（既定 true、`use_slam:=true` のとき 40 秒後に起動） |
| `joy_teleop_node` | Logitech F710 → `/joy_cmd` |
| `lidar_watchdog_node` | `/scan` が止まったら rplidar を起動し直し、だめなら USB の電気を入れ直す |
| `system_monitor_node` | 下を参照 |
| SLAM（`slam_toolbox`） | `/scan` + `/odom` → `/map`・`/pose`（`use_slam`、既定 true） |

`system_monitor_node`

- Publish: `/system_status` (`std_msgs/String`, JSON, 既定 1Hz)
- 内容: CPU 使用率（全体・コア別）・クロック・温度・ロードアベレージ、メモリ/Swap/ディスク、PMIC レール別の電圧・電流・電力と合計消費電力、スロットリング状態、AI HAT+ (Hailo-8) 検出状態
- パラメータ: `publish_rate` / `topic` / `enable_pmic` / `enable_hailo`
- 依存: `python3-psutil`、`vcgencmd`（Raspberry Pi）、`lspci`
- AI HAT+ の情報は `/dev/hailo0`（`hailo_pci` ドライバ）と `hailortcli` の有無に応じて段階的に表示する。ドライバ未導入時は PCIe 検出状態のみ。
- AI HAT+ の温度は HailoRT の C API `hailo_get_chip_temperature()` を ctypes で直接呼び出して取得する（ts0/ts1 の平均。`hailortcli` 4.24 に温度サブコマンドは無い）。
- NPU 使用率と消費電力は取得・表示しない（使用率は推論アプリを `HAILO_MONITOR=1` で起動している間だけ `hailortcli monitor` で参照できる）。
- AI HAT+ の電力は取得できない。`hailortcli measure-power` は `HAILO_UNSUPPORTED_OPCODE`（ボードに電流監視 DVM 非搭載）、`query_health_stats()` / `query_performance_stats()` は HAILO8 アーキテクチャ非対応。`hatctl` は Raspberry Pi OS / 他ベンダ向けで Ubuntu には存在せず、Hailo 専用の hwmon デバイスも無い。PMIC にも HAT 専用レールはなく（AI HAT+ は PCIe コネクタの 5V から給電）、単体の消費電力を測るには INA219/INA3221 などの外付け I2C 電流センサを配線に挿入する必要がある。
- ダッシュボードの AI HAT+ カードは温度（横バーグラフ）・状態・デバイス情報（アーキテクチャ / FW / ドライバ / PCIe アドレス）を表示する（NPU 使用率と消費電力は表示しない）。

## TM1637 4桁7セグメントLED のセットアップ（旧表示器）

`seg_display_node` は TM1637 モジュールを bit-bang 駆動し、`/display_state`
（`std_msgs/String`）へ現在の状態コードを publish する。

| TM1637 | Raspberry Pi 5 |
|--------|----------------|
| VCC | 17（3V3） |
| GND | 20（GND） |
| CLK | 16（GPIO23） |
| DIO | 18（GPIO24） |

```bash
sudo apt install python3-libgpiod
sudo usermod -aG dialout super
# 再ログイン、またはサービスを再起動
```

GPIO チップは `pinctrl-rp1` のラベルから自動検出する（通常は `gpiochip4`）。
今は TM1637 を外しているので、`dashboard.launch.py` の `use_seg_display` は既定 `false`。
つないだときだけ `use_seg_display:=true` を指定する。

表示コード:

| コード | 意味 |
|--------|------|
| `boot` | 起動後30秒以内でシステム状態未受信 |
| `rdy ` | 待機中 |
| `HAnd` | 手動操作中 |
| `ObSt` | 障害物停止 |
| `SLo ` | 減速 |
| `LoU ` | 低電圧（車が走っているときに 3 秒以上つづいたとき。起動から 2 分と、止まっているときは出さない） |
| `Err ` | システム状態未受信・タイムアウト |

## IMU (GY-BNO055) のセットアップ

`imu_node` は GY-BNO055（I2C バス1、ADRピンHigh、アドレス `0x29`）を読み取り、
`/imu/data`（`sensor_msgs/Imu`）へ publish する。I2C のアクセス権を追加して再ログインする。

```bash
sudo usermod -aG i2c super
# 再ログイン、またはサービスを再起動
i2cdetect -y 1    # 0x29 が表示されることを確認
```

`dashboard.launch.py` の `use_imu`（既定 `true`）で起動し、無効化する場合は
`use_imu:=false` を指定する。

## 落下防止センサー (VL53L1X ×2) のセットアップ

`cliff_node` は前後バンパーの Dovhmoh VL53L1X（真下から進行方向へ 45° 前傾）で床までの距離を測り、
段差の判定を `/cliff_status`（`std_msgs/String`、JSON、20 Hz）へ publish する。
測った距離を前後それぞれの平らな床の距離（`front_floor_distance_m` 0.088 m・`rear_floor_distance_m` 0.085 m）とくらべ、
床を 0 cm とした高さ `height_cm = (床の距離 - 測った距離) × cos 45° × 100` に直す。床より `cliff_height_cm`（既定 2.0 cm）以上低いか、
床が見えない状態が `confirm_count` 回続くと段差とする。ダッシュボードの落下防止センサーのカードは、この高さを cm で出す。`drive_mode_node` は前が段差なら前進、後ろが段差なら後退を止める。
自動運転は停止モードにせず、`autonomy_node` が `/odom` で 0.10 m 下がり、空いている側へ 120° 回って走行を続ける（前後とも段差なら止まる）。

前後とも配線済み（`cliff_node.wired: true`・`front_wired: true`・`rear_wired: true`）。`wired: false` の間は I2C / GPIO に触らず
「未配線」だけを配信し、`front_wired` / `rear_wired` を false にした側も「未配線」になる
（ダッシュボードの「落下防止センサー」カードに表示）。ピン割り付けと配線色は `gpio_pins.yaml`。基板のピンは上から VIN・GND・SCL・SDA・GPIO1・XSHUT の 6 本。

| 基板のピン | 配線色 | 前（0x30） | 後（0x31） |
| --- | --- | --- | --- |
| VIN 3V3 | 白 | Motor HAT (3V3 power) | Motor HAT (3V3 power) |
| GND | 黄 | Motor HAT (Ground) | Motor HAT (Ground) |
| SCL | 灰 | pin 5（I2C バス 1 共有） | pin 5 |
| SDA | 青 | pin 3（I2C バス 1 共有） | pin 3 |
| GPIO1（INT） | － | つながない | つながない |
| XSHUT | 緑 | pin 11（GPIO17） | pin 13（GPIO27） |

pin 3（SDA）は IMU と前後の青、pin 5（SCL）は IMU と前後の灰を、それぞれ 3 又でまとめて挿す（SDA と SCL を同じ 3 又に入れない）。

VL53L1X の初期アドレス 0x29 は BNO055 と同じなので、`cliff_node` が XSHUT を 1 台ずつ上げ、
起動直後に 0x30 / 0x31 へ変える。モジュールの XSHUT は基板上で引き上げられていて、
配線したのに `cliff_node` がその側を使っていない（`front_wired` / `rear_wired` が false）と、0x29 で BNO055 とぶつかり
IMU が止まる。配線したら `dashboard.yaml` の `cliff_node.wired` と、その側の `*_wired` を `true` にしてサービスを再起動し、`i2cdetect -y 1` で
0x29（BNO055）・0x30・0x31・0x60（Motor HAT）が見えることを確かめる。
`dashboard.launch.py` の `use_cliff`（既定 `true`）で起動する。

## カメラ (IMX708 / Camera Module v3) のセットアップ（Ubuntu 24.04）

ダッシュボードは `/camera/image_raw/compressed` を購読し、`/api/camera/stream` で MJPEG 配信する。カメラノードは `camera_ros`（libcamera）を使用し、`dashboard.launch.py` の `use_camera`（既定 true）で起動する。

Ubuntu 24.04 の libcamera 0.7.2 は raspi カーネル 6.8 のメディアエンティティ名（`rp1-cfe-fe_image0` などアンダースコア形式）と一致せず `no cameras available` となるため、Raspberry Pi 版 libcamera / libpisp を `~/opt/rpicam` に導入する。

```bash
sudo apt install -y ros-jazzy-camera-ros
sudo apt install -y meson ninja-build pkg-config python3-jinja2 python3-yaml python3-ply \
  libyaml-dev libudev-dev libevent-dev libdrm-dev libjpeg-dev libtiff-dev libpng-dev \
  libssl-dev libgnutls28-dev libboost-dev libglib2.0-dev libgstreamer-plugins-base1.0-dev
# 開発用パッケージが解決できない場合は /etc/apt/sources.list.d/ubuntu.sources に noble-updates を追加する

git clone https://github.com/raspberrypi/libpisp.git ~/libpisp
cd ~/libpisp && meson setup build --prefix=$HOME/opt/rpicam && ninja -C build install

git clone https://github.com/raspberrypi/libcamera.git ~/libcamera
cd ~/libcamera
PKG_CONFIG_PATH=$HOME/opt/rpicam/lib/$(uname -m)-linux-gnu/pkgconfig \
  meson setup build --prefix=$HOME/opt/rpicam \
  -Dpipelines=rpi/pisp,rpi/vc4 -Dipas=rpi/pisp,rpi/vc4 \
  -Dv4l2=false -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
  -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled \
  -Dcpp_args=-I$HOME/opt/rpicam/include
ninja -C build install

# 確認
LD_LIBRARY_PATH=$HOME/opt/rpicam/lib/$(uname -m)-linux-gnu $HOME/opt/rpicam/bin/cam -l
```

`dashboard.launch.py` は `~/opt/rpicam` があればカメラノードの `LD_LIBRARY_PATH` に自動で追加する（apt 版 `camera_ros` をそのまま利用できる）。

## AI HAT+ (Hailo-8) のセットアップ（Ubuntu 24.04）

`hailo-all` は Raspberry Pi OS 専用のため Ubuntu では使えない。ドライバと HailoRT をソースから導入する。

```bash
sudo apt-get install -y linux-headers-$(uname -r) build-essential cmake git
git clone https://github.com/hailo-ai/hailort-drivers.git
cd hailort-drivers && git checkout v4.24.0          # v5 系は Hailo-10 専用で Hailo-8 (1e60:2864) 非対応
cd linux/pcie && make all && sudo make install
cd ~/hailort-drivers && ./download_firmware.sh
sudo mkdir -p /lib/firmware/hailo
sudo cp hailo8_fw*.bin /lib/firmware/hailo/hailo8_fw.bin
sudo cp linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
sudo depmod -a && sudo udevadm control --reload-rules && sudo udevadm trigger
sudo reboot

# 再起動後に HailoRT（ドライバと同じ 4.24.0）をビルド
git clone --depth 1 --branch v4.24.0 https://github.com/hailo-ai/hailort.git
cd hailort && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DHAILO_BUILD_PYBIND=OFF
cmake --build build --config release -j2
sudo cmake --install build && sudo ldconfig
hailortcli fw-control identify
```

### Python バインディング（`perception_node` の推論に必要）

```bash
cmake -S ~/hailort/hailort/libhailort/bindings/python/src -B ~/pyhailort_build \
      -DPYBIND11_PYTHON_VERSION=3.12 -DCMAKE_BUILD_TYPE=Release
cmake --build ~/pyhailort_build -j4
cp ~/pyhailort_build/_pyhailort*.so \
   ~/hailort/hailort/libhailort/bindings/python/platform/hailo_platform/pyhailort/
pip3 install --user --break-system-packages \
   ~/hailort/hailort/libhailort/bindings/python/platform
python3 -c "from hailo_platform import VDevice; print('ok')"
```

### 物体検出モデル

```bash
mkdir -p ~/AI-CAR_ws/models
curl -L -o ~/AI-CAR_ws/models/yolov8m.hef \
  https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.16.0/hailo8/yolov8m.hef
```

今の設定では `config/dashboard.yaml` の `perception_node.hef_path` に `/home/super/AI-CAR_ws/models/yolov8m.hef`（`~` は使えないのでフルパス）を書いて YOLOv8m を動かしている（10 回/秒）。物の名前を出すだけで、`living_guard: false` なので速さには使わない。止めるときは `hef_path` を空（`""`）にする。

## 依存

```bash
sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro
# SLAM（地図と自己位置、use_slam:=true）を使うとき
# 地図の保存はダッシュボードが /map から .pgm / .yaml を書く（nav2_map_server は手で map_saver_cli を使うとき）
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
# Nav2（地図の行き先へ行く Level 2、use_nav2:=true）を使うとき
sudo apt install -y ros-jazzy-navigation2
sudo apt install -y python3-psutil
pip3 install fastapi uvicorn
# WebSocket 配信（未導入なら /api/status の 250ms ポーリングへ自動フォールバック）。
# apt の python3-websockets 10.4 は uvicorn が要求する API を持たないため使わない。
pip3 install --user --break-system-packages "websockets>=13"
```

映像の滑らかさは `config/dashboard.yaml` で調整する。既定は `camera` の `width: 960` /
`height: 540` / `jpeg_quality: 80`（`FrameDurationLimits` により約 30fps。以前の 1280×720 は camera_node の CPU が約 50% になるため縮小）、`dashboard_node.camera_stream_rate: 30.0`、
`perception_node.inference_rate: 15.0`、`dashboard_node.telemetry_rate: 10.0`、`dashboard_node.scan_max_points: 720`、
`autonomy_node.publish_rate: 10.0`、`joy_teleop_node.publish_rate: 10.0`、`imu_node.publish_rate: 20.0` に設定している。
帯域や CPU が厳しい場合は解像度か `jpeg_quality` を下げる。

## ドキュメント

- 開発方針: `PROJECT_RULES.md`
- 実装状況: `PROJECT_STATUS.md`
- ハードウェア構成: `HARDWARE_BOM.md`
- 変更履歴: `CHANGELOG.md`
- 部品の形: `hardware/3d/`（STL・組み立て図）

## Jetson 側（`jetson/`）

Jetson Orin Nano Super（`jetson@<Tailscale IP>`）で動かすもの。それぞれの README に入れ方がある。

| フォルダ | サービス | 役割 |
|----------|----------|------|
| `stackchan_bridge` | `jetson-stackchan.service` | スタックちゃんの会話（聞き取り・会話の AI・声）、話して走る、パソコンが動いていればパソコンで処理 |
| `status_reporter` | `jetson-status.service`・`jetson-upgrade.service` | Jetson の状態をラズパイのダッシュボードへ 1 秒ごとに送る・更新 |
| `oled_panel` | `jetson-oled.service` | ケースの OLED に CPU・温度・メモリ・SSD・IP |
| `ollama` | `ollama.service`・`ollama-preload.service` | 会話の AI（qwen2.5:3b）を読み込んだままにする |
| `health_log` | `jetson-health.service`・`.timer` | 前の起動の記録と 1 分ごとの様子 |
| `object_finder` | `jetson-finder.service` | 声で覚えた物を探す。「覚えて」でカメラのまん中の物を覚え、物を動かしてから「探して」で自動走行しながら最後に覚えた物を探し、見つけたら車を止める（場所は覚えない、5 分で見つからなければ止める。名前つきも使える） |
| `owl_detector` | `jetson-owl.service` | NanoOWL の物体検出（ラズパイへ `/jetson_detections`）。メモリを空けるため 2026/10 から止めている（disabled） |
| `power_saving` | - | Wi-Fi の省電力を切る |
| `voice_chat` | - | Bluetooth のスピーカー＆マイクで話す（試し） |

## バックアップと復元（v1.30.0）

スナップショット（バックアップ）は Pi の SSD 内 `~/ai-car-backups/` に保存する（輪転30個）＋ Windows PC へミラー。1スナップショット = `ai-car-<日時>-v<version>-<gitsha>.tar.zst` ＋ `manifest.json`（tar の sha256 含む）。

- **作る**：`bash scripts/ai-car-backup.sh`（superで可）。毎日04:00に `ai-car-backup.timer` がrootで自動実行。ダッシュボードの「バックアップ」カードからスナップショットを作成（`sudo -n systemctl start --no-block ai-car-backup.service`、sudoersホワイトリスト）。
- **中身**：Tier A＝home一式（`AI-CAR_ws`（build/install/log/ai-car-backups除く）、calib、patrols、poselog・rotlog・heading_log、home直下のテストファイル、`.config/aicar`）、Tier B＝`/etc/systemd/system/ai-car-*`・udev rule・`/boot/firmware/config.txt`+`cmdline.txt`・aptマニフェスト・`tailscale status --json`・crontab。sudoers の中身はリポジトリの `systemd/*.sudoers` が正。
- **復元（稼働Pi・R2）**：`sudo /usr/local/sbin/ai-car-restore <スナップショット名>`（sha256照合→dashboard停止→Tier A rsync上書き→Tier B units/udev/config.txt再導入→dashboard再起動）。colcon buildは自動実行しない（必要時 `colcon build --symlink-install`）。
- **新規SD復元（R3）**：Ubuntu 24.04 インストール→`git clone`→`sudo systemd/install_service.sh`→このREADMEのlibcamera/hailortビルド手順→`ai-car-restore` でデータ復元→Tailscale再登録→受入＝8080起動・status全ノードalive・maps個数一致。
- **開発フック**：PRマージ前に必ずスナップショットを作る（strata側で自動化）。`install_service.sh` のsudo導入はユーザーのパスワードで1回だけ必要。
- **Jetson用（v1.30.2）**：`scripts/ai-jetson-backup.sh`（Piと同一設計）。Tier A＝`/home/jetson`のサービスコード群（stackchan_bridge・face_id・object_finder・oled_panel・owl_detector・health_log・objects・fw_clear等）＋`.ollama/models`＋`.cache`＋home直下ファイル、Tier B＝`jetson-*`ユニット・sudoers内容記録・`extlinux.conf`・netplan・aptマニフェスト・tailscale・crontab。Jetson側 `~/jetson-backups/` 輪転7、PCミラー照合。sudoersは440でjetsonが読めないため内容記録を同梱。復元（R3相当）は新規Jetson时READMEの各サービス導入手順＋スナップショット展開。
- **注意**：同一SSD内のスナップショットは削除・暴走からの回復用。SSD故障はPCミラーが救う（PC側 `Strata/Workspace/ai-car-backups/` にsha256照合済みミラー）。
