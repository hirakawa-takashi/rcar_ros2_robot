# rcar_ros2_robot (AI-CAR)

Raspberry Pi 5 + ROS2 Jazzy で動作するメカナム4輪自律走行ロボット AI-CAR のワークスペース。

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

起動時の自動起動（systemd）:

```bash
sudo ~/AI-CAR_ws/systemd/install_service.sh   # /etc/systemd/system/ai-car-dashboard.service を登録して起動
systemctl status ai-car-dashboard.service
sudo systemctl disable --now ai-car-dashboard.service   # 手動起動に戻す場合
```

サービスは `Restart=always`（5秒間隔）で、USB デバイスの認識を待つため起動を10秒遅延する。カメラと LiDAR のノードは launch 側で `respawn` する。

CPU ファンの作動温度（既定 50/60/67.5/75℃）を前倒しして高温になりにくくする:

```bash
sudo cp ~/AI-CAR_ws/systemd/ai-car-fan.service /etc/systemd/system/
sudo systemctl enable --now ai-car-fan.service   # 45/50/55/62℃ を設定（起動ごとに再適用）
# 温度を変えたい場合は Environment=FAN_TRIP_TEMPS="..." をサービスに追加
```

設定値は `scripts/set_fan_curve.sh` が `thermal_zone0` の active トリップへ書き込む。ファン回転数と段階はダッシュボードの CPU カードに表示される。

## インターフェース

`dashboard_node`

- Publish: `/cmd_vel` (`geometry_msgs/Twist`)、`~/status` (`std_msgs/String`)
- Subscribe: `/scan` (`sensor_msgs/LaserScan`)、`/imu/data` (`sensor_msgs/Imu`)、`/odom` (`nav_msgs/Odometry`)、`/system_status` (`std_msgs/String`)、`/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`)
- HTTP: `GET /`、`GET /api/status`、`POST /api/cmd_vel`、`POST /api/stop`、`GET /api/camera/snapshot`、`GET /api/camera/stream`（MJPEG）、WebSocket `/ws`
- パラメータ: `src/ai_car_web/config/dashboard.yaml`

操作コマンドは -1.0〜1.0 の正規化値で受け取り、`max_linear_speed` / `max_angular_speed` にスケールされる。
`cmd_timeout`（既定 0.7 秒）の間に新しい指令が来ない場合は自動的に停止する。

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
curl -L -o ~/AI-CAR_ws/models/yolov8n.hef \
  https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.16.0/hailo8/yolov8n.hef
```

パスは `config/dashboard.yaml` の `perception_node.hef_path` で指定する（未設定なら LiDAR 判定のみで動作）。

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
