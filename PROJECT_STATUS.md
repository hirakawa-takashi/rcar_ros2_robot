# PROJECT_STATUS.md

## 最終更新
- 更新日: 2026/09/15
- 更新概要: 状態表示器を TM1637 7 セグから ZJY-IPS130-V2.0（ST7789 1.3 インチ 240×240 IPS 液晶、SPI0、7 ピン CS なし）へ変更。新規 `lcd_display_node`（spidev + libgpiod + Pillow）が運転モード / 自律行動 / 前方距離 / 障害物 / 電圧 / CPU 温度 / パッド接続 / IP を描画し、従来と同じコードを `/display_state` に publish。配線: VCC pin17、GND pin20、SCL pin23(GPIO11)、SDA pin19(GPIO10)、RES pin18(GPIO24)、DC pin22(GPIO25)、BLK pin16(GPIO23)。launch 既定を `use_lcd_display=true` / `use_seg_display=false` に変更（GPIO23/24 共用のため同時起動不可）。`gpio_pins.yaml`・BOM・README を更新。以前: 運転モード管理 `drive_mode_node`（手動 / 自動 / 停止、`/cmd_vel` を発行する唯一のノード）と Level 1 自律走行 `autonomy_node`（LiDAR 反応型、`/cmd_vel_auto`）を追加。`dashboard_node` の手動指令出力を `/cmd_vel_manual` に変更し、`GET/POST /api/drive_mode` とテレメトリ `drive_mode` / `autonomy` を追加。ゲームパッドは START 長押し（2 秒）で手動 ⇄ 自動、BACK（自動中は B も）で停止。画面に運転モードバッジ・3 ボタン・自律走行カードを追加、7 セグに `AUto` / `StoP` を追加。以前: Motor HAT 接続図・一覧表を「1 モーター 6 本（Motor+ / Motor− / VCC / GND / Encoder A / Encoder B）」をすべて明示する形に変更（`GET /api/motor_hat` の各モーターに `wires` 6 件、`encoder` に `vcc` / `gnd` を追加、GND はモーターごとに pin 30/34/39/25 を割り当て、VCC は pin 17 共通。VCC / GND は Motor HAT 上の 3V3 / GND ピンに接続する表記に変更し、Pi GPIO ヘッダー表では該当ピンを空きとして表示）。表示から「520 モーター」の文言を削除し、ラベルは「M1 前左 Encoder A」形式。以前: モーターの配線色をメーカー資料に合わせて確定（赤 Motor+ / 白 Motor− / 青 VCC / 黒 GND / 緑 Encoder A / 黄 Encoder B）し、車輪表記を進行方向基準の「M1 前左」形式（前左 / 前右 / 後左 / 後右）に統一。接続図・一覧表・BOM・GPIO 表の表記を Motor+ / Motor− / Encoder A / B に揃えた。以前: OSOYOO 520 モーター内蔵エンコーダー（A/B 相）の Pi GPIO 直結割り付けを追加（`motor_hat.yaml` の `encoder`、`gpio_pins.yaml`。M1: 29/31、M2: 33/35、M3: 37/32、M4: 36/38、VCC 3V3 pin17、GND 30/34/39/25）。Motor HAT 接続図・一覧表・GPIO ヘッダー図にエンコーダーピンを表示。以前: ダッシュボードに「Motor HAT 接続図」カード（Adafruit Motor HAT の M1〜M4 と車輪位置の割り付け・配線図・一覧表）を追加。割り付けは `config/motor_hat.yaml`（M1=前左 / M2=前右 / M3=後左 / M4=後右、右側は `reversed: true`）で編集し `GET /api/motor_hat` で取得。以前: TM1637 4桁7セグメントLED用 `seg_display_node`（`/display_state`）と GY-BNO055 用 `imu_node`（I2C 0x29、`/imu/data`）を追加。Logitech F710 ゲームパッドによる手動操作（`joy_teleop_node`）を追加。以前: ダッシュボードに Raspberry Pi 5 の GPIO 40 ピンヘッダー図（横向き、上段 2〜40 / 下段 1〜39）と 40 ピン一覧表（使用中・配線色・接続先・信号）を追加。配線は `config/gpio_pins.yaml` で編集、`GET /api/gpio` で取得。以前: ダッシュボードの systemd 自動起動サービスと respawn 対応を追記。カメラと LiDAR のカードを先頭に移動し横幅を2列分に拡大。LiDAR の取り付け向きを 180° 補正し、カメラ検出物体の距離推定をクラスタ中央値に変更。CPU カードに入力電圧・入力電流と低電圧・スロットリング警告を追加し、CPU / AI HAT+ カードの注記行を削除。CPU カードのロードアベレージを削除し PD 対応（5A）の 〇 / × 表示に変更。
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
  - Publish: `/cmd_vel_manual`（geometry_msgs/Twist、手動指令。`/cmd_vel` は `drive_mode_node` が発行）、`/drive_mode_request`、`~/status`
  - `drive_mode_node`（運転モード管理）: `/cmd_vel_manual`（Web + ゲームパッド）と `/cmd_vel_auto`（自律走行）のどちらか一方だけを `/cmd_vel` に中継する。モードは manual / auto / stop（起動時 manual）。状態は `/drive_mode`（JSON, transient local, 5Hz）、切替要求は `/drive_mode_request`（JSON `{mode: manual|auto|stop|toggle, source}`）。安全機構: 自動へ入る条件（`/obstacle_status` が新鮮で stop / unknown でない・ゲームパッド接続中・スティック中立・`/cmd_vel_auto` 生存）を満たさないと拒否（`last_reject` に理由）、自動指令が `auto_timeout` 0.5 秒途絶／ゲームパッド切断／障害物判定途絶で STOP、自動中に手動指令を検出すると手動へ復帰（`manual_override`）、モード遷移時はゼロ速度を publish、最終段で障害物 stop 判定中の前進を 0 にする（`obstacle_guard`）。launch 引数 `use_drive_mode`
  - `autonomy_node`（Level 1 自律走行・LiDAR 反応型）: `/scan` を前 / 左 / 右 / 後セクターに分けて最短距離を求め、前進（0.15 m/s）→ 0.8 m 以内で減速（0.08 m/s、空いている側へ緩く曲がる）→ 0.35 m 以内で空いている側へその場旋回（0.6 rad/s、前方 0.9 m 以上空くまで）→ 前進を繰り返す。前後左右とも 0.25 m 以内なら後退。LiDAR が 1 秒以上途絶えると停止。`/cmd_vel_auto` に 20Hz で常時 publish（AUTO 以外はゼロ速度で待機）、状態は `/autonomy_status`（JSON: behavior / sectors / cmd）。地図・自己位置は使わない。launch 引数 `use_autonomy`
  - 運転モード UI: 手動操作カード上部にモードバッジ（手動 / 自動 / 停止）と 3 ボタン（`POST /api/drive_mode`）、拒否理由表示。自律走行カードに行動・各方向距離・自動指令・`/cmd_vel` 出力を表示
  - Subscribe: `/scan`、`/imu/data`、`/odom`、`/camera/image_raw/compressed`
  - `imu_node`: GY-BNO055（I2C バス1、アドレス0x29）を読み取り、`/imu/data`（sensor_msgs/Imu）を50Hzでpublish。パラメータは `i2c_bus` / `i2c_address` / `frame_id` / `imu_topic` / `publish_rate`
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
  - ゲームパッド手動操作（`joy_teleop_node`）: Logitech F710（XInput モード、`/dev/input/js0`）を Linux joystick API で直接読み（追加の Python 依存なし）、正規化済み Twist を `/joy_cmd` に 20Hz で publish。`dashboard_node` が `/joy_cmd` を購読して Web 操作と同じ `publish_cmd_vel()` 経路で `/cmd_vel` に変換するため、最大速度・障害物ガード（減速／停止）・`cmd_timeout` 停止が同様に適用される。割り当て: 左スティック上下=前後、左スティック左右=平行移動、右スティック左右=旋回、十字キー=前後・平行移動（デジタル）、RB=全速（通常は `speed_scale` 0.5）、B=停止（押している間。自動中は STOP モード要求）、START 長押し 2 秒=手動 ⇄ 自動、BACK=STOP モード、`button_enable` でデッドマンボタンも設定可（既定は無効）。F710 未接続でも起動し、抜き差しを検出して再接続（切断時は指令を出さない）。画面の手動操作カードに接続状態（未接続／接続中／操作中）を表示。launch 引数 `use_joy` で無効化可
  - `lcd_display_node`: ZJY-IPS130-V2.0（ST7789、SPI0 /dev/spidev0.0、CS なし、DC=GPIO25 / RES=GPIO24 / BLK=GPIO23）を spidev + libgpiod + Pillow で駆動。`/system_status`・`/obstacle_status`・`/joy_cmd`・`/cmd_vel`・`/drive_mode`・`/autonomy_status` から状態を判定し、上段に状態（起動中 / 待機 / 手動 / 自動 / 停止 / 障害物停止 / 減速 / 低電圧 / システム未受信）、下段に運転モード・自律行動・前方距離・障害物・電圧・CPU 温度・パッド接続・IP を描画。`/display_state` へ従来コードを publish。`use_lcd_display`（既定 true）で切替、`rotation` パラメータで向き調整。依存: python3-spidev, python3-libgpiod, python3-pil, fonts-ipafont-gothic
  - `seg_display_node`（旧表示器、既定では起動しない）: TM1637 4桁7セグメントLEDを libgpiod で駆動し、`/system_status`・`/obstacle_status`・`/joy_cmd`・`/cmd_vel` と `/drive_mode` から状態を判定して `/display_state`（std_msgs/String）へ publish。表示: boot / rdy / HAnd / AUto / StoP / ObSt / SLo / LoU / Err。`use_seg_display` で無効化可
  - パラメータ: `config/dashboard.yaml`（host/port/各トピック名/最大速度/タイムアウト/配信レート）
  - 自動起動: `systemd/ai-car-dashboard.service`（`install_service.sh` で登録、`Restart=always`、USB デバイス待ちで起動を10秒遅延）。カメラ / LiDAR / 液晶表示ノードは launch 側で `respawn`
  - 自動起動: `systemd/ai-car-dashboard.service`（`install_service.sh` で登録、`Restart=always`、USB デバイス待ちで起動を10秒遅延）。カメラ / LiDAR / IMU ノードは launch 側で `respawn`
  - `system_monitor_node`: Raspberry Pi 5 / AI HAT+ の状態監視（読み取りのみ）
    - Publish: `/system_status`（std_msgs/String, JSON、1Hz）
    - 取得内容: CPU使用率（全体・コア別）・クロック・温度・ロードアベレージ、メモリ/Swap/ディスク、PMIC レール別電圧・電流・電力、スロットリング状態、Hailo-8 検出状態
    - パラメータ: `publish_rate` / `topic` / `enable_pmic` / `enable_hailo`
    - 依存: `psutil`、`vcgencmd`、`lspci`、`hailortcli`（あれば利用）
  - GPIO 40 ピンヘッダーカード（全幅）: `ai_car_web/gpio_pinout.py` の固定ピン定義（物理番号・名称・BCM・種別）と `config/gpio_pins.yaml` の配線定義（`pins` / `device` / `signal` / `color` / `note`）を結合し `GET /api/gpio` で返す。画面は SVG のヘッダー図（種別ごとの色、使用中ピンは緑の外枠、配線色バー、ホバーで接続先表示）と一覧表（ピン / 名称 / BCM / 使用 / 配線色 / 接続先 / 信号・備考）。同一ピンへの複数接続（I2C バス共有）は「/」区切りで併記。パラメータ `gpio_config`（空なら share 内の `gpio_pins.yaml`）。YAML 編集後は `dashboard_node` 再起動で反映
  - Motor HAT 接続図カード（全幅）: `ai_car_web/motor_hat.py` が `config/motor_hat.yaml` の割り付け（`channel` / `wheel` / `reversed` / `colors` / `note`）に端子ごとの固定情報（PCA9685 の PWM / IN1 / IN2 チャンネル、TB6612 ブリッジ）を重ねて `GET /api/motor_hat` で返す。割り付け: M1=前左（front_left）/ M2=前右（front_right）/ M3=後左（rear_left）/ M4=後右（rear_right）。右側 2 輪は左右対称取り付けのため `reversed: true`（モーター制御ノードで符号反転する前提）。画面は上面図の SVG（前方が上、中央に HAT の端子台 M1 M2 | +− | M3 M4、四隅に車輪、端子→車輪の配線を +/− の配線色で描画、電源端子→12V 系統）と一覧表（端子 / 車輪 / 回転 / 配線色 / PCA9685 ch / ブリッジ / 備考）。端子・車輪の重複や未割り付けは警告表示。各モーターの `encoder`（`a_pin` / `b_pin` / `colors`）と `hat.encoder`（`vcc_pin` / `gnd_pins`）で 520 モーターエンコーダーの Pi 直結ピンを定義し、BCM 番号へ解決・非 GPIO ピン・I2C/ID ピン・ピン重複・VCC が 3V3 以外を警告。接続図は車輪ごとに「M1 前左」＋ 6 本の線（名称・配線色・接続先）を列挙し、一覧表は 1 線 1 行（モーター / 線番号 / ピン名称 / 配線色 / 接続先 / 回転・ドライバ・備考）。パラメータ `motor_hat_config`（空なら share 内の `motor_hat.yaml`）

## 現在作業中
- ZJY-IPS130-V2.0 液晶の実機表示確認（モジュール配線後。ノード起動と SPI/GPIO 確保は確認済み、画面表示は未確認）

## 未実装機能
- モーター制御ノード（Adafruit Motor HAT / メカナム逆運動学。`/cmd_vel` を購読する）
- エンコーダー読み取り・オドメトリ発行
- SLAM（slam_toolbox）
- Level 2 自律走行（Nav2 による地図ベースのナビゲーション）。現在は Level 1（LiDAR 反応型）のみ

## 既知の問題
- Pi には rviz2 が未インストール（ros-base のみ）。RViz2 は開発PC側での表示を想定
- `/odom` のノードが未実装のため、オドメトリのテレメトリは現状 null
- LiDAR は `ros-jazzy-rplidar-ros` の導入と `super` の `dialout` グループ所属が必要
- AI HAT+ は `hailo_pci` 4.24.0 と HailoRT 4.24.0 をソース導入済み（Ubuntu 24.04 に `hailo-all` は存在しない。手順は README 参照）。温度は HailoRT C API `hailo_get_chip_temperature()` で取得済み。NPU 使用率と電力はダッシュボードでは扱わない（使用率は `HAILO_MONITOR=1` の推論アプリがある間のみ `hailortcli monitor` で参照可）。電力は未取得（`measure-power` は `UNSUPPORTED_OPCODE`、`query_health_stats()`/`query_performance_stats()` は HAILO8 非対応、`hatctl` は Ubuntu に存在せず Hailo 専用 hwmon も無し、PMIC に HAT 専用レール無し。外付け INA219 等が必要）
- 検出物体の距離は LiDAR の方位角対応による概算で、カメラとLiDARの外部キャリブレーションは未実施（設置が同軸・同方向である前提）
- `perception_node` の推論には HailoRT の Python バインディング（`hailo_platform`）が必要。apt には無いため HailoRT 4.24.0 のソースからビルドして導入する（手順は README 参照）。未導入の場合は LiDAR 判定のみで動作し、`/obstacle_status` の `note` に理由を表示する
- 実機の電源が不足している。ダッシュボード（カメラ＋LiDAR＋Hailo 推論）起動と同時に `hwmon3: Undervoltage detected!` が連発し、2026/09/14 19:22 に shutdown シーケンスなしで電源断した（前回起動だけで 198 回発生、EXT5V は 4.75、5.11V、`get_throttled=0x50000`）。`usb_max_current_enable=0` で 5A PD として認識されておらず、Pi 5 が制限モード（USB 周辺機器 合計 600mA）で動作している。公式 27W USB-C PD 電源への交換と、RPLIDAR の別系統（セルフパワーハブ等）給電が必要
- カメラは Ubuntu 標準の libcamera 0.7.2 だと raspi カーネル 6.8 のエンティティ名不一致で `no cameras available` となる。`~/opt/rpicam` の Raspberry Pi 版 libcamera を使う必要がある（手順は README 参照）
- `vcgencmd get_throttled` が `0x50000` = 過去に低電圧/スロットリングを検出（現在は正常）
- `motor_hat.yaml` の割り付け（M1〜M4 と車輪位置、`reversed`、配線色）は暫定値。実機のモーター配線と照合し、モーター制御ノード実装時に回転方向を確認して修正が必要
- 520 モーターエンコーダーの GPIO 割り付け（`motor_hat.yaml` `encoder` / `gpio_pins.yaml`）と配線色（VCC 青 / GND 黒 / A 黄 / B 緑）は未使用ピンから選んだ暫定値。実配線と照合して修正が必要。エンコーダー読み取りノードは未実装
- `gpio_pins.yaml` の配線内容（Motor HAT / BNO055 の I2C・電源・GND、配線色）は HARDWARE_BOM からの暫定値。実配線と照合して修正が必要。TM1637 の配線色は実配線を反映済み（オレンジ・黒・黄・緑）。AI HAT+ の ID EEPROM ピン（27/28）はヘッダー直挿しのため配線色は `PIN` 表記
- WebSocket は `pip install --user --break-system-packages "websockets>=13"` で有効化済み（apt の python3-websockets 10.4 は uvicorn が要求する `ServerProtocol` を持たず、入れると dashboard_node が ImportError で起動しない）。未導入環境では `/api/status` の 250ms ポーリングへ自動フォールバックする

## テスト結果
- `lcd_display_node`（実機、2026/09/15）: `python3-spidev` / `fonts-ipafont-gothic` を apt 導入、`/dev/spidev0.0` は dialout グループで super から書き込み可。サービス再起動後 `lcd_display_node` が起動し「ST7789 液晶表示を接続しました」、`/display_state` に `rdy ` を publish。`seg_display_node` は起動していないことを確認。**液晶モジュール未配線のため実際の画面表示（初期化シーケンス・SPI モード 3・向き・色）は未確認**。表示が出ない場合は `spi_speed_hz` を下げる、`spi.mode` を 0 にする、`rotation` を変えるなどの調整が必要
- `seg_display_node`（実機、2026/09/15、旧表示器）: `python3-libgpiod` を apt 導入後、`gpiochip4 (pinctrl-rp1)` の GPIO23/24 を `seg_display_node` が output で確保（`gpioinfo` で確認）。起動 1 秒で `boot` → `/system_status` 受信後 `rdy` に遷移、`/display_state` を publish。TM1637 配線後に LED の点灯を確認。初回は CLK 線の挿し違いで消灯だった（GPIO24 にはモジュールのプルアップを検出、GPIO23 にはなし → CLK 未接続と判定）。スティック操作時の `HAnd` 表示は未確認
- `imu_node`（実機、2026/09/15）: `i2cdetect -y 1` で 0x29 を検出、`BNO055 接続` ログ後に `/imu/data` を 50.0Hz で受信。加速度の合成値 約9.8m/s²、キャリブレーション gyr=3 まで進行を確認。`/api/status` の `imu` に roll/pitch/yaw・角速度・加速度が入ることを確認（ブラウザ表示は未確認）。`super` を `i2c` グループへ追加が必要（未所属だと Permission denied）
- 取り付け向き: 静止時に加速度 x≈6.9 / y≈4.0 / z≈-5.8 と重力が z 軸に乗っておらず、センサーの搭載向きが `imu_link`（機体と同一向き）と一致していない。向き確定後に軸の入れ替え（またはURDF の `imu_joint` の rpy）で補正が必要
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
- Motor HAT 接続図: `load_motor_hat('config/motor_hat.yaml')` で 4 端子の割り付けと警告なし、YAML 不在時は `error` を返すことを確認。開発 PC 上で `/api/motor_hat` を模擬した静的サーバーにより、ブラウザで接続図（4 輪・配線・端子台）と一覧表の表示を確認。実機（AI-CAR）で `GET /api/motor_hat` が 200 で割り付けを返すことと、I2C 0x60 に Motor HAT を検出することを確認。エンコーダー割り付けは `load_motor_hat` で警告なし・BCM 解決を確認し、`build_pinout` で信号ピンの競合なし（使用 25/40）を確認。開発 PC のブラウザでエンコーダーピン付きの接続図・一覧表を確認（実機は未反映）。`flake8 --max-line-length 100` エラー無し
- GPIO ヘッダー: `build_pinout('config/gpio_pins.yaml')` で使用中 9 ピン（1,2,3,4,5,6,9,27,28）、YAML 不在時は `error` を返すことを確認。実機で `GET /api/gpio` → `used_count: 9`、ブラウザでヘッダー図（上下段ラベルの重なり無し）と 40 行の一覧表の表示を確認。`flake8 --max-line-length 100` エラー無し
- `ros2 launch ai_car_description view_robot.launch.py`: 起動成功（`/robot_description`・`/joint_states`・`/tf` 発行を確認）

## カメラ仕様（実測）
- 解像度: 1280x720 / JPEG 品質 80（IMX708 / Camera Module v3、`config/dashboard.yaml` の `width`/`height`/`jpeg_quality`）。1 枚 約100KB・約2.9MB/s、camera_ros の CPU は約 36〜45%
- ROS 配信レート: 約 30Hz（`/camera/image_raw/compressed`）
- ダッシュボード MJPEG: 30fps（`camera_stream_rate`）
- AI HAT+ 推論: 640x640 にリサイズして 10Hz 設定（実測 約5Hz、JPEG デコード＋リサイズが律速。推論自体は 約9ms、高温時は自動低下）

## 次回作業
- 実機で自動モードの走行挙動（旋回方向・速度・距離しきい値）を確認し `autonomy_node` のパラメータを調整する（モーター配線後）
- モーター制御ノード（`/cmd_vel` 購読 → Motor HAT 駆動）を実装する。`motor_hat.yaml` の割り付け（`channel` / `wheel` / `reversed`）を参照してメカナム逆運動学の出力を各端子へ振り分ける
- 実配線（M1〜M4とエンコーダージャンパー線）に合わせて `motor_hat.yaml` / `gpio_pins.yaml` を修正する
- エンコーダー読み取りノード（libgpiod で A/B 相をカウントし `/wheel_ticks` 等を publish）を実装し、カウント方向を確認する

## 変更ファイル
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/drive_mode_node.py` - 新規作成（運転モード管理）
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/autonomy_node.py` - 新規作成（Level 1 自律走行）
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/lcd_display_node.py` - 新規作成（ZJY-IPS130-V2.0 / ST7789 液晶表示）
- `/home/super/AI-CAR_ws/src/ai_car_web/{setup.py, config/dashboard.yaml, config/gpio_pins.yaml, launch/dashboard.launch.py}`、`HARDWARE_BOM.md`、`README.md` - 液晶表示器への置き換えで更新
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/dashboard_node.py, ai_car_web/joy_teleop_node.py, ai_car_web/seg_display_node.py, config/dashboard.yaml, launch/dashboard.launch.py, setup.py, static/index.html}` - 運転モード切替・自律走行対応で更新
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
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/imu_node.py` - 新規作成（GY-BNO055 I2C IMU）
- `/home/super/AI-CAR_ws/src/ai_car_web/{setup.py, config/dashboard.yaml, launch/dashboard.launch.py, config/gpio_pins.yaml, static/index.html}` - IMUノード対応で更新
- `/home/super/AI-CAR_ws/src/ai_car_web/ai_car_web/motor_hat.py`、`config/motor_hat.yaml` - 新規作成（Motor HAT M1〜M4 割り付け）
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/dashboard_node.py, config/dashboard.yaml, static/index.html}` - Motor HAT 接続図カード対応で更新
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/motor_hat.py, config/motor_hat.yaml, config/gpio_pins.yaml, static/index.html}`、`HARDWARE_BOM.md` - 520 モーターエンコーダーの Pi GPIO 割り付けを追加
- `/home/super/AI-CAR_ws/PROJECT_STATUS.md` - 本ファイル
- `/home/super/AI-CAR_ws/CHANGELOG.md` - 更新
