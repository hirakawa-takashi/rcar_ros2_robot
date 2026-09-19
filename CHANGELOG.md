# CHANGELOG.md

## [Unreleased]

### Added
- CPU 負荷軽減のためカメラ解像度を 1280×720 → 960×540 に変更（camera_node の JPEG 圧縮負荷を削減。検出側はレターボックスで 640×640 に揃えるため入力サイズ非依存）
- CPU 負荷軽減のため推論レート上限を 15 Hz（YOLOv8m の実効約 12 Hz は不変）、MJPEG 配信を 15 fps に変更（カメラ取り込みは 30 fps のまま）
- カメラ物体検出を YOLOv8m に変更し推論レートを 30 Hz に引き上げ（推論タイマー 0.2 s → 0.02 s で `inference_rate` 上限が実際に効くよう修正。従来は実効 5 Hz 上限）
- ダッシュボードにネイビー×シアンの計器パネル風テーマを適用（CSS のみ。カードのガラス風背景・数値の等幅フォント・状態バッジの発光・接続ドット / AUTO バッジのパルス・カメラ / LiDAR 枠のコーナーマーカー。レイアウトと表示内容は不変）
- GPIO 40 ピンヘッダー / Motor HAT 接続図カードをメイン画面からプロジェクト説明「ハードウェア構成」タブへ移動
- LiDAR (/scan) 表示の距離リングに数値ラベル（m）を追加
- プロジェクト説明ボタンと7タブの全画面モーダルを追加（`architecture.yaml` / `/api/architecture` による構成・安全機構・API・開発状況表示、dashboard.yaml の実値と生存ノード / Topic を反映）
- 運転モード切替・手動指令の安全停止を修正し、手動指令ウォッチドッグと起動時ゼロ出力、自律走行の後方距離未受信時の後退抑止、ダッシュボードの停止動作と任意トークン認証を追加
- ZJY-IPS130-V2.0（ST7789 1.3 インチ 240×240 IPS 液晶、SPI0、7 ピン CS なし）用 `lcd_display_node` を追加（spidev + libgpiod + Pillow。運転モード・自律行動・前方距離・障害物・電圧・CPU 温度・パッド接続・IP を描画、`/display_state` は従来コード互換）
- `gpio_pins.yaml` に ZJY-IPS130 液晶の配線色（GND 黒 / VCC 赤 / SCL 橙 / SDA 黄 / RES 緑 / DC 青 / BLK 紫）を設定
- 運転モード管理ノード `drive_mode_node` を追加（手動 / 自動 / 停止。`/cmd_vel_manual` と `/cmd_vel_auto` のどちらか一方だけを `/cmd_vel` に中継。`/drive_mode` 状態、`/drive_mode_request` 切替要求、自動進入条件・指令途絶・ゲームパッド切断時の安全停止）
- Level 1 自律走行ノード `autonomy_node` を追加（LiDAR 反応型: 前進 → 減速 → 空いている側へ旋回。`/cmd_vel_auto`、`/autonomy_status`）
- ゲームパッド: START 長押し 2 秒で手動 ⇄ 自動、BACK（自動中は B も）で停止
- ダッシュボード: 運転モードバッジ・3 ボタン（`GET/POST /api/drive_mode`）と自律走行カードを追加。7 セグに `AUto` / `StoP` を追加

### Changed
- `perception_node` のカメラ推論を YOLOv8m（640x640 / 78.9 GOP）へ更新。レターボックス前処理でアスペクト比を維持し、検出枠を元画像座標へ復元。信頼度 0.5 と複数フレーム確認（履歴3フレーム中2回）で誤検出を抑制
- 状態表示器を TM1637 7 セグから ZJY-IPS130-V2.0 液晶へ置き換え（launch 既定 `use_lcd_display=true` / `use_seg_display=false`。`gpio_pins.yaml`・BOM・README の配線を pin 16/17/18/19/20/22/23 の液晶接続に更新）
- `dashboard_node` の手動指令出力を `/cmd_vel` から `/cmd_vel_manual` に変更（`/cmd_vel` は `drive_mode_node` が発行）
- Motor HAT 接続図・一覧表を 1 モーター 6 本（Motor+ / Motor− / VCC / GND / Encoder A / Encoder B）すべて明示する形に変更（`/api/motor_hat` に `wires`）。エンコーダー VCC / GND は Motor HAT 上の 3V3 / GND ピンへ（`motor_hat.yaml` `hat.encoder.power_board`）
- OSOYOO 520 モーターの配線色をメーカー資料どおり（赤 Motor+ / 白 Motor− / 青 VCC / 黒 GND / 緑 Encoder A / 黄 Encoder B）に修正し、車輪表記を進行方向基準の「M1 前左」形式に統一
- OSOYOO 520 モーターエンコーダー（A/B 相）の Pi GPIO 直結割り付けを `motor_hat.yaml`（`encoder`）と `gpio_pins.yaml` に追加し、Motor HAT 接続図・一覧表・GPIO ヘッダー図に表示
- ダッシュボードに Motor HAT 接続図カードを追加（`config/motor_hat.yaml` で M1〜M4 と車輪位置を割り付け、`GET /api/motor_hat` で取得）
- TM1637 4桁7セグメントLED用 `seg_display_node` を追加し、ロボット状態コードを表示
- GY-BNO055 用 `imu_node` を追加し、I2C（0x29）から `/imu/data` を publish
- `gpio_pins.yaml` の BNO055 配線色を SDA 白 / SCL 灰へ更新
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
- LiDAR (`/scan`) の360度スキャンを点群マップ（Canvas）で表示し、LiDAR カードをカメラの次に配置
  - `dashboard_node`: LaserScan を直交座標の点群（最大720点に間引き）へ変換して `/api/status` の `scan.points` で配信
  - `dashboard.launch.py`: `rplidar_ros`（`rplidar_composition`）を `use_lidar` 引数で起動
  - `config/dashboard.yaml`: RPLIDAR のシリアルポート（by-id）・ボーレート・`frame_id` を追加
- 障害物判定ノード `perception_node` を新規作成（LiDAR 主・カメラ/AI HAT+ 補助）
  - `/scan` の前方 ±30°（`front_angle_deg`）を左/中央/右セクターに分けて最近距離を算出し、`stop_distance`（0.35m）/`slow_distance`（0.8m）で 停止 / 減速 / 安全 / 不明 を判定
  - `/camera/image_raw/compressed` を Hailo-8（`yolov8n.hef`、HailoRT 4.24 Python バインディング）で推論し、前方に写る物体名を補足情報として付与（推論約8ms）
  - サーマルガバナ: CPU 70℃ / AI HAT+ 75℃ で推論レートを 40% に低下、CPU 78℃ / AI HAT+ 85℃ で推論停止（LiDAR による安全判定は継続）
  - 判定結果を `/obstacle_status`（std_msgs/String, JSON）で 5Hz publish。距離・推論レート・温度閾値は実行中にパラメータ変更可能
  - `dashboard_node` が `/obstacle_status` を購読して `/api/status` の `obstacle` で配信し、前進指令に `speed_scale` を適用（`obstacle_guard`）
  - index.html に「障害物判定（LiDAR 主 / AI HAT+ 補助）」カードを追加
  - `dashboard.launch.py` に `use_perception` 引数を追加
  - 検出物体の距離を LiDAR と連動して算出（画像の横位置を `camera_hfov_deg`（66°）で方位角へ変換し、その角度範囲の最近距離を採用）
  - カメラ映像に検出枠と距離をオーバーレイ表示。`danger_distance`（0.3m）以内は赤枠、それ以外は緑枠
  - LiDAR 点群マップで `danger_distance` 以内かつ前方セクター（`front_angle_deg` 60°）内の点のみ赤点で強調
  - `scan_angle_offset_deg`（既定0°）で LiDAR 取り付け向きを補正可能にし、点群マップに前方／後方／左／右のラベルを表示
- CPU カードに入力電圧（`EXT5V`）・入力電流・電源状態を追加。入力電流は PMIC が EXT5V の電流を出さないため各レールの合計電力から換算する。`vcgencmd get_throttled` の低電圧 / スロットリング / クロック制限 / 温度制限をバッジで表示し、現在発生中は赤・起動後に発生した場合は黄色で警告する（電源不足による突然の電源断を事前に気付けるため）
- `ai-car-dashboard.service` の停止シグナルを SIGINT にし、起動前に `/dev/ttyUSB0` の解放を待つようにした（`systemctl restart` 時に前回の `rplidar_composition` がポートを掴んだままで LiDAR 初期化がタイムアウトし、スキャンが流れなくなるため）
- 速度指令 (/cmd_vel) とオドメトリ (/odom) のカードを削除し、AI HAT+ カードと IMU カードを同じ列に縦積み（AI HAT+ は内容分の高さのみ）に変更
- 動体のぶれ（残像）低減のため、カメラの露光を手動 8ms 固定（`ExposureTimeMode: 1` / `ExposureTime: 8000`）＋アナログゲイン自動（`AnalogueGainMode: 0`）に変更。室内では AE が露光を 1 フレーム分（33ms）まで伸ばすため。高ゲイン時のノイズ対策に `NoiseReductionMode: 1`（Fast）も有効化
- 障害物判定カードを廃止し、内容（レベル＋前方距離、検出物体、左/中/右、速度制限、推論時間＋サーマル）をカメラカードに統合。受信フレーム数と1枚あたりサイズは fps 行のツールチップへ移動
- カメラカードに解像度・fps（ROS 受信の実測 / MJPEG 配信上限）・1枚あたりの JPEG サイズを表示。あわせて映像枠のアスペクト比を実解像度に追従させ、16:9 化で生じていた検出枠の縦ずれを解消
- 映像を滑らかにするためカメラを 1280x720 / JPEG 品質 80 に変更し、MJPEG 配信を 10fps から 30fps へ引き上げた。配信ループは同間隔スリープだと位相ずれで実効レートが半減するため、短周期監視＋送信間隔保持に変更
- 検出枠の追従を滑らかにするため推論を 4Hz → 10Hz、テレメトリを 5Hz → 10Hz、ポーリングフォールバックを 1s → 250ms に変更
- AI HAT+ カードに推論レート（現在 / 連続実行時）と実効 TOPS の推定値を表示。`perception_node` が `obstacle.throughput` として配信し、`model_gops`（既定 8.7 = YOLOv8n 640x640）× 推論レートで換算する。Hailo は実効 TOPS を報告しないため推定値
- CPU ファンの作動温度を 45/50/55/62℃ へ前倒しする `scripts/set_fan_curve.sh` と `systemd/ai-car-fan.service` を追加
- CPU カードにファン回転数と段階（`cpu.fan`）を表示
- カード内の表を1行固定にし（`table-layout: fixed` ＋ 省略記号）、障害物判定カードの項目名・値を短縮。検出物体は先頭1件＋「他N」表示とし全件は title に保持
- CPU カードのロードアベレージ表示を削除し、代わりに PD 対応（5A）の 〇 / × 表示を追加。`vcgencmd get_config usb_max_current_enable` を `power.pd_5a` として配信する
- AI HAT+ カードから PCIe リンク使用率・リンク速度の表示を削除し、温度をカード最上部に移動
- ダッシュボードの CPU / AI HAT+ の温度を横バーグラフ表示に変更（0–100℃ スケール、70℃ で警告色、85℃ で危険色）
- AI HAT+ (Hailo-8) のオンチップ温度を HailoRT C API `hailo_get_chip_temperature()`（ctypes 直接呼び出し）で取得し、ダッシュボードの温度欄に表示（ts0/ts1 の平均）
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
- `systemd/ai-car-dashboard.service` と `systemd/install_service.sh` を追加し、ダッシュボードを起動時に自動起動（`Restart=always`、USB デバイス待ちで10秒遅延）
- README に Ubuntu 24.04 向け AI HAT+ セットアップ手順（hailort-drivers v4.24.0 と HailoRT 4.24.0 のソース導入）を追記
- カメラ映像をダッシュボードに追加
  - `dashboard_node` が `/camera/image_raw/compressed`（`sensor_msgs/CompressedImage`）を購読
  - `GET /api/camera/snapshot`（JPEG）と `GET /api/camera/stream`（MJPEG）を追加
  - index.html にカメラカード（映像・状態・受信フレーム数・最終受信時刻）を追加
  - `dashboard.launch.py` に `use_camera` 引数を追加し、`camera_ros` があればカメラノードを起動。`~/opt/rpicam` の Raspberry Pi 版 libcamera を `LD_LIBRARY_PATH` に自動追加
  - README に Ubuntu 24.04 向けカメラ（IMX708）セットアップ手順（Raspberry Pi 版 libcamera / libpisp のビルド）を追記

### Changed
- AI HAT+ カードを Raspberry Pi 5 — CPU カードと同じ構成（使用率バー＋4行テーブル＋詳細グリッド）に統一。PCIe リンク使用率（現在幅/最大幅）とリンク速度（現在/最大）を追加し、デバイス・FW・ドライバ・PCIe アドレスを下部グリッドに表示
- 表記を AI-CAR に統一（旧 AT-CAR）
- ダッシュボードの表示順を Raspberry Pi 5 — CPU、AI HAT+ — Hailo-8 NPU の順に変更
- ダッシュボードのカード順を カメラ → LiDAR → CPU → AI HAT+ に変更し、カメラと LiDAR のカードを横幅 2 列分（`grid-column: span 2`、620px 以下では 1 列）に拡大
- 全体消費電力を CPU カードの温度の下に移動し、電源 (PMIC) とメモリ / ストレージのカードを削除（`/system_status` の `memory` / `power` 収集自体は継続）

### Fixed
- LiDAR の取り付け向きに合わせて `scan_angle_offset_deg` を 180° に設定し、点群マップの前後反転と前方距離の誤りを解消
- カメラ検出物体の距離を、方位角範囲内の点を距離でクラスタリングして最も手前のまとまりの中央値から求めるようにし、単発の外れ点で極端に近い距離（例: 実距離約2.7mに対し0.17m）を表示しないようにした。スキャンが `scan_max_age`（1秒）より古い場合は距離を出さない
- カメラ / LiDAR ノードを `respawn` 対応にし、デバイス切断などで落ちても自動復帰するようにした
- Hailo の状態判定を `hailortcli` の有無ではなく `/dev/hailo0` の存在で行うように変更。AI HAT+ 非対応の温度取得呼び出しを削除
- WebSocket が利用できない環境（uvicorn に websockets/wsproto 未導入）では `/api/status` の1秒ポーリングへ自動フォールバックするようにした
