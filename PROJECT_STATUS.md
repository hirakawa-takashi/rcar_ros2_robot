# PROJECT_STATUS.md

## 最終更新
- バージョン: 1.10.0（2026-10-19）
- 更新日: 2026/10/17
- 更新概要: 機能の開発がひと区切りしたので、文書（README・PROJECT_STATUS・HARDWARE_BOM・CHANGELOG・プロジェクト説明 `architecture.yaml`）を今の main のプログラムと実機（ラズパイ・Jetson）に合わせて見直した。前後の落下防止センサー（VL53L1X、0x30 / 0x31）は配線済み・床の上で読める、IMU（0x29）はセンサーと取り合わずに動く、後ろ右のエンコーダーは止まっているとき・手で回したときに問題なし（2026/10/17、下の「テスト結果」）。これまでの変更の中身は CHANGELOG.md
- 更新担当: Devin

## システム構成
- 実行環境: Raspberry Pi 5
- ROS: ROS2 Jazzy（ros-base）
- ナビゲーション: Nav2（ros-jazzy-navigation2 1.3.13、Level 2。slam_toolbox の地図と自己位置で行き先へ）
- センサー: LiDAR（RPLIDAR A1M8）、カメラ（Camera Module 3 Wide、IMX708）、IMU（BNO055）、落下防止センサー（VL53L1X ×2、前後）、エンコーダー（520 モーター ×4）
- 駆動: Adafruit DC & Stepper Motor HAT（M1〜M4、メカナム 4 輪）
- 外の機械: Jetson Orin Nano Super（スタックちゃんの会話・状態）、パソコン（WSL・RTX 4080 SUPER、使えるときだけ聞き取り・会話・声）、M5 スタックちゃん（CoreS3）
- Web: FastAPI + uvicorn
- 開発環境: VS Code Remote-SSH
- AI: Cline + LM Studio + DeepSeek 32B Q4_K_M

## 実装済み機能
- ai_car_description
  - URDF/Xacro（`urdf/ai_car.xacro`）: base_footprint / base_link / メカナム4輪 / laser / camera_link / camera_optical_frame / imu_link（計10リンク）。laser と camera_link は 3D マウントの CAD の位置
  - Launch（`launch/view_robot.launch.py`）: robot_state_publisher、joint_state_publisher、use_gui / use_rviz / use_sim_time 引数
  - RViz2設定（`rviz/ai_car.rviz`）: RobotModel / TF / LaserScan
- ai_car_web
  - `dashboard_node`: FastAPI サーバー（別スレッド）+ rclpy ノード
  - 「プロジェクト説明」ボタンと 9 タブの全画面モーダルを追加（「3D プリント」タブは STL ダウンロード・三面図・立体図・ドラッグで回せる 3D 表示・印刷設定・組み立て手順）。`architecture.yaml` と `/api/architecture` から構成を表示し、`dashboard.yaml` の実値（秘密値はマスク）と実機の生存ノード / Topic を反映
  - Publish: `/cmd_vel_manual`（geometry_msgs/Twist、手動指令。`/cmd_vel` は `drive_mode_node` が発行）、`/drive_mode_request`、`~/status`
  - `drive_mode_node`（運転モード管理）: `/cmd_vel_manual`（Web + ゲームパッド）と `/cmd_vel_auto`（自律走行）のどちらか一方だけを `/cmd_vel` に中継する。モードは manual / auto / stop（起動時 manual）。状態は `/drive_mode`（JSON, transient local, 5Hz）、切替要求は `/drive_mode_request`（JSON `{mode: manual|auto|stop|toggle, source}`）。安全機構: 自動へ入る条件（`/obstacle_status` が新鮮で stop / unknown でない・ゲームパッド接続中・スティック中立・`/cmd_vel_auto` 生存）を満たさないと拒否（`last_reject` に理由）、自動指令が `auto_timeout` 0.5 秒途絶／ゲームパッド切断／障害物判定途絶で STOP、自動中に手動指令を検出すると手動へ復帰（`manual_override`）、モード遷移時はゼロ速度を publish、最終段で LiDAR の 360° を使い、動く向き（前後左右・ななめ）の車体の端から 0.25 m（`move_stop_gap`）以内に物があればその向きの動きを、その向きに回ると車体の箱に点が入る（1° ずつ回して `turn_center_m` のまわりに確かめる、回れる角度が 5° `turn_stop_deg` より小さい）ならその向きの回転だけを 0 にする。ななめの向きが止まっても、前後・横の片方が空いていればその分だけ動く。手動は 0.7 m（`move_slow_gap`）以内で半分の速さ。LiDAR データなし（unknown）・判定の途絶中は前後左右の動きを 0 にする（`obstacle_guard`、手動・ゲームパッド・声・自動のどれも。止めた理由は `/drive_mode` の `guard`）。すき間は `perception_node` が車体の箱 `body_box_m`（LiDAR の中心から 後ろ 0.176・前 0.090・右 0.135・左 0.190 m。左は LiDAR の左 17 cm の箱まで）を 5° ごとの向きへ動かして求め、`/obstacle_status` の `move` に出す。launch 引数 `use_drive_mode`
  - `autonomy_node`（Level 1 自律走行・LiDAR 反応型）: `/scan` を前 / 左 / 右 / 後セクターに分けて最短距離を求め、前進（0.15 m/s）→ 0.8 m 以内で減速（0.08 m/s、空いている側へ緩く曲がる）→ 0.35 m 以内で空いている側へその場旋回（0.6 rad/s、前方 0.9 m 以上空くまで）→ 前進を繰り返す。前後左右とも 0.25 m 以内なら後退。前の落下防止センサーが段差なら、`/odom` で 0.10 m 下がり、空いている側へ 120° 回って走行を続ける（前後とも段差なら止まる）。LiDAR が 1 秒以上途絶えると停止。`/cmd_vel_auto` に 10Hz で常時 publish（AUTO 以外はゼロ速度で待機）、状態は `/autonomy_status`（JSON: behavior / sectors / cmd）。地図・自己位置は使わない。launch 引数 `use_autonomy`
  - Level 2 自律走行（Nav2）: `nav2.launch.py`（`use_nav2` 既定 true、SLAM が active になる 40 秒後に起動）で `controller_server`（Regulated Pure Pursuit、0.15 m/s・旋回 0.6 rad/s）・`planner_server`（NavFn）・`behavior_server`（回る・下がる・待つ）・`bt_navigator` を `lifecycle_manager_navigation` が active にする。地図と自己位置は slam_toolbox。ダッシュボードの「地図と自己位置（SLAM）」で地図をクリックすると、`POST /api/nav` → `/nav_goal` → `autonomy_node` が `compute_path_to_pose` で道を考え、緑の線で出す（車は動かない）。「ここへ行く」で運転モードを自動にし、`navigate_to_pose` で向かう。Nav2 の速度は `/cmd_vel_nav` に出し、`autonomy_node` が AUTO のあいだだけ `/cmd_vel_auto` へ中継するので、`drive_mode_node` の 360° の見張り・段差の止めはそのまま効く。段差から離れる動きは行き先より先。着いた・行けない・やめたら止まって待ち、AUTO を抜ける（手動・停止）か 5 秒たっても自動に入らないと行き先をやめる
  - 運転モード UI: 手動操作カード上部にモードバッジ（手動 / 自動 / 停止）と 3 ボタン（`POST /api/drive_mode`）、拒否理由表示。自律走行カードに行動・各方向距離・自動指令・`/cmd_vel` 出力を表示
  - Subscribe: `/scan`、`/imu/data`、`/odom`、`/camera/image_raw/compressed`
  - `imu_node`: GY-BNO055（I2C バス1、アドレス0x29）を読み取り、`/imu/data`（sensor_msgs/Imu）を20Hzでpublish。パラメータは `i2c_bus` / `i2c_address` / `frame_id` / `imu_topic` / `publish_rate` / `axis_map_config` / `axis_map_sign`（BNO055 の軸の割り当て。USB スイッチの受けの上の台では基板が x 左・y 後ろなので `0x21` / `0x04` で imu_link にそろえる）・`operation_mode`（既定 `0x07` AMG。3 つのセンサーの生の値を読み、向きは imu_node がジャイロ＋加速度の Mahony フィルターで計算する（`filter_gain`、yaw は起動時からの相対）。磁気は `/imu/mag`（sensor_msgs/MagneticField）に出し、ダッシュボードの IMU のカードに x / y / z・強さ・向きを出す。`0x08` IMU は磁気が止まる。車の上は磁気の乱れが大きく NDOF では補正が進まない）。補正が 3 になったら補正値を `calibration_file`（`~/.ros/bno055_calibration.json`）に保存し、次につないだとき書きもどす
  - REST: `GET /api/status`、`POST /api/cmd_vel`、`POST /api/stop`、`GET /api/camera/snapshot`、`GET /api/camera/stream`（MJPEG）、WebSocket `/ws`（テレメトリ配信）
  - カメラカード: MJPEG 映像と受信フレーム数・最終受信時刻を表示（未受信時は待機表示）
  - LiDAR カード: `/scan` の360度スキャンを上面視の点群マップ（Canvas、最大720点（`scan_max_points`）、表示範囲は 3 m で固定、3 m より遠い点は描かない）として描画。カード順は カメラ → LiDAR → CPU → AI HAT+ で、カメラと LiDAR は横幅 2 列分（狭い画面では 1 列）
  - LiDAR ノード: `rplidar_ros`（`rplidar_composition`）を `dashboard.launch.py` の `use_lidar` で起動。ポートは by-id パス、115200bps、`frame_id: laser`。`lidar_watchdog_node` が `/scan` の 3 秒途絶で rplidar を止め、respawn で起動し直させる（USB が抜けてつながり直すと、rplidar は `/scan` を出さなくなるため）。2 回続けて戻らなければ LiDAR の USB をつなぎ直す（電池の電圧低下の後に CP2102 が返事をしなくなったとき用）。4 回で USB の電気を 3 秒切って入れ直し（`ai-car-usb-power-cycle`、uhubctl、ゲームパッドも数秒切れる）、7 回でラズパイを再起動する（この起動で `/scan` が来ていて、起動から 30 分たっているときだけ）
  - カメラノード: `dashboard.launch.py` の `use_camera` で Camera Module 3 Wide の `camera_ros`（libcamera）を起動。`~/opt/rpicam` の Raspberry Pi 版 libcamera を `LD_LIBRARY_PATH` に自動追加
  - 障害物判定ノード（`perception_node`）: LiDAR を主として前方 ±30° を左/中央/右セクターで評価し、停止 0.35m / 減速 0.8m で 停止・減速・安全・不明 を判定。カメラ画像は AI HAT+（Hailo-8）で 1 眼の奥行きの AI（SC-Depth v3）だけを動かし、床から出っぱった低い物を表示する（2026/10/09 から物の名前を当てる YOLO `yolov8m.hef` は `hef_path: ""` で止めている。`hef_path` に入れれば、検出枠を履歴3中2回で確認して物体名を補助情報として付ける）。結果は `/obstacle_status`（JSON, 5Hz）
  - 検出物体の距離: LiDAR の距離（画面に「（LiDAR）」）。LiDAR は床から 17.5 cm の 1 本の線なので、高い所の物（テレビなど）では手前の低い物の距離になることがある。LiDAR の距離: 画像の横位置を `camera_hfov_deg`（102°）で方位角に変換し、LiDAR の同方位の距離を採用（LiDAR の点は、LiDAR の中心から見たカメラの位置 `camera_offset_x_m` 0.063 / `camera_offset_y_m` 0.0（真ん前 63 mm。Camera Module 3 Wide のレンズの先、CAD 値）に移し、カメラから見た方位で照合する。距離はカメラのレンズから前方への距離）。カメラ映像に検出枠と距離を重畳し（枠の色は LiDAR の図の点と同じ距離の色: `danger_distance` 以内 赤 #ff5c6c / 停止距離以内 橙 #ff8a4c / 減速距離以内 黄 #f5b942 / それより遠い 水色 #35d6ff、距離不明 灰色。1 眼の低い物の枠も同じ）、LiDAR 点群マップでは前方 ±30°（`front_angle_deg` 60°）内かつ 0.3m 以内の点を赤点で表示。点群マップには前方／後方／左／右のラベルを表示（上＝前方）。方位角範囲内の点は距離でクラスタリングし（`cluster_gap` 0.25m）、最も手前のまとまりの中央値を採用する。スキャンが `scan_max_age`（1秒）より古い場合は距離を出さない。LiDAR の取り付け向きは `scan_angle_offset_deg` で補正する。画面の「前方」は前方カメラの正面とし、カメラの画面の真ん中に置いたボトルが LiDAR で右 4°・0.47 m に見えたことから −1.7° とした（2026/10 実測。`dashboard_node`・`perception_node`・`autonomy_node` と URDF の `laser` の向きで同じ値）
  - 人・動物の早めの減速・停止: カメラの前方（画面の中央半分）に写った人・犬・猫・鳥（`living_labels`）の LiDAR 距離が 1.5 m（`living_slow_distance`）以内で減速、0.6 m（`living_stop_distance`）以内で停止と判定し、`/obstacle_status` の `living` に出す。`living_guard: true` のときだけ LiDAR の判定より厳しい方を `level`・`speed_scale` に使う（既定は false で表示のみ）
  - 1 眼の距離: `perception_node` が AI HAT+ の 1 眼の奥行きの AI（SC-Depth v3、`mono_depth_hef`）で床から出っぱった低い物を見つけ、`/stereo/depth_status`（いちばん近い物・`objects`）と `/stereo/depth/compressed`（距離の色の画像）に出す（トピック名は前の 2 眼カメラのときのまま）。ダッシュボードのカメラカードの「1眼の距離」の行に出し、「距離の画像」ボタンで映像を距離の色に切り替える（`GET /api/camera/depth_stream`）。減速・停止に使うのは `stereo_guard` と `mono_depth_guard` の両方が true のときだけ（今は `mono_depth_guard: false` で表示だけ）。車の傾きは、LiDAR が当たった物と床のさかい目の行から毎回出して距離の計算に使う（`mono_auto_pitch`、出せないときは `mono_camera_pitch_deg`）
  - システム構成（プロジェクト説明の「システム構成」タブ、`architecture.yaml` の `system`）: AI-CAR・Jetson Orin Nano Super・M5 スタックちゃんの役割とやりとり（① 画像 ② 名前・位置・スタックちゃんとの声・スタックちゃんのカメラの映像・声の命令（話して走る）はどれも稼働中）を図と表で出す
  - 液晶（ZJY-IPS130-V2.0、ST7789）は 2026/10 に取り外し、`lcd_display_node`・launch の `use_lcd_display`・`dashboard.yaml` の設定・`gpio_pins.yaml` の配線（pin 16/17/18/19/20/22/23）を消した。旧 TM1637 の `seg_display_node` は残っている（既定では起動しない）
  - Jetson の物体検出（表示のみ）: Jetson Orin Nano Super の `jetson/owl_detector`（NanoOWL、`jetson-owl.service`）が AI-CAR のカメラ画像から `prompts.json` の物（段ボール箱・袋など）を探し、`POST /api/jetson/detections` → `/jetson_detections` → `perception_node` が LiDAR の距離を付けて `/obstacle_status` の `jetson` に出す。2 秒以上届かないと「未接続」。速度制限には使わない。LiDAR の図にもカメラの見える範囲（扇形）と物の位置を点で重ねる（名前・距離の吹き出しは出さない）。2 眼の物（`/obstacle_status.stereo.objects`）も、色を距離、直径を横幅の 2 倍にした白ふちの点で描く
  - Jetson の状態（表示のみ）: Jetson の `jetson/status_reporter`（`jetson-status.service`）が 1 秒ごとに温度・CPU / GPU の使用率・メモリ・スワップ・消費電力・AI の部品（whisper-server・Ollama・Kokoro。声→文字・文字→声は仕組みの名前（例: whisper.cpp small（GPU）・Kokoro TTS 1.0.0）、会話の AI はモデル名と GPU / CPU で動く割合も）・セキュリティ更新の残り・再起動が必要か・IP アドレスを `POST /api/jetson/status` へ送り、ダッシュボードの「Jetson Orin Nano Super — AI 担当」カード（「Raspberry Pi 5 — CPU」の右）に出す。5 秒以上届かないと「未接続」。速度や安全の判定には使わない。NanoOWL は 2026/10 からメモリを空けるため止めている（電源モード・ファン・NanoOWL の値も送るが、カードには出さない）
  - スタックちゃんのカード（表示のみ）: ダッシュボードの「M5 スタックちゃん（CoreS3）」カード（Jetson カードの下）。Jetson の `stackchan_bridge` の `GET /status`（8003）を `GET /api/stackchan/status`（API トークン）で 2 秒ごとに取って、今の様子（聞いている・呼びかけ待ち・考えている・話している・待ち受け）に合わせた顔、最後の会話（聞き取り・返事・かかった時間）、IP アドレス・ファームウェアの版・会話の数・使っている仕組みを出す。写真（320×240）は、スタックちゃんに「撮影して」と言って撮ったものを `GET /api/stackchan/photo.jpg` で出す（カードには撮るボタンを置かない）。行き先は `stackchan_bridge_url`、空なら Jetson が送ってくる IP アドレス（Tailscale を先に）の 8003。ネットのときもローカルのときも、つながったら「呼びかけ待ち」で、「スタックちゃん」と呼ぶと会話モードになり、最後に話してから 30 秒だまると「じゃあ、またね。」と言って呼びかけ待ちにもどる（`jetson/stackchan_bridge/README.md`「呼びかけ」）。テレビの声で起きないように、呼びかけは文の頭で名前を呼んだときだけ（whisper の音の注釈・文の途中・名前の話題・`--prompt` の返しはとらない）。走行・安全には使わない
  - スタックちゃんの会話: 純正ファームウェア（1.5.1）の AI Agent を、Jetson の `jetson/stackchan_bridge`（`jetson-stackchan.service`、OTA 8003・WebSocket 8000）経由で whisper-server・Ollama qwen2.5:3b・Kokoro jf_alpha につなぐ。パソコン（WSL の Ubuntu、RTX 4080 SUPER、Tailscale の taka3-wsl）が動いているあいだは、聞き取りを whisper large-v3、返事を qwen2.5:14b でパソコンにさせ、止まっていれば Jetson にもどす（`jetson/stackchan_bridge/README.md`「パソコンで聞き取りと返事」）。ダッシュボードの Jetson のカードの「動かす所」（Jetson / MyPC）で、どちらでするかを選べる（`POST /api/stackchan/backend`）。ファームウェアは行き先（`CONFIG_OTA_URL`）と自動更新をやめるパッチだけを変える。純正の全部（16 MB）は Jetson の `~/stackchan_backup/` にある。走行・安全には使わない
  - Jetson の更新ボタン: Jetson カードの「Jetson を更新」（確認の画面あり、API トークンが必要）。次の状態の返事で伝え、`jetson-upgrade.service`（`jetson-upgrade.sh`）が NVIDIA の部品（JetPack）以外を更新する。試しに動かして NVIDIA の部品が変わるか消える部品があるときはやめる。「全部」の数も NVIDIA の部品を除いた数
  - 再起動ボタン: ラズパイと Jetson のカードの「再起動」ボタン（確認の画面あり、API トークンが必要）。ラズパイは走行中・自動運転中は断り、押してから 10 秒後に再起動する（そのあいだに押した Jetson の再起動も受け付け、伝え終わってから再起動する）。Jetson は次の状態の返事で伝え、`jetson_status.py` が再起動する。「電源を切る」ボタンも同じしくみ（`POST /api/system/poweroff`・`/api/jetson/poweroff`、ダッシュボードからは入れ直せないので、本体の電源ボタンか電源のつなぎ直しで入れる）。どちらも `systemctl reboot` と `systemctl poweroff` だけをパスワードなしで許す sudoers が要る
  - つながりの印: カメラ・LiDAR・Raspberry Pi 5 — CPU・AI HAT+・IMU・落下防止センサー・Jetson のカードのタイトルの横に「接続中」などを出す（LiDAR・IMU は 2 秒、`/system_status` は 3 秒、Jetson は 5 秒届かないと途絶 / 未接続）
  - 更新の進み具合: ボタンの更新のあいだ「更新中 45 %」のように出す（ダウンロード 0〜30 %、インストール 30〜100 %。`/run/ai-car-upgrade.progress`・`/run/jetson-upgrade.progress`）
  - ラズパイの更新: 「Raspberry Pi 5 — CPU」カードに「セキュリティ更新」（`apt-check`、1 時間に 1 回とボタンの更新のあと）「再起動」（必要 / いらない）と「ラズパイを更新」ボタン（確認の画面あり、API トークンが必要、走行中・自動運転中は断る）。ボタンは `ai-car-upgrade.service`（`systemd/ai-car-upgrade.sh`、ROS 2 も含めて更新、消える部品があるときはやめる）を始める。自動のセキュリティ更新は Ubuntu の unattended-upgrades（自動の再起動なし）
  - SLAM（地図と自己位置）: `use_slam`（既定 true）で `slam_toolbox` を起動し、ダッシュボードの「地図と自己位置（SLAM）」に地図と自己位置を出す。地図は `~/AI-CAR_ws/maps` に保存できる。位置はエンコーダーの `/odom` とスキャンの照合で出し、0.1 m か 0.1 rad 動くたびにスキャンを地図に足す（`minimum_travel_*`）。止まっていても `/map` は 2 秒ごとに届くので地図は出たまま。自律走行にはまだ使わない
  - サーマル制御: CPU 70℃ / AI HAT+ 75℃ で推論レート 40%、CPU 78℃ / AI HAT+ 85℃ で推論停止（LiDAR 判定は継続）。減速・停止は `/obstacle_status` を通して `drive_mode_node` の `obstacle_guard` が最後に `/cmd_vel` にかける
  - 画面（`static/index.html`）: 手動操作カードは LiDAR カードの直下（4 列固定グリッド、幅 2 列分）に配置。メカナム方向操作（前後・平行移動・旋回）、出力ゲイン、テレメトリ表示
  - 安全機構: `cmd_timeout`（既定0.7秒）無指令で自動停止
  - ゲームパッド手動操作（`joy_teleop_node`）: Logitech F710（XInput モード、`/dev/input/js0`）を Linux joystick API で直接読み（追加の Python 依存なし）、正規化済み Twist を `/joy_cmd` に 10Hz で publish。`dashboard_node` が `/joy_cmd` を購読して Web 操作と同じ `publish_cmd_vel()` 経路で `/cmd_vel_manual` に出し、`drive_mode_node` が `/cmd_vel` にするため、最大速度×「出力」・360° の見張り・段差ガード・`cmd_timeout` 停止が同じようにかかる。割り当て: 左スティック上下=前後、左スティック左右=平行移動、右スティック左右=旋回、十字キー=前後・平行移動（デジタル）、RB=全速（`speed_scale` 1.0 = ダッシュボードの「出力」）、B=停止（押している間。自動中は STOP モード要求）、START 長押し 2 秒=手動 ⇄ 自動、BACK=STOP モード、`button_enable` でデッドマンボタンも設定可（既定は無効）。F710 未接続でも起動し、抜き差しを検出して再接続（切断時は指令を出さない）。画面の手動操作カードに接続状態（未接続／接続中／操作中）を表示。launch 引数 `use_joy` で無効化可
  - `motor_driver_node`（モーター制御）: `/cmd_vel` をメカナムの逆運動学で 4 輪に分け、Motor HAT（PCA9685 0x60）の M1〜M4 を PWM で回す（右側 2 輪は `reversed`）。`/cmd_vel` が `timeout` 届かないと止める。旋回 0 で動くあいだは IMU の向きを保つ（`heading_hold`、床で合わせた `heading_kp` 0.25・`heading_kd` 0.05・上限 0.05 rad/s・1° の遊び）。様子は `/motor_status`（JSON 5 Hz）
  - `encoder_odom_node`（オドメトリ）: 4 輪の A/B 相 8 本を GPIO v2 uAPI でまとめて両エッジで数え（別プロセス、イベントの取りこぼしと相のとび越しを数えて 10 秒ごとに記録）、メカナムの順運動学で `/odom` と TF `odom → base_footprint` を 20 Hz で出す（`counts_per_rev` 1408・`yaw_scale` 1.42、床で合わせた値）
  - `cliff_node`（落下防止）: 前後バンパーの VL53L1X ×2 を XSHUT（前 GPIO17・後 GPIO27）で 1 個ずつ起こして 0x30 / 0x31 にし、`/cliff_status` を 20 Hz で出す。測った距離を床を 0 cm とした高さに直し、2.0 cm 以上低い（`cliff_height_cm`）と段差。ダッシュボードは高さを cm で出す。`front_wired` / `rear_wired` は両方 true（2026/10/17 配線済み）。前が段差なら前進、後ろなら後退を `drive_mode_node` が 0 にし、自動は STOP
  - スタックちゃんに話して走る: Jetson の `stackchan_bridge` が「前に 1 メートル進んで」「右に 90 度回って」「止まって」を聞き分け、ダッシュボードの `POST /api/cmd_vel` / `POST /api/stop` で動かす（言わなければ 0.5 m・90°。1 回の上限は 2 m・720°・10 秒。進んだ量は `/odom`、回った角度は IMU で見て、のこりが少ないと半分の速さにし、3 秒動かなければ止める）。ラズパイの安全ガード（360° の見張り・段差・指令の途絶）はそのまま効く
  - `seg_display_node`（旧表示器、既定では起動しない）: TM1637 4桁7セグメントLEDを libgpiod で駆動し、`/system_status`・`/obstacle_status`・`/joy_cmd`・`/cmd_vel` と `/drive_mode` から状態を判定して `/display_state`（std_msgs/String）へ publish。表示: boot / rdy / HAnd / AUto / StoP / ObSt / SLo / LoU / Err。`use_seg_display` で無効化可
  - パラメータ: `config/dashboard.yaml`（host/port/各トピック名/最大速度/タイムアウト/配信レート）
  - 自動起動: `systemd/ai-car-dashboard.service`（`install_service.sh` で登録、`Restart=always`、USB デバイス待ちで起動を10秒遅延）。カメラ / LiDAR / IMU ノードは launch 側で `respawn`
  - `system_monitor_node`: Raspberry Pi 5 / AI HAT+ の状態監視（読み取りのみ）
    - Publish: `/system_status`（std_msgs/String, JSON、1Hz。IP アドレス `ips` は 30 秒ごとに調べる）
    - 取得内容: CPU使用率（全体・コア別）・クロック・温度・ロードアベレージ、メモリ/Swap/ディスク、PMIC レール別電圧・電流・電力、スロットリング状態、Hailo-8 検出状態
    - パラメータ: `publish_rate` / `topic` / `enable_pmic` / `enable_hailo`
    - 依存: `psutil`、`vcgencmd`、`lspci`、`hailortcli`（あれば利用）
  - GPIO 40 ピンヘッダーカード（プロジェクト説明「ハードウェア構成」タブ内「配線詳細」に表示）: `ai_car_web/gpio_pinout.py` の固定ピン定義（物理番号・名称・BCM・種別）と `config/gpio_pins.yaml` の配線定義（`pins` / `device` / `signal` / `color` / `note`）を結合し `GET /api/gpio` で返す。画面は SVG のヘッダー図（種別ごとの色、使用中ピンは緑の外枠、配線色バー、ホバーで接続先表示）と一覧表（ピン / 名称 / BCM / 使用 / 配線色 / 接続先 / 信号・備考）。同一ピンへの複数接続（I2C バス共有）は「/」区切りで併記。パラメータ `gpio_config`（空なら share 内の `gpio_pins.yaml`）。YAML 編集後は `dashboard_node` 再起動で反映
  - Motor HAT 接続図カード（同じく「配線詳細」に表示）: `ai_car_web/motor_hat.py` が `config/motor_hat.yaml` の割り付け（`channel` / `wheel` / `reversed` / `colors` / `note`）に端子ごとの固定情報（PCA9685 の PWM / IN1 / IN2 チャンネル、TB6612 ブリッジ）を重ねて `GET /api/motor_hat` で返す。割り付け: M1=前左（front_left）/ M2=前右（front_right）/ M3=後左（rear_left）/ M4=後右（rear_right）。右側 2 輪は左右対称取り付けのため `reversed: true`（モーター制御ノードで符号反転する前提）。画面は上面図の SVG（前方が上、中央に HAT の端子台 M1 M2 | +− | M3 M4、四隅に車輪、端子→車輪の配線を +/− の配線色で描画、電源端子→12V 系統）と一覧表（端子 / 車輪 / 回転 / 配線色 / PCA9685 ch / ブリッジ / 備考）。端子・車輪の重複や未割り付けは警告表示。各モーターの `encoder`（`a_pin` / `b_pin` / `colors`）と `hat.encoder`（`vcc_pin` / `gnd_pins`）で 520 モーターエンコーダーの Pi 直結ピンを定義し、BCM 番号へ解決・非 GPIO ピン・I2C/ID ピン・ピン重複・VCC が 3V3 以外を警告。接続図は車輪ごとに「M1 前左」＋ 6 本の線（名称・配線色・接続先）を列挙し、一覧表は 1 線 1 行（モーター / 線番号 / ピン名称 / 配線色 / 接続先 / 回転・ドライバ・備考）。パラメータ `motor_hat_config`（空なら share 内の `motor_hat.yaml`）

## 現在作業中
- なし（2026/10/17 に Level 2 の Nav2 を入れた。のこりは下の「次回作業」の床での確かめ）

## 未実装機能
- なし（Level 2 の Nav2 は入れた。床ではまだ走らせていない）

## 既知の問題
- Pi には rviz2 が未インストール（ros-base のみ）。RViz2 は開発PC側での表示を想定
- `/odom`（encoder_odom_node）の距離は床で合わせた値（counts_per_rev 1408、実測 1.8 m ≒ 約 10,000 カウント）。旋回は IMU の yaw の変わった分（`yaw_source: imu`。IMU が使えないときだけ車輪から出し、yaw_scale 1.42（床で 1.5 回転して合わせた値）をかける）。横移動はまだ寸法からの計算値
- LiDAR は `ros-jazzy-rplidar-ros` の導入と `super` の `dialout` グループ所属が必要
- AI HAT+ は `hailo_pci` 4.24.0 と HailoRT 4.24.0 をソース導入済み（Ubuntu 24.04 に `hailo-all` は存在しない。手順は README 参照）。温度は HailoRT C API `hailo_get_chip_temperature()` で取得済み。NPU 使用率と電力はダッシュボードでは扱わない（使用率は `HAILO_MONITOR=1` の推論アプリがある間のみ `hailortcli monitor` で参照可）。電力は未取得（`measure-power` は `UNSUPPORTED_OPCODE`、`query_health_stats()`/`query_performance_stats()` は HAILO8 非対応、`hatctl` は Ubuntu に存在せず Hailo 専用 hwmon も無し、PMIC に HAT 専用レール無し。外付け INA219 等が必要）
- 検出物体の距離は LiDAR の方位角対応による概算で、カメラとLiDARの外部キャリブレーションは未実施（設置が同軸・同方向である前提）
- `perception_node` の推論には HailoRT の Python バインディング（`hailo_platform`）が必要。apt には無いため HailoRT 4.24.0 のソースからビルドして導入する（手順は README 参照）。未導入の場合は LiDAR 判定のみで動作し、`/obstacle_status` の `note` に理由を表示する
- 電源は PD なし（バッテリー PD 12V → DROK 5.2V/5A → USB-A → ラズパイの USB-C。USB-A には PD のやりとりがない）。ラズパイは 3 A の電源と見ている（device tree `chosen/power/max_current` 3000）。`usb_max_current_enable=1` で USB に 1.6 A まで流す。2026/09 の電圧低下・電源断（USB 2 眼カメラのころ）は、Camera Module 3 Wide にしてからは出ていない（2026/10 の 3 日間で `Undervoltage detected` 0 回、`get_throttled=0x0`、EXT5V 4.83V）
- カメラは Ubuntu 標準の libcamera 0.7.2 だと raspi カーネル 6.8 のエンティティ名不一致で `no cameras available` となる。`~/opt/rpicam` の Raspberry Pi 版 libcamera を使う必要がある（手順は README 参照）
- モーター M1〜M4・エンコーダーの割り付けと配線色（`motor_hat.yaml` / `gpio_pins.yaml`）は、Motor HAT 交換のとき実機で確かめた（下の「テスト結果」）。TM1637 の配線色は実配線を反映済み（オレンジ・黒・黄・緑）。AI HAT+ の ID EEPROM ピン（27/28）はヘッダー直挿しのため配線色は `PIN` 表記
- WebSocket は `pip install --user --break-system-packages "websockets>=13"` で有効化済み（apt の python3-websockets 10.4 は uvicorn が要求する `ServerProtocol` を持たず、入れると dashboard_node が ImportError で起動しない）。未導入環境では `/api/status` の 250ms ポーリングへ自動フォールバックする
- 落下防止センサー（VL53L1X ×2）は前後とも配線済み（0x30 / 0x31 で距離が読める）。段差の前で本当に止まるかは、床でまだ確かめていない
- LiDAR は床から約 17.5 cm の高さだけを見る。それより低い物・細い物は 360° の見張りで止まらないことがある（1 眼の奥行きの低い物は表示だけ）
- IMU（BNO055、AMG モード）の加速度の補正は 0 のまま（ジャイロ 3）。傾きのずれは 1〜2°。磁気は LiDAR の下で 110〜156 µT（地球の約 3 倍）あり、磁気の向きは目安だけ
- Jetson のケースの OLED（I2C）は、ときどき書きこみがタイムアウトする（2026/10/17 は 24 時間で 31 回）。画面が 1 回更新されないだけで、ほかのサービスには影響しない
- Motor HAT（Adafruit DC & Stepper Motor HAT）の出力故障（2026/09/25）は、同型品（Adafruit 2348）への交換で解決した（2026/10/12、下の「テスト結果」）

## テスト結果
- 落下防止センサー・IMU（実機、2026/10/17、車は床の上で止めたまま）: 前後の VL53L1X は XSHUT で 0x30（前）・0x31（後ろ）になり、距離は前 0.069 m・後ろ 0.085 m でどちらも「床あり」。正規の位置に固定したあと（同じ日）は前 0.086〜0.090 m・後ろ 0.084〜0.085 m、付け直したあと（同じ日）は前 0.087 m・後ろ 0.082 m（20 秒の中央。これを床 0 cm の基準にした）（平らな床の目安 0.095 m、段差は 0.15 m より遠いとき）。IMU（0x29）は前のセンサーと取り合わずに動く（前を `front_wired: false` のままつないだときは、前のセンサーが 0x29 で起きて IMU が読めなくなった → PR #225 で直した）。段差の手前で止まるかは床でまだ試していない
- 後ろ右のエンコーダー（実機、2026/10/17、モーターは回さない）: 止まっているとき 1 分で `/odom` は 0.0 mm（1 カウントのさかい目で +1 / −1 するだけ）。手でゆっくり 1,905 カウント回して数えそこね 0 回、強くはじいて約 3,890 カウントで数えそこね 9 回・取りこぼし 3 回（約 0.3 %、1 m で約 3 mm）。ふだんの走行では問題なしと判断
- ラズパイ・Jetson の記録（2026/10/17）: ラズパイは電圧低下 0 回（throttled 0x0）、温度 56.5 °C、止まったサービスなし。Jetson は 2 日 12 時間動き続け、6 サービス（会話・顔の見分け・状態・OLED・ollama・whisper）とも自動の再起動 0 回、メモリ不足 0 回、温度 45 °C。ケースの OLED は 24 時間で 31 回書きこみに失敗（I2C のタイムアウト、画面が 1 回更新されないだけ）
- Motor HAT 交換後（実機、2026/10/12）: 12V 投入で緑 LED 点灯、I2C 0x60 / 0x70 を検出（最初は Pi 側の線のゆるみで未検出 → 差し直しで検出）。タイヤを浮かせ、M1〜M4 を 1 台ずつ PWM 30% で 1 秒駆動（PCA9685 を i2cset で直接設定）。正転指令で M1 前左・M3 後左は前進、M2 前右・M4 後右は後退（`reversed` どおり）。エンコーダーは 1 秒で -2,580〜-2,800 カウント（約 1.2〜1.3 回転、4 台とも負で同じ向き）、`get_throttled=0x0`。M2・M4 は最初 Motor+/− が逆で、入れ替えて合わせた（M 端子の左から M1・M3 は 黒 / 赤、M2・M4 は 赤 / 黒）。M3 の Encoder A が 37 番ではなく 36 番に挿さっていたのを差し直した
- `lcd_display_node`（実機、2026/09/15）: `python3-spidev` / `fonts-ipafont-gothic` を apt 導入、`/dev/spidev0.0` は dialout グループで super から書き込み可。サービス再起動後 `lcd_display_node` が起動し「ST7789 液晶表示を接続しました」、`/display_state` に `rdy ` を publish。`seg_display_node` は起動していないことを確認。2026/09/16 に配線（GND 黒 / VCC 赤 / SCL 橙 / SDA 黄 / RES 緑 / DC 青 / BLK 紫）後、ノード再起動で **正常表示を確認**（SPI モード 3、32MHz、rotation 270 = 反時計回り 90°、横 80px オフセット）。2026/09/17 に更に反時計回り 90° 回転させ rotation 180（縦 80px オフセット）に変更し、実機で **要望通りの向きで正常表示を確認**。配線後はパネル初期化のためノード再起動が必要（起動中に配線しても表示されない）
- `seg_display_node`（実機、2026/09/15、旧表示器）: `python3-libgpiod` を apt 導入後、`gpiochip4 (pinctrl-rp1)` の GPIO23/24 を `seg_display_node` が output で確保（`gpioinfo` で確認）。起動 1 秒で `boot` → `/system_status` 受信後 `rdy` に遷移、`/display_state` を publish。TM1637 配線後に LED の点灯を確認。初回は CLK 線の挿し違いで消灯だった（GPIO24 にはモジュールのプルアップを検出、GPIO23 にはなし → CLK 未接続と判定）。スティック操作時の `HAnd` 表示は未確認
- `imu_node`（実機、2026/09/15）: `i2cdetect -y 1` で 0x29 を検出、`BNO055 接続` ログ後に `/imu/data` を 50.0Hz で受信。加速度の合成値 約9.8m/s²、キャリブレーション gyr=3 まで進行を確認。`/api/status` の `imu` に roll/pitch/yaw・角速度・加速度が入ることを確認（ブラウザ表示は未確認）。`super` を `i2c` グループへ追加が必要（未所属だと Permission denied）
- 取り付け向き（2026/09/15 の値）: 重力が z 軸に乗っていなかったので、PR #200 で BNO055 の軸の割り当て（x・y 反転）を `imu_link`（x 前・y 左）にそろえた
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
- `ros2 launch ai_car_web dashboard.launch.py`（カメラ含む）: `/camera/image_raw/compressed` 30Hz、`/api/status` の `camera.available: true`、`GET /api/camera/snapshot` → 200（約53KB JPEG）を確認
- LiDAR（RPLIDAR, CP2102 USB）: `rplidar_composition` 起動で `/scan` を 約8Hz で受信。ダッシュボードの点群マップに最大720点（有効点は実測約477点）が描画されることをブラウザで確認
- 障害物判定: `/obstacle_status` で `level: stop`（前方 0.177m）を確認。Hailo-8 推論は約 8ms、YOLOv8n で物体検出（例: bed 0.61 / sink 0.50）を確認。温度閾値を一時的に下げて warn（推論 40%）→ critical（推論停止、LiDAR 判定は継続）→ 復帰を確認
- F710 手動操作: 実機で `joy_teleop_node` が `Logitech Gamepad F710 (/dev/input/js0)` を検出し `/joy_cmd` 10Hz を確認。`/joy_cmd {x:0.5, y:-0.5, z:0.5}` を publish → `/cmd_vel {x:0.075, y:-0.15, z:0.5}`（最大速度 0.3/1.0 と前方障害物による減速 0.5 が適用）を確認。入力停止 → 0.7 秒後に「指令タイムアウトのため停止しました」を確認。`/api/status` の `joy.connected / active` を確認。スティックの実操作による前後・左右の向きは未確認（ユーザーによる実機確認が必要）
- Motor HAT 接続図: `load_motor_hat('config/motor_hat.yaml')` で 4 端子の割り付けと警告なし、YAML 不在時は `error` を返すことを確認。開発 PC 上で `/api/motor_hat` を模擬した静的サーバーにより、ブラウザで接続図（4 輪・配線・端子台）と一覧表の表示を確認。実機（AI-CAR）で `GET /api/motor_hat` が 200 で割り付けを返すことと、I2C 0x60 に Motor HAT を検出することを確認。エンコーダー割り付けは `load_motor_hat` で警告なし・BCM 解決を確認し、`build_pinout` で信号ピンの競合なし（使用 25/40）を確認。開発 PC のブラウザでエンコーダーピン付きの接続図・一覧表を確認（実機は未反映）。`flake8 --max-line-length 100` エラー無し
- GPIO ヘッダー: `build_pinout('config/gpio_pins.yaml')` で使用中 9 ピン（1,2,3,4,5,6,9,27,28）、YAML 不在時は `error` を返すことを確認。実機で `GET /api/gpio` → `used_count: 9`、ブラウザでヘッダー図（上下段ラベルの重なり無し）と 40 行の一覧表の表示を確認。`flake8 --max-line-length 100` エラー無し
- `ros2 launch ai_car_description view_robot.launch.py`: 起動成功（`/robot_description`・`/joint_states`・`/tf` 発行を確認）

## カメラ仕様（実測）
- 3D プリント部品（PR #80、未印刷）: Pi 5 は台の左に寄せ（基板の中心 X 47 mm）、カバーの左の穴から電源ボタンを押せる。右の USB / LAN 側はカバーの壁まで約 61 mm あき、ストレートの USB プラグも入る。印刷後に、ボタンの位置が穴に合うかとプラグが当たらないかを確かめる
- 前方カメラは Camera Module 3 Wide（IMX708、`camera_ros`）。横の画角 102°（公称）、レンズの先は LiDAR の回転中心の真ん前 63 mm。960×540・30.1 fps を実測
- 2026/10/01〜10/08 に使った USB 2 眼カメラ（ELP 3D USB Camera）は、今後使わない。2 眼のノード（`stereo_camera_node`・`stereo_depth_node`・`stereo_calibrate`）、キャリブレーション（`docs/calib`）、`launch` の `camera_type`、3D の 2 眼用の板・カバーは消した（くわしくは CHANGELOG）
- 解像度: 960x540 / JPEG 品質 80（IMX708 / Camera Module v3、`config/dashboard.yaml` の `width`/`height`/`jpeg_quality`）。1 枚 約42〜53KB
- ROS 配信レート: 約 30Hz（`/camera/image_raw/compressed`、`FrameDurationLimits` [33333, 33333]、実測 30.04Hz）
- ダッシュボード MJPEG: 30fps（`camera_stream_rate`）
- AI HAT+ 推論（2026/09 の YOLOv8m のときの値。今は `hef_path` が空で YOLO は止め、1 眼の奥行きの AI SC-Depth v3 を 15 Hz で動かす）: YOLOv8m、640x640 にレターボックスして 15Hz 設定（推論タイマー 0.02秒）。推論 約32.6ms、dashboard CPU 28〜33%、CPU 53.8〜54.3℃、Hailo 49.6〜50.7℃、サーマル状態 normal
- 電源: 12V 系 → DROK 降圧コンバータ 5.2V/5A → USB-C。`/boot/firmware/config.txt` に `usb_max_current_enable=1`（USB-A からなので PD のやりとりはなく、ラズパイが認識する電源の上限は 3 A。ダッシュボードの「電源の電流上限」は `power.supply` = device tree の `chosen/power/max_current`）。実測 入力電圧 4.96〜5.02V、`get_throttled=0x0`（低電圧履歴なし）。以前のモバイルバッテリー（CIO SMARTCOBY Pro SLIM 35W、5V 時 3A 上限）では 4.77〜4.90V・低電圧フラグが発生していた
- 低負荷設定（カメラ 15fps・推論 5Hz・テレメトリ 5Hz・LiDAR 360 点）時の参考値: dashboard CPU 18.4〜21.1%、camera_node 約18.1%、perception_node 約16.5%、dashboard_node 約16.8%

## 次回作業
- 360° の見張りを床で確かめる（出力 15 %。前後・左右・ななめ・回転で 0.25 m の所で止まるか、物から離れる向きには動けるか）
- スタックちゃんの声で走るのを床で確かめる（「前に 50 センチ進んで」「右に 90 度回って」「止まって」）
- SLAM の地図を部屋 1 周で作って、壁が二重にならないか確かめる
- 実機で自動モードの走行挙動（旋回方向・速度・距離しきい値）を確認し `autonomy_node` のパラメータを調整する
- 前後の VL53L1X で、段差の前で止まるかを床で確かめる（台の端に向けて「出力」15 % でゆっくり）
- `/odom` の横移動を床で合わせる
- Nav2 の行き先へ床で走らせる（出力 15 %。部屋 1 周の地図のあと、地図をクリック →「ここへ行く」。着いたら止まるか、360° の見張り・段差の止めが効くか）

## 変更ファイル
- 下は 2026/09 の最初の作業の記録（液晶・TM1637 は今は使っていない）。そのあとの変更は CHANGELOG.md と各 PR を見る
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
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/architecture.py, ai_car_web/dashboard_node.py, config/architecture.yaml, config/dashboard.yaml, static/index.html}` - プロジェクト説明モーダルと `/api/architecture`
- `/home/super/AI-CAR_ws/src/ai_car_web/{ai_car_web/motor_hat.py, config/motor_hat.yaml, config/gpio_pins.yaml, static/index.html}`、`HARDWARE_BOM.md` - 520 モーターエンコーダーの Pi GPIO 割り付けを追加
- `/home/super/AI-CAR_ws/PROJECT_STATUS.md` - 本ファイル
- `/home/super/AI-CAR_ws/CHANGELOG.md` - 更新
