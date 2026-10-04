// AI-CAR バッテリー 2 個 + RPLIDAR A1M8 マウント
// 座標: 天板 154 x 200 mm の左下角が原点。X = 横幅方向、Y = 前後方向（Y=200 側が前 / LiDAR 側）
// 天板の穴位置は Y 方向に対称なので、前後どちら向きでも同じ穴に合う。
//
// 出力: openscad -D 'part="box"' -o box.stl battery_lidar_mount.scad
//       openscad -D 'part="lid"' -o lid.stl battery_lidar_mount.scad
//       openscad -D 'part="pi_base"' -o pi_base.stl battery_lidar_mount.scad
//       openscad -D 'part="cam_mount"' -o cam_mount.stl battery_lidar_mount.scad（2 眼カメラの板。裏を下にした向き）
//       openscad -D 'part="bumper"' -o bumper.stl battery_lidar_mount.scad（前後共通、2 個印刷）
//       openscad -D 'part="cover"' -o cover.stl battery_lidar_mount.scad（上面カバー、上面を下にした向き）

part = "assembly"; // "box" | "lid" | "pi_base" | "cam_mount" | "bumper" | "cover" | "assembly"
show_cables = false;  // 組立図に配線経路の目安を描く
show_cover = false;   // 組立図に上面カバーを描く

$fn = 48;

// ---- 天板（ユーザー提供の CAD 図面）----
plate_w = 154;
plate_l = 200;
plate_t = 2;               // 実測
plate_holes = [[11, 11], [143, 11], [11, 189], [143, 189],
               [9, 90], [145, 90], [9, 110], [145, 110]];
mount_holes = [[11, 189], [143, 189], [9, 110], [145, 110]];
mount_hole_d = 4.4;

// 天板の上に頭が出ているモーター取付のネジ（図面: 28 x 28 mm の範囲 4 か所、左右の端から 7 mm、前後の端から 38 mm、高さ 3 mm）
// 上に載る部品（前は箱、後ろ左はラズパイ台）の底にくぼみを付けて逃げる
motor_scr_sq = 28;
motor_scr_h = 3;
motor_scr_pts = [[7, 38], [plate_w - 7 - motor_scr_sq, 38],
                 [7, plate_l - 38 - motor_scr_sq], [plate_w - 7 - motor_scr_sq, plate_l - 38 - motor_scr_sq]];  // 各範囲の左後ろの角
recess_clr = 0.5;         // ネジとくぼみのすき間（横と上）
recess_roof = 1.5;        // くぼみの上に残す厚さ
recess_rim = 1.5;         // くぼみの周りの縁
recess_t = motor_scr_h + recess_clr + recess_roof;  // くぼみの上の部品の厚さ

module motor_recess() {
    for (p = motor_scr_pts) translate([p[0] - recess_clr, p[1] - recess_clr, -1])
        cube([motor_scr_sq + 2 * recess_clr, motor_scr_sq + 2 * recess_clr, 1 + motor_scr_h + recess_clr]);
}

module motor_recess_pad(pts) {
    m = recess_clr + recess_rim;
    for (p = pts) rrect(p[0] - m, p[1] - m, p[0] + motor_scr_sq + m, p[1] + motor_scr_sq + m, 2, recess_t);
}

// ---- ネジ穴 ----
// "tap": ネジを直接ねじ込んで、ネジ自身にねじ山を切らせる下穴（M2〜M3 のねじ山は FDM ではきれいに出ないため）
// "insert": 熱圧入インサート（真鍮）を入れる穴
screw_mode = "tap";
function tap_d(m) = m == 2 ? 1.7 : m == 2.5 ? 2.1 : 2.5;
function insert_d(m) = m == 2 ? 3.2 : m == 2.5 ? 3.6 : 4.0;
function nut_hole_d(m) = screw_mode == "insert" ? insert_d(m) : tap_d(m);
function clear_d(m) = m + 0.4;
function head_d(m) = m == 2 ? 4.4 : m == 2.5 ? 5.2 : 6.2;

// 入口を z = 0 とし、-Z 方向へ depth の深さのねじ込み穴（入口に面取り）
module screw_hole(m, depth, d = undef) {
    d = is_undef(d) ? nut_hole_d(m) : d;
    translate([0, 0, -depth]) cylinder(d = d, h = depth + 0.01);
    translate([0, 0, -0.6]) cylinder(d1 = d, d2 = d + 1.2, h = 0.61);
    cylinder(d = d + 1.2, h = 1);
}

// z = 0 から +Z 方向へ h の厚さを通す穴。cb > 0 なら下側から深さ cb の座ぐり（ネジ頭を中に沈める）
module clear_hole(m, h, cb = 0) {
    translate([0, 0, -1]) cylinder(d = clear_d(m), h = h + 2);
    if (cb > 0) translate([0, 0, -1]) cylinder(d = head_d(m), h = cb + 1);
}

// ---- バッテリー（CIO SMARTCOBY Pro SLIM 35W）x 2 段重ね ----
bat_x = 98.2;   // 長辺（実測。X 方向に置く。ポートのある短辺が開口側を向く）
bat_y = 66.8;   // 短辺（実測）
bat_z = 16.2;
bat_n = 2;
clr = 0.4;      // 前後の片側のすき間
clr_x = 0.3;    // 左右の片側のすき間

// ---- 箱 ----
wall = 2.5;
wall_y = 3.8;   // 前後の壁（外形 75.2 mm を保ち、内側へ厚くしてすき間を clr にする）
floor_t = recess_t;   // 前のモーターのネジの上に載るので、くぼみの分だけフランジより厚い
flange_t = 3;
flange_y0 = 104;
flange_y1 = 196;
flange_x0 = 3;
flange_x1 = 151;
flange_r = 4;
open_end = "right";   // バッテリー出し入れ・ケーブル側: "right"(+X) / "left"(-X)
strap_w = 12;         // 面ファスナー / 結束バンド用スロットの高さ
strap_len = 3;        // スロットの幅（ストラップ厚み方向）

in_x = bat_x + 2 * clr_x;
in_y = bat_y + 2 * clr;
in_z = bat_n * bat_z + 1.0;
box_x = in_x + 2 * wall;
box_y = in_y + 2 * wall_y;
box_h = floor_t + in_z;
box_cx = plate_w / 2;
box_cy = (flange_y0 + flange_y1) / 2;
box_x0 = box_cx - box_x / 2;
box_y0 = box_cy - box_y / 2;
// 開口と反対側の端の壁を内側へ厚くする量（実物の長手方向のすき間 約 2 mm を 2 mm のプレートでふさいで確認）
end_fill = 2;
in_x0 = box_x0 + wall + (open_end == "right" ? end_fill : 0);  // バッテリーを入れる空間の左端

// 上のバッテリーの残量表示（ロゴの面を上、ポートを右にして入れる）
// ポートの短辺から 7〜20 mm、ロゴの面でポートを上にしたときの右の長辺から 9〜21 mm（21 は写真からの推定）
bat_disp = [7, 20, 9, 21];
disp_lid_m = 1;           // ふたの窓の余白
disp_cov_m = 2;           // カバーの窓の余白（斜めから見る分）
function disp_rect() = let(x1 = in_x0 + clr_x + bat_x, y0 = box_y0 + wall_y + clr)
    [x1 - bat_disp[1], y0 + bat_disp[2], x1 - bat_disp[0], y0 + bat_disp[3]];

// ふた固定用ボス（箱の外側 4 隅、M3 タッピング）
boss_d = 8;
boss_pts = [[box_x0 - boss_d / 2 + 1.5, box_y0 + 6],
            [box_x0 + box_x + boss_d / 2 - 1.5, box_y0 + 6],
            [box_x0 - boss_d / 2 + 1.5, box_y0 + box_y - 6],
            [box_x0 + box_x + boss_d / 2 - 1.5, box_y0 + box_y - 6]];
// 中を抜くボス（boss_pts の番号）。開口側の後ろの角は、バッテリーに差す L 字の USB プラグが当たるので、上下 boss_keep だけ残す
boss_open = [1];
boss_keep = 5;            // 残す高さ（下はフランジの上から、上は箱の上端から）
boss_tie_l = 20;          // 上の部分を、後ろ（前）の壁の外側に沿わせて留める長さ
boss_tie_t = 2;           // 壁の外側に足す厚さ

// ---- 2 眼カメラ（3D USB Camera 3D-1080P02、基板 80 x 17 mm）: ふたの前、LiDAR のモーターの下に、USB 端子を上にして立てる ----
// 四隅の M2 の穴で、ふたの前端の面に当てる板（cam_mount.stl）に留める。板は左右の耳を、ふたの前端の柱 2 本に M3 で留める
// USB は基板の上の端から L 字のプラグを上から差し、ケーブルを後ろへ曲げて、ふたの上を Pi へ通す
scam_board = [80, 17, 1.6];   // 幅 x 高さ（実測）x 厚さ（仮）
scam_baseline = 60;           // 左右のレンズの中心の間隔（約 60 mm）
scam_lens_d = 18;             // レンズのいちばん太い所（実測）
scam_lens_l = 26;             // 基板の表からレンズの先まで（実測）
scam_lens_z = 0;              // 仮: 基板の上下の中心からレンズの中心まで
scam_hole_in = 1.5;           // 四隅の M2 の穴の中心の、基板のふちからの距離（実測）
scam_back_h = 2;              // 基板の裏の部品の高さ（実測）
scam_usb_room = 15;           // 基板の上端から LiDAR のモーターの下端まで（L 字の USB-C プラグの分）
scam_mnt_t = 2.5;             // 板の厚さ
scam_so = scam_back_h + 0.5;  // 板から基板の裏までの支柱の長さ
scam_so_d = 4.6;              // 支柱の太さ
scam_hole_d = 2.5;            // 支柱と板の M2 の下穴（1.7 では印刷で約 0.5 mm につぶれたため）
scam_ear_w = 8;               // 板の左右の耳（ふたの柱に M3 で留める）の幅
scam_lpost_d = 6;             // ふたの柱の奥行き
scam_slot_clr = 1.5;          // カバーの窓とレンズのすき間（片側）

tie_w = 4.5;              // 結束バンド（幅 3.6 mm まで）を通すトンネル
tie_h = 2;

// ---- ラズパイ台（Raspberry Pi 5、天板の後ろ側）----
pi_mount_holes = [[11, 11], [143, 11], [9, 90], [145, 90]];
pi_cx = 47;               // 基板中心。電源ボタンの短辺（-X）をカバーの左の壁に寄せ、USB / LAN 側（+X）を広く空ける
pi_cy = 50;
pi_board = [85, 56];
pi_hole_dx = 58;
pi_hole_dy = 49;
pi_hole_off = 3.5;        // 電源ボタン側の短辺からの距離（USB / LAN は +X 側）
pi_btn_y = [0, 18];       // 仮: 電源ボタンを探す範囲（-X の短辺に沿って、USB-C の長辺（-Y）から）。カバーの左の壁の穴になる
pi_btn_z = [-2, 9];       // 仮: 穴の高さ（Pi の基板の下面から）
pi_so_hole_d = 6.2;       // 幅 5 mm の M2.5 六角支柱を差し込む丸い穴（六角の角と角の間 5.77 mm + 0.43 mm）
pi_so_in = 5;             // 支柱を差し込む深さ
pi_so_l = 5;              // 支柱の長さ（Pi の基板の下に出ている部分。仮の値で、Pi の高さの図とカバーの確認だけに使う）
pi_boss_d = pi_so_hole_d + 2 * 1.6;
pi_boss_h = 6;
pi_screw_cb = 2;          // 台の裏から支柱を留める M2.5 のネジの頭を沈める深さ
pi_base_t = 3;
pi_z = pi_base_t + pi_boss_h - pi_so_in + pi_so_l;   // Pi の基板の下面
pi_stack_h = 45;          // Pi 5 + AI HAT+ + Motor HAT の 3 段（実測。台のぶん高くなる側で見積もる）
pi_pts = [for (dx = [0, pi_hole_dx], dy = [0, pi_hole_dy])
          [pi_cx - pi_board[0] / 2 + pi_hole_off + dx, pi_cy - pi_hole_dy / 2 + dy]];

// ---- IMU（GY-BNO055）: ふたの上、LiDAR のヘッドの下（右寄り）----
imu_board = [12, 20];     // 台の座標で長辺を Y、ピンヘッダーは +X 側の長辺
imu_c = [65, 122.3];      // ふたの上、LiDAR のヘッドの下の後ろ寄り（後ろの柱 2 本の間）。前の壁の外側が Y 130.2
imu_rot = -90;            // 台を Z まわりに回す角度。-90 で長辺が左右、ピンヘッダーの開いた側が後ろ（Pi 側）
imu_lift = 25;            // 基板の下の空き。ピンヘッダーは下向きで、ジャンパー線のコネクターを下に収める
imu_wall = 1.6;
imu_ledge = 1;            // 基板の縁を下から受ける幅（ピンヘッダーのない 3 辺）
imu_hdr = 2.56;           // 基板の下のピンヘッダーの列（+X 側の長辺に沿って全長）の幅。台はこの分を受けずに外へ出す
imu_lip = 3;              // 閉じた長辺（台の座標で -X、組み立てでは前）の壁から内側へ出して、基板の縁を上から押さえる出っ張りの幅
imu_lip_t = 3;            // 出っ張りの厚さ


// ---- 前後バンパー + 落下防止センサー（VL53L1X、下向き）----
// 前は箱のフランジ、後ろはラズパイ台の腕に重ねて四隅の穴で共締め。後ろは前を 180° 回したもの
bump_t = 4;
bump_base_d = 19;         // 固定側の板の奥行き（天板の端から）
bump_travel = 4;          // 前面の板が後ろへ逃げられる量（ストッパーまで）
bump_face_t = 3;
bump_edge = 1;            // 手で触る縁の面取り
bump_drop = 15;           // 前面の板を天板上面から下へ伸ばす長さ
bump_arm_x = [[3, 18], [136, 151]];  // 箱のボスを避けた腕の範囲
bump_arm_y0 = 183;
// 腕の下の爪: 箱のフランジの前の縁（後ろはラズパイ台の腕の端）に当て、衝撃をネジではなく左右 2 か所の面で受ける
bump_hook_x = [[7.2, 18], [136, 146.8]];  // フランジの角の丸みを避けた平らな範囲
bump_hook_clr = 0.1;      // 爪と縁のすき間
bump_hook_y1 = plate_l + 4;  // 爪の前端（天板の前端より前まで伸ばして腕と板につなぐ）
// 板ばね: 横長の板の片端を固定側、反対の端を前面の板につなぐ（向きを交互にして前面の板を平行に動かす）
spring_x = [[7, 40], [41, 74], [80, 113], [114, 147]];  // 板ばねの左端と右端
spring_t = 0.8;           // 板厚（PLA でもひずみ約 1.1%。ノズル 0.4 mm の 2 周）
spring_h = 8;
spring_clr = 1;           // ストッパーで止まったときに残るすき間
post_w = 3;
stop_x = [4, 77, 150];
bump_gap = 2 * (bump_travel + spring_clr) + spring_t;
bump_out = bump_base_d + bump_gap + bump_face_t;
// 両端を固定した板ばね（一端がずれる）の曲げひずみ = 3 t δ / L²
spring_strain = 3 * spring_t * bump_travel / pow(spring_x[0][1] - spring_x[0][0] - post_w, 2);
tof_board = [25, 10.7, 1.6];  // VL53L1X 基板（Dovhmoh: 25 x 10.7、長辺を横幅方向に置く）
tof_cover = [12.2, 7.4];  // 下面のレンズのカバー（実測、写真から）。中心は基板の中心から 1.9 mm ピンと反対側へ寄り、縁は基板の長辺まで来る
tof_win = tof_cover[0] + 1.2;  // 窓の幅。長辺の方向は受けの穴いっぱいに開け、基板を左右の耳（丸穴の部分）で受ける
tof_tilt = 45;            // 真下から進行方向へ傾ける角度
tof_drop = 8;             // 受けの中心をバンパー下面から下げる量
tof_dy = 14;              // 受けの中心の天板の端からの距離（上から落とし込む穴が天板の角にかからない位置）
tof_face_t = 2;

// ---- ふた + LiDAR 台 ----
lid_t = 4;
lidar_tower_h = 48;     // LiDAR 取付面より下に出ている部分（約 26.5 mm）をかわし、ヘッドをカバーの上面（82.4 mm）より上に出す高さ
lidar_tower_d = 8.5;
lidar_floor = 4;        // 柱の上端に残す厚さ。LiDAR 底面の M2.5 ねじ穴へ、ふたの裏から柱の中を通したネジで留める
lidar_motor_front = true; // LiDAR のモーター側（細い側）を前（+Y）に向ける
// RPLIDAR A1M8 取付穴（回転中心基準、データシート Figure 5-2）
//   ヘッド側 2 穴: 中心から 28 mm、間隔 56 mm / モーター側 2 穴: 中心から 42 mm、間隔 40 mm
lidar_holes_rel = [[-28, 28], [28, 28], [-20, -42], [20, -42]];
lidar_dx = -8;          // 左へ寄せる（ROS の TF と dashboard.yaml のカメラとの位置の差に反映済み。動かすときは両方を直す）
lidar_cx = box_cx + lidar_dx;
lidar_motor_d = 32;     // 取付面より下に出るモーター（回転中心から 45.5 mm、取付面から 26.5 mm 下まで）
lidar_motor_off = 45.5;
lidar_below = 26.5;
lidar_core_below = 8;   // ヘッドの下の基板部（約 60 x 60 mm）が取付面から下に出る量（図からの読み取り）
lidar_cy = box_cy + (lidar_motor_front ? -7 : 7);

function lidar_hole(p) = lidar_motor_front
    ? [lidar_cx + p[0], lidar_cy - p[1]]
    : [lidar_cx + p[0], lidar_cy + p[1]];

scam_cx = lidar_cx;                         // 左右のレンズの真ん中を LiDAR の回転中心にそろえる
scam_z1 = box_h + lid_t + lidar_tower_h - lidar_below - scam_usb_room;  // 基板の上端（組み立ての Z）
scam_z0 = scam_z1 - scam_board[1];          // 基板の下端。ふたの上面より下（箱の前）
scam_mnt_y0 = box_y0 + box_y + 2;           // 板の裏（ふたの前端の面に当てる）
scam_back_y = scam_mnt_y0 + scam_mnt_t + scam_so;  // 基板の裏
scam_front = scam_back_y + scam_board[2];   // 基板の表
scam_lens_zc = scam_z0 + scam_board[1] / 2 + scam_lens_z;
scam_tip_y = scam_front + scam_lens_l;
scam_ends = [scam_cx - scam_board[0] / 2, scam_cx + scam_board[0] / 2];
scam_holes = [for (x = [scam_ends[0] + scam_hole_in, scam_ends[1] - scam_hole_in], z = [scam_z0 + scam_hole_in, scam_z1 - scam_hole_in]) [x, z]];
scam_mnt_x = [scam_ends[0] - 3.5, scam_ends[1] + 3.5];  // 板の左右の端（カバーのリブ X 21〜25 / 129〜133 の内側）
scam_ear_x = [scam_mnt_x[0], scam_mnt_x[1] - scam_ear_w];  // 耳の左端
scam_ear_top = scam_z1 + 9;
scam_ear_scr_z = scam_z1 + 5;               // 耳の M3 の高さ（支柱の M2 の頭とネジの頭が重ならない）
scam_tie = [77, 166];                       // カメラの USB ケーブルを留める結束バンドの受け（LiDAR の USB 変換基板の右）
rear_tie = [86, box_y0 + 7];                // LiDAR とカメラの USB ケーブルを留める受け（ふたの後ろ端、IMU の台と後ろ右の柱の間）
scam_nut_af = 4.4;                          // M2 ナット（二面幅 4 mm、厚さ 1.6 mm）を入れる六角のくぼみの二面幅
scam_nut_d = 2.5;                           // くぼみの深さ（柱の前の面から）

module rrect(x0, y0, x1, y1, r, h) {
    hull() for (x = [x0 + r, x1 - r], y = [y0 + r, y1 - r])
        translate([x, y, 0]) cylinder(r = r, h = h);
}

// 角丸の板の上下の縁を c だけ面取りしたもの（手で触る部品）
module crrect(x0, y0, x1, y1, r, h, c) {
    hull() for (x = [x0 + r, x1 - r], y = [y0 + r, y1 - r]) translate([x, y, 0]) {
        cylinder(r1 = r - c, r2 = r, h = c);
        translate([0, 0, c]) cylinder(r = r, h = h - 2 * c);
        translate([0, 0, h - c]) cylinder(r1 = r, r2 = r - c, h = c);
    }
}

// 結束バンドを下にくぐらせるブリッジ。ケーブルは Y 方向に沿って上に載せ、バンドは X 方向に通す
module tie_mount() {
    difference() {
        translate([-4, -3, 0]) cube([8, 6, tie_h + 1.4]);
        translate([-5, -tie_w / 2, -0.01]) cube([10, tie_w, tie_h]);
    }
}

tie_pts_box = [[13, 128], [13, 168], [134, 108]];  // 左の 2 個はモーターのネジのくぼみの前後

// DROK 降圧コンバーター（12V → 5.2V、B09JKHD3SH）。取付穴がないので、箱の後ろの壁に立てて結束バンドで留める
drok_board = [63, 27, 1.6];   // 商品画像の寸法
drok_h = 13;                  // 基板の裏から部品の頂点まで（推定）
drok_gap = 3;                 // 基板の裏と壁のすき間（裏のピンの逃げ）
drok_cx = box_cx - 4;
drok_z0 = flange_t + 2.5;     // 基板の下端。フランジとの間に結束バンドを通す
drok_by = box_y0 - drok_gap;  // 基板の裏面
drok_tie_x = box_cx - 6;      // 通気スロットの間
drok_rib_x = [box_cx - 18, box_cx + 18];

module drok_holder() {
    x0 = drok_cx - drok_board[0] / 2;
    x1 = drok_cx + drok_board[0] / 2;
    fy = drok_by - drok_board[2] - 0.4;
    difference() {
        union() {
            translate([drok_tie_x - 3.5, drok_by, drok_z0]) cube([7, drok_gap + 0.01, drok_board[1]]);
            for (x = drok_rib_x) translate([x - 1, drok_by, drok_z0]) cube([2, drok_gap + 0.01, drok_board[1]]);
            for (r = [[x0, drok_tie_x - 4.5], [drok_tie_x + 4.5, x1]])
                translate([r[0], fy - 1.2, flange_t - 0.01]) cube([r[1] - r[0], box_y0 - fy + 1.2, drok_z0 - flange_t + 0.01]);
            for (x = [drok_cx - 23, drok_cx + 7])
                translate([x, fy - 1.2, drok_z0 - 0.01]) cube([12, 1.2, 1.5]);
        }
        translate([drok_tie_x - tie_w / 2, box_y0 - tie_h, drok_z0 - 1]) cube([tie_w, tie_h + 0.01, drok_board[1] + 2]);
    }
}

// ふた固定用のボス。boss_open のものは上下だけ残し、上の部分を壁の端と外側に広くつなぐ
function boss_side(p) = [p[0] > box_cx ? 1 : -1, p[1] > box_cy ? 1 : -1];
function boss_wall(p) = let(s = boss_side(p), yo = s[1] > 0 ? box_y0 + box_y : box_y0)
    [s[0] > 0 ? box_x0 + box_x : box_x0, yo, yo - s[1] * wall_y];  // 壁の端の X、壁の外側と内側の Y
module boss_end_rect(p, z, h) {
    w = boss_wall(p);
    translate([min(w[0], w[0] - boss_side(p)[0] * 1.5), min(w[1], w[2]), z]) cube([1.5, wall_y, h]);
}
// 上の部分。上端から boss_keep の厚さだけで、上から見て三角の板にして壁の端と外側につなぐ（これより下には作らない）
module boss_top(p) {
    sd = boss_side(p);
    w = boss_wall(p);
    z1 = box_h - boss_keep;
    tx0 = min(w[0] - sd[0] * boss_tie_l, p[0]);
    tx1 = max(w[0] - sd[0] * boss_tie_l, p[0]);
    ty = sd[1] > 0 ? w[1] : w[1] - boss_tie_t;
    difference() {
        union() {
            hull() {
                translate([p[0], p[1], z1]) cylinder(d = boss_d, h = boss_keep);
                boss_end_rect(p, z1, boss_keep);
            }
            hull() {
                translate([p[0], p[1], z1]) cylinder(d = boss_d, h = boss_keep);
                translate([tx0, ty, z1]) cube([tx1 - tx0, boss_tie_t, boss_keep]);
            }
        }
        // バッテリーの入る側へははみ出さない
        translate([sd[0] > 0 ? w[0] - 1.5 - 60 : w[0] + 1.5, sd[1] > 0 ? w[2] - 60 : w[2], z1 - 1])
            cube([60, 60, boss_keep + 2]);
    }
}
module boss_body(i) {
    p = boss_pts[i];
    if (len(search(i, boss_open)) == 0) {
        translate([p[0], p[1], 0]) cylinder(d = boss_d, h = box_h);
    } else {
        translate([p[0], p[1], 0]) cylinder(d = boss_d, h = flange_t + boss_keep);
        boss_top(p);
    }
}

module box() {
    difference() {
        union() {
            rrect(flange_x0, flange_y0, flange_x1, flange_y1, flange_r, flange_t);
            translate([box_x0, box_y0, 0]) cube([box_x, box_y, box_h]);
            for (i = [0 : len(boss_pts) - 1]) boss_body(i);
            motor_recess_pad([motor_scr_pts[2], motor_scr_pts[3]]);
            for (p = tie_pts_box) translate([p[0], p[1], flange_t - 0.01]) tie_mount();
            drok_holder();
        }
        // バッテリー収納部
        translate([in_x0, box_y0 + wall_y, floor_t]) cube([in_x - end_fill, in_y, in_z + 1]);
        // 開口側の端（全面開口）
        ox = open_end == "right" ? box_x0 + box_x - wall - 1 : box_x0 - 1;
        difference() {
            translate([ox, box_y0 + wall_y, floor_t]) cube([wall + 2, in_y, in_z + 1]);
            for (i = [0 : len(boss_pts) - 1]) boss_body(i);
        }
        // 反対側の端: 押し出し用の窓
        cx = open_end == "right" ? box_x0 - 1 : box_x0 + box_x - wall - end_fill - 1;
        translate([cx, box_cy - 20, floor_t + 4]) cube([wall + end_fill + 2, 40, in_z - 8]);
        // 固定用ストラップのスロット（上下バッテリーの境目の高さ、開口側寄り）
        sx = open_end == "right" ? box_x0 + box_x - wall - 12 : box_x0 + wall + 12 - strap_len;
        for (y = [box_y0 - 1, box_y0 + box_y - wall_y - 1])
            translate([sx, y, floor_t + bat_z - strap_w / 2 + 0.5]) cube([strap_len, wall_y + 2, strap_w]);
        // 通気スロット（長辺）
        for (i = [-2 : 2])
            translate([box_cx + i * 12 - 2.5, box_y0 - 1, floor_t + 6]) cube([5, wall_y + 2, in_z - 12]);
        // 床の肉抜き
        translate([box_x0 + wall + 12, box_y0 + wall_y + 10, -1]) cube([in_x - 24, in_y - 20, floor_t + 2]);
        // 天板への取付穴
        for (p = mount_holes) translate([p[0], p[1], -1]) cylinder(d = mount_hole_d, h = flange_t + 2);
        // ふた固定用の下穴
        for (p = boss_pts) translate([p[0], p[1], box_h]) screw_hole(3, 12);
        motor_recess();
    }
}

module lid() {
    lx0 = box_x0 - boss_d + 1.5;
    lx1 = box_x0 + box_x + boss_d - 1.5;
    difference() {
        union() {
            rrect(lx0, box_y0 - 2, lx1, box_y0 + box_y + 2, 4, lid_t);
            for (p = lidar_holes_rel) {
                q = lidar_hole(p);
                translate([q[0], q[1], 0]) cylinder(d = lidar_tower_d, h = lid_t + lidar_tower_h);
            }
            scam_lid_posts();
            translate([scam_tie[0], scam_tie[1], lid_t - 0.01]) tie_mount();
            imu_holder(lid_t);
            // LiDAR のハーネスと USB ケーブルの固定（ふたの後ろ端）
            translate([rear_tie[0], rear_tie[1], lid_t - 0.01]) rotate([0, 0, 90]) tie_mount();
        }
        for (p = boss_pts) translate([p[0], p[1], 0]) clear_hole(3, lid_t);
        for (p = lidar_holes_rel) {
            q = lidar_hole(p);
            translate([q[0], q[1], 0]) clear_hole(2.5, lid_t + lidar_tower_h, lid_t + lidar_tower_h - lidar_floor);
        }
        scam_lid_post_cut();
        // バッテリーの残量表示の窓
        d = disp_rect();
        translate([d[0] - disp_lid_m, d[1] - disp_lid_m, -1]) cube([d[2] - d[0] + 2 * disp_lid_m, d[3] - d[1] + 2 * disp_lid_m, lid_t + 2]);
    }
}

// ふたの前端の柱（耳を M3 で留める）。ふたの座標
module scam_lid_posts() {
    for (x = scam_ear_x) translate([x, scam_mnt_y0 - scam_lpost_d, lid_t - 0.01])
        cube([scam_ear_w, scam_lpost_d, scam_ear_top - box_h - lid_t + 0.01]);
}

module scam_lid_post_cut() {
    for (x = scam_ear_x) translate([x + scam_ear_w / 2, scam_mnt_y0, scam_ear_scr_z - box_h]) rotate([-90, 0, 0])
        screw_hole(3, scam_lpost_d + 1);
    // 板の上の M2 の後ろ（ナットとネジの先）の逃げ
    for (h = scam_holes) if (h[1] > box_h + lid_t) translate([h[0], scam_mnt_y0, h[1] - box_h]) rotate([-90, 0, 0]) {
        translate([0, 0, -scam_nut_d]) rotate([0, 0, 30]) cylinder(d = scam_nut_af / cos(30), h = scam_nut_d + 0.01, $fn = 6);
        translate([0, 0, -scam_lpost_d - 1]) cylinder(d = clear_d(2), h = scam_lpost_d + 2);
    }
}

// 2 眼カメラの板（組み立ての座標）。基板は前から M2 × 6 mm を 4 本、板は耳を M3 × 8 mm で 2 本
module cam_mount() {
    y1 = scam_mnt_y0 + scam_mnt_t;
    zb = scam_z0 - 2;
    difference() {
        union() {
            translate([scam_mnt_x[0], scam_mnt_y0, zb]) cube([scam_mnt_x[1] - scam_mnt_x[0], scam_mnt_t, scam_z1 - 3 - zb]);
            for (x = scam_ear_x) translate([x, scam_mnt_y0, zb]) cube([scam_ear_w, scam_mnt_t, scam_ear_top - zb]);
            for (h = scam_holes) translate([h[0], y1 - 0.01, h[1]]) rotate([-90, 0, 0]) cylinder(d = scam_so_d, h = scam_so + 0.01);
        }
        for (h = scam_holes) translate([h[0], scam_back_y, h[1]]) rotate([-90, 0, 0]) screw_hole(2, scam_so + scam_mnt_t + 1, scam_hole_d);
        for (x = scam_ear_x) translate([x + scam_ear_w / 2, scam_mnt_y0, scam_ear_scr_z]) rotate([-90, 0, 0]) clear_hole(3, scam_mnt_t);
    }
}

module pi_base() {
    pad = [pi_pts[0][0] - 5, pi_pts[0][1] - 5, pi_pts[3][0] + 5, pi_pts[3][1] + 5];
    corners = [[pad[0] + 5, pad[1] + 5], [pad[2] - 5, pad[1] + 5], [pad[0] + 5, pad[3] - 5], [pad[2] - 5, pad[3] - 5]];
    // 天板の穴 → 近い台の角
    arm = [[0, 0], [1, 1], [2, 2], [3, 3]];
    difference() {
        union() {
            rrect(pad[0], pad[1], pad[2], pad[3], 5, pi_base_t);
            for (a = arm) hull() {
                translate([pi_mount_holes[a[0]][0], pi_mount_holes[a[0]][1], 0]) cylinder(r = 6, h = pi_base_t);
                translate([corners[a[1]][0], corners[a[1]][1], 0]) cylinder(r = 6, h = pi_base_t);
            }
            for (p = pi_pts) translate([p[0], p[1], 0]) cylinder(d = pi_boss_d, h = pi_base_t + pi_boss_h);
            // 後ろのバンパーの爪を受ける平らな端（前のフランジの縁と同じ位置 Y = plate_l - flange_y1）
            for (a = bump_arm_x) rrect(a[0], plate_l - flange_y1, a[1], plate_l - bump_arm_y0, 1, pi_base_t);
        }
        for (p = pi_mount_holes) translate([p[0], p[1], -1]) cylinder(d = mount_hole_d, h = pi_base_t + 2);
        motor_recess();
        // 六角支柱を差し込む丸い穴。支柱は台の裏から M2.5 のネジで留める
        for (p = pi_pts) translate([p[0], p[1], 0]) {
            translate([0, 0, pi_base_t + pi_boss_h - pi_so_in]) cylinder(d = pi_so_hole_d, h = pi_so_in + 1);
            translate([0, 0, pi_base_t + pi_boss_h - 0.5]) cylinder(d1 = pi_so_hole_d, d2 = pi_so_hole_d + 1, h = 0.51);
            clear_hole(2.5, pi_base_t + pi_boss_h - pi_so_in, pi_screw_cb);
        }
        // 配線通し・通気（後ろ側は台の縁まで開け、天板の開口から上がるモーター線を Pi の後ろへ出す）
        translate([pad[0] + 10, pad[1] - 1, -1]) cube([pad[2] - pad[0] - 20, pad[3] - pad[1] - 9, pi_base_t + 2]);
    }
}

// z0: 台を載せる面の高さ（部品の座標）
module imu_holder(z0) {
    ix = imu_board[0] + 0.6;
    iy = imu_board[1] + 0.6;
    ox = ix + 2 * imu_wall;
    oy = iy + 2 * imu_wall;
    h = imu_lift + 1.6 + 0.3 + 1.2;
    // ピンヘッダーのある +X 側は壁なしで開け、ピンヘッダーの列の幅だけ台を短くして基板をはみ出させ、ジャンパー線を差したまま上から入れて縁の受けに載せる
    translate([imu_c[0], imu_c[1], z0 - 0.01]) rotate([0, 0, imu_rot]) difference() {
        translate([-ox / 2, -oy / 2, 0]) cube([ox / 2 + imu_board[0] / 2 - imu_hdr, oy, h]);
        translate([-ix / 2, -iy / 2, imu_lift]) cube([ix + 5, iy, h]);
        translate([-ix / 2 + imu_ledge, -iy / 2 + imu_ledge, 0.01]) cube([ix + 5, iy - 2 * imu_ledge, h + 1]);
    }
    // 奥の長辺の出っ張り: 基板は +X 側から斜めに差し込み、-X 側の縁をこの下に入れてから段に載せる
    translate([imu_c[0], imu_c[1], z0]) rotate([0, 0, imu_rot]) translate([-ox / 2, -oy / 2, imu_lift + 1.6 + 0.3]) cube([imu_wall + imu_lip, oy, imu_lip_t]);
}

function tof_c() = [plate_w / 2, plate_l + tof_dy];

// 受けのローカル座標: センサーは -Z 方向を見る。rotate([tof_tilt, 0, 0]) で前下を向く
module tof_at(z0) {
    c = tof_c();
    translate([c[0], c[1], z0 - tof_drop]) rotate([tof_tilt, 0, 0]) children();
}

// 固定側（腕 + 板 + センサー受け）と前面の板を、板ばねでつなぐ。ぶつかると前面の板だけが後ろへ逃げ、ストッパーで止まる
module bumper() {
    z0 = flange_t;
    yb = plate_l + bump_base_d;
    yf = yb + bump_gap;
    zs = z0 + bump_t - spring_h;
    c = tof_c();
    m = 2.5;
    bw = tof_board[0] + 0.6;
    bh = tof_board[1] + 0.6;
    difference() {
        union() {
            for (a = bump_arm_x) translate([0, 0, z0]) rrect(a[0], bump_arm_y0, a[1], plate_l + 1, 3, bump_t);
            for (h = bump_hook_x) translate([h[0], flange_y1 + bump_hook_clr, 0])
                cube([h[1] - h[0], bump_hook_y1 - flange_y1 - bump_hook_clr, z0 + 0.01]);
            translate([0, 0, z0]) crrect(0, plate_l, plate_w, yb, 6, bump_t, bump_edge);
            translate([0, 0, zs]) rrect(0, yb - 3, plate_w, yb, 1, spring_h);
            for (x = stop_x) translate([x - 2, yb - 0.01, zs]) cube([4, bump_gap - bump_travel, spring_h]);
            yl = yb + bump_travel + spring_clr;
            for (i = [0 : len(spring_x) - 1]) {
                x0 = spring_x[i][0]; x1 = spring_x[i][1];
                fixed_left = i % 2 == 0;
                translate([x0, yl, zs]) cube([x1 - x0, spring_t, spring_h]);
                translate([fixed_left ? x0 : x1 - post_w, yb - 0.01, zs]) cube([post_w, yl - yb + 0.02, spring_h]);
                translate([fixed_left ? x1 - post_w : x0, yl + spring_t - 0.01, zs]) cube([post_w, yf - yl - spring_t + 0.02, spring_h]);
            }
            translate([0, 0, z0 - bump_drop]) crrect(0, yf, plate_w, yf + bump_face_t, bump_face_t / 2, bump_drop + bump_t, bump_edge);
            hull() {
                tof_at(z0) translate([-bw / 2 - m, -bh / 2 - m, -tof_face_t]) cube([bw + 2 * m, bh + 2 * m, tof_face_t + tof_board[2] + 1]);
                translate([c[0] - bw / 2 - m, c[1] - bh / 2 - m, z0]) cube([bw + 2 * m, bh + 2 * m, bump_t]);
            }
        }
        for (p = [[11, 189], [143, 189]]) translate([p[0], p[1], z0 - 1]) cylinder(d = mount_hole_d, h = bump_t + 2);
        // 上から斜めの穴に落とし込み、縁で受ける（ピンは上向き、配線は穴から上へ）
        tof_at(z0) {
            translate([-bw / 2, -bh / 2, 0]) cube([bw, bh, 60]);
            translate([-tof_win / 2, -bh / 2, -tof_face_t - 1]) cube([tof_win, bh, tof_face_t + 2]);
        }
    }
}

module tof_ghost() {
    for (r = [0, 180]) translate([plate_w / 2, plate_l / 2, 0]) rotate([0, 0, r]) translate([-plate_w / 2, -plate_l / 2, 0])
        tof_at(flange_t) {
            color("darkgreen") translate([-tof_board[0] / 2, -tof_board[1] / 2, 0.05]) cube([tof_board[0], tof_board[1], 1.6]);
            color("black") translate([-tof_cover[0] / 2, tof_board[1] / 2 - tof_cover[1], -1.55]) cube([tof_cover[0], tof_cover[1], 1.6]);
        }
}

module both_bumpers() {
    bumper();
    translate([plate_w, plate_l, 0]) rotate([0, 0, 180]) bumper();
}

module pi_ghost() {
    z = pi_z;
    color("green", 0.8) translate([pi_cx - pi_board[0] / 2, pi_cy - pi_board[1] / 2, z]) cube([pi_board[0], pi_board[1], 1.6]);
    color("darkslateblue", 0.5) translate([pi_cx - pi_board[0] / 2, pi_cy - pi_board[1] / 2, z + 1.6]) cube([65, pi_board[1], pi_stack_h - 1.6]);
}

module sensor_ghost() {
    z = box_h + lid_t;
    translate([imu_c[0], imu_c[1], z]) rotate([0, 0, imu_rot]) {
        color("purple") translate([-imu_board[0] / 2, -imu_board[1] / 2, imu_lift]) cube([imu_board[0], imu_board[1], 1.6]);
        color("black") translate([imu_board[0] / 2 - imu_hdr, -imu_board[1] / 2, imu_lift - 14]) cube([imu_hdr, imu_board[1], 14]);
    }
}

module camera_ghost() {
    color("darkgreen") translate([scam_ends[0], scam_back_y, scam_z0]) cube([scam_board[0], scam_board[2], scam_board[1]]);
    for (dx = [-scam_baseline / 2, scam_baseline / 2]) {
        color("dimgray") translate([scam_cx + dx - 7.5, scam_front, scam_lens_zc - 7.5]) cube([15, 10, 15]);
        color("black") translate([scam_cx + dx, scam_front - 0.01, scam_lens_zc]) rotate([-90, 0, 0]) cylinder(d = scam_lens_d, h = scam_lens_l);
    }
    // L 字の USB-C プラグ（目安）
    color("silver") translate([scam_cx - 6, scam_front, scam_z1]) cube([12, 7, 12]);
    color("white") translate([0, 0, 0]) cam_mount();
}

module drok_ghost() {
    x0 = drok_cx - drok_board[0] / 2;
    color("darkgreen") translate([x0, drok_by - drok_board[2], drok_z0]) cube([drok_board[0], drok_board[2], drok_board[1]]);
    color("silver") translate([x0 + 2, drok_by - drok_h, drok_z0 + 2]) cube([drok_board[0] - 4, drok_h - drok_board[2], drok_board[1] - 4]);
}

module plate() {
    color("skyblue") difference() {
        translate([0, 0, -plate_t]) cube([plate_w, plate_l, plate_t]);
        for (p = plate_holes) translate([p[0], p[1], -plate_t - 1]) cylinder(d = 4, h = plate_t + 2);
    }
}

module batteries() {
    for (i = [0 : bat_n - 1])
        color(i == 0 ? "dimgray" : "gray")
            translate([in_x0 + clr_x, box_y0 + wall_y + clr, floor_t + i * bat_z])
                cube([bat_x, bat_y, bat_z - 0.2]);
}

module lidar_ghost() {
    z = box_h + lid_t + lidar_tower_h;
    s = lidar_motor_front ? -1 : 1;
    color("black", 0.6) translate([0, 0, z]) {
        translate([lidar_cx, lidar_cy, 0]) cylinder(d = 70, h = 24);
        translate([lidar_cx, lidar_cy - s * 42 + s * 0, 0]) cylinder(d = 20, h = 3);
        hull() {
            translate([lidar_cx, lidar_cy, 0]) cylinder(d = 72, h = 2);
            translate([lidar_cx, lidar_cy - s * 50, 0]) cylinder(d = 24, h = 2);
        }
        translate([lidar_cx, lidar_cy - s * lidar_motor_off, -lidar_below]) cylinder(d = lidar_motor_d, h = lidar_below);
        translate([lidar_cx - 30, lidar_cy - 30, -lidar_core_below]) cube([60, 60, lidar_core_below]);
    }
}

// 配線経路の目安（組立図のみ）
module path(pts, d) {
    for (i = [0 : len(pts) - 2]) hull() {
        translate(pts[i]) sphere(d = d, $fn = 12);
        translate(pts[i + 1]) sphere(d = d, $fn = 12);
    }
}

module cable_ghost() {
    lt = box_h + lid_t;
    pz = pi_z + 1.6;
    // 2 眼カメラの USB（基板の裏 → ふたの上を後ろへ → LiDAR の線と一緒に Pi の USB へ）
    color("white") path([[scam_cx, scam_front + 3.5, scam_z1 + 9], [scam_cx, scam_mnt_y0 - 4, scam_z1 + 7], [scam_tie[0], scam_mnt_y0 - 16, lt + 3], [scam_tie[0], scam_tie[1], lt + 4.5],
                         [rear_tie[0] - 2, box_y0 + 22, lt + 4], [rear_tie[0] - 1, rear_tie[1], lt + 4], [lidar_cx + 30, box_y0 - 6, lt + 2],
                         [pi_cx + pi_board[0] / 2 + 12, pi_cy - 2, pz + 12], [pi_cx + pi_board[0] / 2 + 2, pi_cy - 3, pz + 6]], 4);
    // LiDAR（USB 変換基板）→ Pi の USB
    color("silver") path([[lidar_cx - 6, box_y0 + 22, lt + 3], [rear_tie[0] - 3, box_y0 + 22, lt + 3], [rear_tie[0] + 1, rear_tie[1], lt + 4], [lidar_cx + 30, box_y0 - 6, lt],
                          [pi_cx + pi_board[0] / 2 + 12, pi_cy + 20, pz + 12], [pi_cx + pi_board[0] / 2 + 2, pi_cy + 19, pz + 12]], 3.5);
    // バッテリー（PD 12V）→ DROK の入力（左端）
    color("red") path([[box_x0 + box_x + 3, box_cy - 10, floor_t + 8], [box_x0 + box_x + 3, box_y0 - 12, flange_t + 3],
                       [drok_cx - drok_board[0] / 2 - 4, box_y0 - 20, flange_t + 3],
                       [drok_cx - drok_board[0] / 2 - 4, drok_by - 6, drok_z0 + 5]], 3);
    // DROK の USB-A（右端）→ Pi の USB-C（後ろ）
    color("black") path([[drok_cx + drok_board[0] / 2 + 2, drok_by - 6, drok_z0 + 6], [drok_cx + drok_board[0] / 2 + 18, drok_by - 6, drok_z0 + 6],
                         [134, 108, flange_t + 5], [146, 95, 2],
                         [146, 10, 2], [pi_cx - pi_board[0] / 2 + 11.2, 8, 2],
                         [pi_cx - pi_board[0] / 2 + 11.2, pi_cy - pi_board[1] / 2 - 10, pz + 1.6]], 4);
    // IMU / 前後の VL53L1X → GPIO ヘッダー（前端）
    hy = pi_cy + pi_board[1] / 2 - 3.5;
    ht = pi_z + pi_stack_h + 4;
    // IMU: ピンヘッダー（下向き）→ 台の後ろの開いた側 → ふたの後ろ端から Pi の前の GPIO へ
    color("purple") path([[imu_c[0], imu_c[1] - imu_board[0] / 2 + 1.3, lt + imu_lift - 14], [imu_c[0], box_y0 + 3, lt + 4],
                          [imu_c[0] - 4, box_y0 - 4, ht + 2],
                          [pi_cx + 20, hy + 2, ht], [pi_cx - 34, hy + 2, ht]], 2.5);
    color("yellow") {
        path([[plate_w / 2, plate_l + tof_dy - 20, flange_t + 12], [13, plate_l + 4, flange_t + 6], [13, 168, flange_t + 5],
              [13, 128, flange_t + 5], [20, 100, 10], [pi_cx - 25, hy + 2, ht]], 2.5);
        path([[plate_w / 2, 20 - tof_dy, flange_t + 12], [plate_w / 2, 10, pi_z + pi_stack_h + 8],
              [pi_cx - 20, hy, ht + 2]], 2.5);
    }
}


// ---- 上面カバー（天板全体を覆う。左右のフックで天板の端の裏に引っかける。工具不要）----
// LiDAR のヘッドはカバーの上面から上に出る。LiDAR の外形より大きく抜いてあるので、LiDAR を付けたまま真上へ抜ける
cov_t = 2.4;              // 壁と上面の厚さ
cov_clr = 0.3;            // 天板の端とのすき間
cov_top = 80;             // 上面の裏（Pi 3 段の上端 54 mm の上に 26 mm）
cov_y0 = -1;              // 後ろの壁の内側（後ろのバンパーの上）
cov_y1 = 206.5;           // 前の壁の内側（LiDAR の前端 205 mm の前）
cov_skirt_z = 11;         // 前後の壁の下端。バンパー上面（7 mm）との間に VL53L1X の線を通す
cov_ch = 6;               // 上面の角の面取り
cov_edge = 1;             // 手で触る縁（外側の角、下端、充電口）の面取り
cov_rear = [38.4, 18];    // 後ろの斜面: 後ろの壁の上端の高さ、斜面が上面に届く Y（外側）
cov_hook_y = [54, plate_l - 30];  // フックの位置（左右とも。後ろは Pi の電源ボタンの穴をよけて天板の後ろの端から 54 mm、前は前の端から 30 mm）
cov_hook_w = 12;
cov_hook_root = 28;       // フックの付け根の高さ（ここから下がたわむ）
cov_hook_lip = 2;         // 天板の裏にかかる深さ
// 天板の上面に載せる受け（[左, 右]）。フックの両隣と中央
cov_stop_y = [[42, 66, 100, 158, 182], [42, 66, 100, 158, 182]];
cov_rib_x = [[21, 25], [129, 133]];  // ふたの前後の縁をはさんで前後の位置を決めるリブ
cov_chg = [113, 185, 5, 38];         // 右側面の充電口（Y0, Y1, Z0, Z1）: バッテリーのポート側の端
cov_vent = [9, 79, 28, 72];          // Pi の上の六角の通気口の範囲

cov_ox0 = -cov_clr - cov_t;
cov_ox1 = plate_w + cov_clr + cov_t;
cov_oy0 = cov_y0 - cov_t;
cov_oy1 = cov_y1 + cov_t;
cov_oz = cov_top + cov_t;

// 2 眼カメラのレンズを通す前の窓（X0, X1, 上端の Z）。レンズの先はカバーより前に出るので、壁の下の端から切り込み、カバーを真上へ抜けるようにする
function cov_scam_slot(dx) = let(r = scam_lens_d / 2 + scam_slot_clr) [scam_cx + dx - r, scam_cx + dx + r, scam_lens_zc + r];

// LiDAR を真上へ抜くための外形（ヘッド、柱 4 本、前のモーター側）
module cover_lidar_cut() {
    hull() {
        translate([lidar_cx, lidar_cy]) circle(r = 38);
        for (p = lidar_holes_rel) translate(lidar_hole(p)) circle(r = 8);
        translate([lidar_cx, lidar_cy + 47]) circle(r = 16);
    }
}

// o = 0: 外形、o = cov_t: 内側（面取りと後ろの斜面を厚さぶん内へずらす）
module cover_solid(o) {
    c = cov_ch; e = o * sqrt(2) - o;
    dy = cov_rear[1] - cov_oy0; dz = cov_oz - cov_rear[0]; L = sqrt(dy * dy + dz * dz);
    ny = dz / L; nz = -dy / L;
    rz = cov_rear[0] + o * nz + (o - o * ny) / dy * dz;          // 斜面と後ろの壁の交点の Z
    ry = cov_oy0 + o * ny + (cov_oz - o - cov_rear[0] - o * nz) / dz * dy;  // 斜面と上面の交点の Y
    intersection() {
        rotate([90, 0, 0]) translate([0, 0, -300]) linear_extrude(600)
            polygon([[cov_ox0 + o, -20], [cov_ox1 - o, -20], [cov_ox1 - o, cov_oz - c - e],
                     [cov_ox1 - c - e, cov_oz - o], [cov_ox0 + c + e, cov_oz - o], [cov_ox0 + o, cov_oz - c - e]]);
        rotate([90, 0, 90]) translate([0, 0, -300]) linear_extrude(600)
            polygon([[cov_oy0 + o, -20], [cov_oy1 - o, -20], [cov_oy1 - o, cov_oz - c - e],
                     [cov_oy1 - c - e, cov_oz - o], [ry, cov_oz - o], [cov_oy0 + o, rz]]);
    }
}

// 外形をこの形でふくらませて縁を面取りする（軸方向と 45° 方向に 1 ずつ）
module cover_edge_k() {
    a = 1 / sqrt(2);
    hull() for (p = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
                     [a, 0, a], [-a, 0, a], [a, 0, -a], [-a, 0, -a], [0, a, a], [0, -a, a], [0, a, -a], [0, -a, -a]])
        translate(p * cov_edge) cube(0.001, center = true);
}

module cover_chg_profile(d) {
    translate([plate_w - 1, cov_chg[0], cov_chg[2]]) rotate([0, 90, 0]) translate([-(cov_chg[3] - cov_chg[2]), 0, 0])
        children();
}

// 充電口の下の細い帯にあるフックは、たわませる切れ目を入れず固定の爪にする
function cov_hook_fixed(x, y) = x == 1 && y > cov_chg[0] && y < cov_chg[1];

module cover_hook() {
    // 天板の裏にかかる爪（上面が掛かり面、下面は押し込むと外へ逃げる斜面）と、外へ引く指かけ
    rotate([90, 0, 0]) linear_extrude(cov_hook_w, center = true) {
        polygon([[-cov_clr, -plate_t - 0.2], [cov_hook_lip, -plate_t - 0.2], [-cov_clr, -plate_t - 3.5]]);
        polygon([[cov_ox0, -plate_t - 3.5], [cov_ox0 - 2.5, -plate_t - 3.5], [cov_ox0, -plate_t - 1]]);
        translate([cov_ox0, -plate_t - 3.5]) square([cov_t, 3.5]);
    }
}

module cover() {
    difference() {
        union() {
            difference() {
                minkowski() { cover_solid(cov_edge); cover_edge_k(); }
                cover_solid(cov_t);
                // 下端の外側の縁の面取り（左右は天板の裏の高さ、前後は壁の下端）
                for (x = [cov_ox0, cov_ox1]) translate([x, (cov_oy0 + cov_oy1) / 2, -plate_t])
                    rotate([0, 45, 0]) cube([cov_edge * sqrt(2), cov_oy1 - cov_oy0 + 2, cov_edge * sqrt(2)], center = true);
                for (y = [cov_oy0, cov_oy1]) translate([plate_w / 2, y, cov_skirt_z])
                    rotate([45, 0, 0]) cube([cov_ox1 - cov_ox0 + 2, cov_edge * sqrt(2), cov_edge * sqrt(2)], center = true);
                translate([cov_ox0 - 1, cov_oy0 - 1, -50]) cube([cov_ox1 - cov_ox0 + 2, cov_oy1 - cov_oy0 + 2, 50 - plate_t]);
                // 前後の壁は天板の上面より上だけ（下はバンパー）
                for (y = [[cov_oy0 - 1, cov_y0 + 0.01], [cov_y1 - 0.01, cov_oy1 + 1]])
                    translate([-cov_clr, y[0], -10]) cube([plate_w + 2 * cov_clr, y[1] - y[0], 10 + cov_skirt_z]);
            }
            // フック
            for (x = [0, 1], y = cov_hook_y)
                translate([x ? plate_w : 0, y, 0]) mirror([x, 0, 0]) cover_hook();
            // 天板の上面に載る受け（下は平ら、上は 45° で壁へ）
            for (x = [0, 1], y = cov_stop_y[x])
                translate([x ? plate_w : 0, y, 0]) mirror([x, 0, 0])
                    rotate([90, 0, 0]) linear_extrude(8, center = true)
                        polygon([[-cov_clr - 0.01, 0], [2.5, 0], [2.5, 0.8], [-cov_clr - 0.01, 3.6]]);
            // ふたの前後の縁をはさむリブ
            for (r = cov_rib_x, y = [[box_y0 - 5.3, box_y0 - 2.3], [box_y0 + box_y + 2.3, box_y0 + box_y + 5.3]])
                translate([r[0], y[0], box_h + 1]) cube([r[1] - r[0], y[1] - y[0], cov_top - box_h - 0.99]);
        }
        // フックのまわりの切れ目（1 mm）
        for (x = [0, 1], y = cov_hook_y) if (!cov_hook_fixed(x, y))
            translate([x ? plate_w : 0, y, 0]) mirror([x, 0, 0])
                for (s = [-1, 1]) translate([cov_ox0 - 3, s > 0 ? cov_hook_w / 2 : -cov_hook_w / 2 - 1, -10])
                    cube([cov_t + 3 + cov_clr + 0.01, 1, 10 + cov_hook_root]);
        // LiDAR
        translate([0, 0, box_h + lid_t + 1]) linear_extrude(cov_oz) cover_lidar_cut();
        // 充電口（右側面）
        cover_chg_profile() linear_extrude(cov_t + cov_clr + 2) offset(r = 4) offset(delta = -4) square([cov_chg[3] - cov_chg[2], cov_chg[1] - cov_chg[0]]);
        hull() for (d = [[cov_ox1 - cov_edge - (plate_w - 1), 0], [cov_ox1 - (plate_w - 1) + 0.01, cov_edge + 0.01]])
            cover_chg_profile() translate([0, 0, d[0]]) linear_extrude(0.01)
                offset(delta = d[1]) offset(r = 4) offset(delta = -4) square([cov_chg[3] - cov_chg[2], cov_chg[1] - cov_chg[0]]);
        // 2 眼カメラの窓（前の壁。左右のレンズごと。間の基板と USB は壁で守る）
        for (dx = [-scam_baseline / 2, scam_baseline / 2]) let (w = cov_scam_slot(dx), r = (w[1] - w[0]) / 2)
            translate([0, cov_oy1 + 1, 0]) rotate([90, 0, 0]) linear_extrude(cov_t + 3) hull() {
                translate([w[0] + r, w[2] - r]) circle(r = r);
                translate([w[0], -20]) square([w[1] - w[0], 1]);
            }
        // バッテリーの残量表示の窓（上面。縁は面取り）
        let (d = disp_rect(), m = disp_cov_m) {
            translate([d[0] - m, d[1] - m, cov_top - 1]) cube([d[2] - d[0] + 2 * m, d[3] - d[1] + 2 * m, cov_t + 2]);
            hull() {
                translate([d[0] - m, d[1] - m, cov_oz - cov_edge]) cube([d[2] - d[0] + 2 * m, d[3] - d[1] + 2 * m, 0.01]);
                translate([d[0] - m - cov_edge, d[1] - m - cov_edge, cov_oz]) cube([d[2] - d[0] + 2 * (m + cov_edge), d[3] - d[1] + 2 * (m + cov_edge), 1]);
            }
        }
        // 前面のスリット（飾り。カメラの窓の下）
        for (i = [0 : 1]) translate([scam_cx - 14, cov_y1 - 1, 20 + i * 7]) cube([28, cov_t + 2, 3]);
        // 六角の通気口
        for (i = [0 : 12], j = [0 : 8]) {
            x = cov_vent[0] + i * 6 + (j % 2) * 3;
            y = cov_vent[2] + j * 5.2;
            if (x <= cov_vent[1] && y <= cov_vent[3]) translate([x, y, cov_top - 1]) cylinder(r = 2.4, h = cov_t + 2, $fn = 6);
        }
        // Pi の電源ボタン（左の壁）
        let (y0 = pi_cy - pi_board[1] / 2 + pi_btn_y[0], y1 = pi_cy - pi_board[1] / 2 + pi_btn_y[1],
             z0 = pi_z + pi_btn_z[0], z1 = pi_z + pi_btn_z[1], r = 3)
            hull() for (y = [y0 + r, y1 - r], z = [z0 + r, z1 - r])
                translate([cov_ox0 - 2, y, z]) rotate([0, 90, 0]) cylinder(r = r, h = cov_t + cov_clr + 2.5);
        // 文字
        translate([140, 150, cov_oz - 0.8]) rotate([0, 0, 90]) linear_extrude(1)
            text("AI-CAR", size = 9, halign = "center", valign = "center", font = "Liberation Sans:style=Bold");
    }
}

if (part == "box") box();
else if (part == "lid") lid();
else if (part == "pi_base") pi_base();
else if (part == "cam_mount") translate([0, 0, -scam_mnt_y0]) rotate([90, 0, 0]) cam_mount();
else if (part == "bumper") translate([0, 0, flange_t + bump_t]) mirror([0, 0, 1]) bumper();
else if (part == "cover") translate([0, cov_oy0 + cov_oy1, cov_oz]) rotate([180, 0, 0]) cover();
else if (part == "cover_asm") cover();
else {
    plate();
    color("royalblue") box();
    batteries();
    color("orange") translate([0, 0, box_h]) lid();
    lidar_ghost();
    color("seagreen") pi_base();
    pi_ghost();
    camera_ghost();
    drok_ghost();
    sensor_ghost();
    if (show_cables) cable_ghost();
    color("dimgray") both_bumpers();
    tof_ghost();
    if (show_cover) color("#3c4452") cover();
}

echo(box_outer = [box_x, box_y, box_h], interior = [in_x - end_fill, in_y, in_z],
     lidar_holes = [for (p = lidar_holes_rel) lidar_hole(p)],
     lidar_top_z = box_h + lid_t + lidar_tower_h,
     scam_board_xz = [scam_ends[0], scam_ends[1], scam_z0, scam_z1], scam_holes = scam_holes, scam_lens_center = [scam_cx, scam_tip_y, scam_lens_zc],
     scam_board_y = [scam_back_y, scam_front], scam_mnt_x = scam_mnt_x, scam_mnt_y = [scam_mnt_y0, scam_mnt_y0 + scam_mnt_t], scam_ear_top_z = scam_ear_top,
     cov_scam_slots = [cov_scam_slot(-scam_baseline / 2), cov_scam_slot(scam_baseline / 2)], cov_front_outer_y = cov_oy1, pi_holes = pi_pts, pi_board_x = [pi_cx - pi_board[0] / 2, pi_cx + pi_board[0] / 2],
     pi_btn_hole_yz = [pi_cy - pi_board[1] / 2 + pi_btn_y[0], pi_cy - pi_board[1] / 2 + pi_btn_y[1], pi_z + pi_btn_z[0], pi_z + pi_btn_z[1]], cov_inner_x = [-cov_clr, plate_w + cov_clr],
     pi_stack_top_z = pi_z + pi_stack_h,
     pi_stack_front_y = pi_cy + pi_board[1] / 2,
     lidar_head_rear_y = lidar_cy - 35, box_flange_rear_y = flange_y0,
     imu_board_z = box_h + lid_t + imu_lift, imu_holder_top_z = box_h + lid_t + imu_lift + 1.6 + 0.3 + imu_lip_t,
     lidar_core_bottom_z = box_h + lid_t + lidar_tower_h - lidar_core_below,
     tof_front = tof_c(), tof_rear = [plate_w - tof_c()[0], plate_l - tof_c()[1]],
     tof_center_z = flange_t - tof_drop, tof_tilt = tof_tilt,
     screw_mode = screw_mode, tap_d = [tap_d(2), tap_d(2.5), tap_d(3)], insert_d = [insert_d(2), insert_d(2.5), insert_d(3)],
     bat_disp_rect = disp_rect(),
     lidar_tower_x_max = lidar_cx + 28 + lidar_tower_d / 2, lidar_motor_x_max = lidar_cx + lidar_motor_d / 2,
     lidar_motor_bottom_z = box_h + lid_t + lidar_tower_h - lidar_below, lid_top_z = box_h + lid_t,
     motor_recess = [for (p = motor_scr_pts) [p[0] - recess_clr, p[1] - recess_clr, p[0] + motor_scr_sq + recess_clr, p[1] + motor_scr_sq + recess_clr]], recess_t = recess_t,
     lid_screw_head_x_min = boss_pts[1][0] - head_d(3) / 2,
     drok_x = [drok_cx - drok_board[0] / 2, drok_cx + drok_board[0] / 2], drok_front_y = drok_by - drok_h,
     drok_top_z = drok_z0 + drok_board[1],
     bump_out = bump_out, bump_gap = bump_gap, stop_len = bump_gap - bump_travel, spring_strain = spring_strain);
