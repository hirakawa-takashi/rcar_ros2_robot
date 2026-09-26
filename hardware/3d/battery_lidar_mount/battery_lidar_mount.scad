// AI-CAR バッテリー 2 個 + RPLIDAR A1M8 マウント
// 座標: 天板 154 x 200 mm の左下角が原点。X = 横幅方向、Y = 前後方向（Y=200 側が前 / LiDAR 側）
// 天板の穴位置は Y 方向に対称なので、前後どちら向きでも同じ穴に合う。
//
// 出力: openscad -D 'part="box"' -o box.stl battery_lidar_mount.scad
//       openscad -D 'part="lid"' -o lid.stl battery_lidar_mount.scad
//       openscad -D 'part="pi_base"' -o pi_base.stl battery_lidar_mount.scad
//       openscad -D 'part="bumper"' -o bumper.stl battery_lidar_mount.scad（前後共通、2 個印刷）
//       openscad -D 'part="cover"' -o cover.stl battery_lidar_mount.scad（上面カバー、上面を下にした向き）

part = "assembly"; // "box" | "lid" | "pi_base" | "bumper" | "cover" | "assembly"
show_cables = false;  // 組立図に配線経路の目安を描く
show_cover = false;   // 組立図に上面カバーを描く

$fn = 48;

// ---- 天板（ユーザー提供の CAD 図面）----
plate_w = 154;
plate_l = 200;
plate_t = 3;
plate_holes = [[11, 11], [143, 11], [11, 189], [143, 189],
               [9, 90], [145, 90], [9, 110], [145, 110]];
mount_holes = [[11, 189], [143, 189], [9, 110], [145, 110]];
mount_hole_d = 4.4;

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
module screw_hole(m, depth) {
    d = nut_hole_d(m);
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
bat_x = 97.6;   // 長辺（X 方向に置く。ポートのある短辺が開口側を向く）
bat_y = 69;
bat_z = 16.2;
bat_n = 2;
clr = 0.6;      // 片側のすき間

// ---- 箱 ----
wall = 2.5;
floor_t = 3;          // フランジと同じ厚さ（バッテリーを段差なしで引き出せる）
flange_t = 3;
flange_y0 = 104;
flange_y1 = 196;
flange_x0 = 3;
flange_x1 = 151;
flange_r = 4;
open_end = "right";   // バッテリー出し入れ・ケーブル側: "right"(+X) / "left"(-X)
strap_w = 12;         // 面ファスナー / 結束バンド用スロットの高さ
strap_len = 3;        // スロットの幅（ストラップ厚み方向）

in_x = bat_x + 2 * clr;
in_y = bat_y + 2 * clr;
in_z = bat_n * bat_z + 1.0;
box_x = in_x + 2 * wall;
box_y = in_y + 2 * wall;
box_h = floor_t + in_z;
box_cx = plate_w / 2;
box_cy = (flange_y0 + flange_y1) / 2;
box_x0 = box_cx - box_x / 2;
box_y0 = box_cy - box_y / 2;

// ふた固定用ボス（箱の外側 4 隅、M3 タッピング）
boss_d = 8;
boss_pts = [[box_x0 - boss_d / 2 + 1.5, box_y0 + 6],
            [box_x0 + box_x + boss_d / 2 - 1.5, box_y0 + 6],
            [box_x0 - boss_d / 2 + 1.5, box_y0 + box_y - 6],
            [box_x0 + box_x + boss_d / 2 - 1.5, box_y0 + box_y - 6]];

// ---- カメラ（Raspberry Pi Camera Module 3、ふたの前端に立てた板に正立で取付）----
// 基板 25 x 23.86 mm、M2 穴 21 x 12.5 mm（ケーブルと反対の上端から 2 / 14.5 mm）
// ケーブルは基板の下端から出て、水平に後ろへ折れ、板の下の窓をくぐってふたの上を通る
cam_w = 25;
cam_h = 23.86;
cam_hole_dx = 21;
cam_hole_top = [2, 14.5]; // 基板の上端からの距離
cam_lens_z = 9.46;        // レンズ中心の基板下端からの高さ（上端から 14.4 mm）
cam_boss_d = 5.5;
cam_boss_l = 4;
cam_fpc_gap = 4;          // ふたの上面から基板の下端まで（ケーブルを下から後ろへ曲げる分）
cam_plate_t = 3;          // カメラを留める板
cam_plate_w = 28;
cam_rib_l = 12;           // 板の後ろの補強リブの長さ
cam_cx = 113;             // 右寄せ: フラットケーブルを LiDAR のモーター（ふたの上 1.5 mm まで下がる）と柱の横に通す
fpc_w = 16;               // カメラ用フラットケーブル（Standard 側の幅）
fpc_clip_y = [125, 160];  // ふたの上でケーブルを押さえるブリッジ
tie_w = 4.5;              // 結束バンド（幅 3.6 mm まで）を通すトンネル
tie_h = 2;

// ---- ラズパイ台（Raspberry Pi 5、天板の後ろ側）----
pi_mount_holes = [[11, 11], [143, 11], [9, 90], [145, 90]];
pi_cx = 77;               // 基板中心
pi_cy = 50;
pi_board = [85, 56];
pi_hole_dx = 58;
pi_hole_dy = 49;
pi_hole_off = 3.5;        // GPIO 側の短辺からの距離（USB / LAN は +X 側）
pi_boss_d = 6;
pi_boss_h = 6;
pi_base_t = 3;
pi_stack_h = 45;          // Pi 5 + AI HAT+ + Motor HAT の 3 段（実測。台のぶん高くなる側で見積もる）
pi_pts = [for (dx = [0, pi_hole_dx], dy = [0, pi_hole_dy])
          [pi_cx - pi_board[0] / 2 + pi_hole_off + dx, pi_cy - pi_hole_dy / 2 + dy]];

// ---- ラズパイ台の左側: IMU（GY-BNO055）と液晶（ZJY-IPS130）----
wing = [1, 18, 34, 80];   // x0, y0, x1, y1（液晶の枠と左右の支えの下まで広げる）
imu_board = [12, 20];     // 実物に合わせて変更（長辺を前後方向に置く）
imu_c = [18, 64];         // 液晶より前（基板の上端が天板から約 30 mm になるので、後ろ向きの画面を隠さないよう前に置く）
imu_lift = 25;            // 基板の下の空き。ピンヘッダーは下向きで、ジャンパー線のコネクターを下に収める
imu_post = 2;             // 四隅の受けが基板の角を受ける幅
imu_cable_hole = [13, 3, 19];  // Pi 側（+X）の壁のケーブル穴（幅、台の上面からの下端、高さ）
imu_wall = 1.6;
lcd_board = [27.5, 39, 1.6];  // 幅 x 高さ x 基板厚（実物に合わせて変更）
lcd_cx = 17.5;
lcd_y = 24;               // 画面の下端（後ろ向き）。ピンは上端の裏側で、配線は前上の GPIO ヘッダーへ。上端は前へ倒れて IMU の上に来る
lcd_tilt = 30;            // 垂直から後ろへ倒す角度
lcd_lip = 1;              // 基板の左右の縁を押さえる幅

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
spring_t = 1.2;           // 板厚（PETG 推奨）
spring_h = 8;
spring_clr = 1;           // ストッパーで止まったときに残るすき間
post_w = 3;
stop_x = [4, 77, 150];
bump_gap = 2 * (bump_travel + spring_clr) + spring_t;
bump_out = bump_base_d + bump_gap + bump_face_t;
// 両端を固定した板ばね（一端がずれる）の曲げひずみ = 3 t δ / L²
spring_strain = 3 * spring_t * bump_travel / pow(spring_x[0][1] - spring_x[0][0] - post_w, 2);
tof_board = [25, 10.7, 1.6];  // VL53L1X 基板（Dovhmoh: 25 x 10.7、長辺を横幅方向に置く）
tof_lip = 1.5;            // 基板を受ける縁の幅（その内側は窓）
tof_tilt = 30;            // 真下から進行方向へ傾ける角度
tof_drop = 8;             // 受けの中心をバンパー下面から下げる量
tof_face_t = 2;

// ---- ふた + LiDAR 台 ----
lid_t = 4;
lidar_tower_h = 28;     // LiDAR 取付面より下に出ている部分（約 26.5 mm）をかわす高さ
lidar_tower_d = 8.5;
lidar_floor = 4;        // 柱の上端に残す厚さ。LiDAR 底面の M2.5 ねじ穴へ、ふたの裏から柱の中を通したネジで留める
lidar_motor_front = true; // LiDAR のモーター側（細い側）を前（+Y）に向ける
// RPLIDAR A1M8 取付穴（回転中心基準、データシート Figure 5-2）
//   ヘッド側 2 穴: 中心から 28 mm、間隔 56 mm / モーター側 2 穴: 中心から 42 mm、間隔 40 mm
lidar_holes_rel = [[-28, 28], [28, 28], [-20, -42], [20, -42]];
lidar_dx = -8;          // 左へ寄せ、右側にカメラのフラットケーブルの通り道を空ける（カメラとの位置の差は dashboard.yaml の camera_offset_x_m / _y_m に反映すること）
lidar_cx = box_cx + lidar_dx;
lidar_motor_d = 32;     // 取付面より下に出るモーター（回転中心から 45.5 mm、取付面から 26.5 mm 下まで）
lidar_motor_off = 45.5;
lidar_below = 26.5;
lidar_core_below = 8;   // ヘッドの下の基板部（約 60 x 60 mm）が取付面から下に出る量（図からの読み取り）
lidar_cy = box_cy + (lidar_motor_front ? -7 : 7);

function lidar_hole(p) = lidar_motor_front
    ? [lidar_cx + p[0], lidar_cy - p[1]]
    : [lidar_cx + p[0], lidar_cy + p[1]];

cam_by = box_y0 + box_y + 2;          // 板の前面（ふたの前端）
cam_z0 = box_h + lid_t + cam_fpc_gap;  // 基板の下端
cam_top = cam_z0 + cam_h + 1;          // 板の上端（上の穴のボスを覆う）
function cam_pts() = [for (dx = [-cam_hole_dx / 2, cam_hole_dx / 2], z = cam_hole_top) [cam_cx + dx, cam_z0 + cam_h - z]];

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

tie_pts_box = [[13, 135], [13, 165], [134, 108]];

// DROK 降圧コンバーター（12V → 5.2V、B09JKHD3SH）。取付穴がないので、箱の後ろの壁に立てて結束バンドで留める
drok_board = [63, 27, 1.6];   // 商品画像の寸法
drok_h = 13;                  // 基板の裏から部品の頂点まで（推定）
drok_gap = 3;                 // 基板の裏と壁のすき間（裏のピンの逃げ）
drok_cx = box_cx - 4;         // FPC の通り道（X = cam_cx ± fpc_w / 2）より左
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

module box() {
    difference() {
        union() {
            rrect(flange_x0, flange_y0, flange_x1, flange_y1, flange_r, flange_t);
            translate([box_x0, box_y0, 0]) cube([box_x, box_y, box_h]);
            for (p = boss_pts) translate([p[0], p[1], 0]) cylinder(d = boss_d, h = box_h);
            for (p = tie_pts_box) translate([p[0], p[1], flange_t - 0.01]) tie_mount();
            drok_holder();
        }
        // バッテリー収納部
        translate([box_x0 + wall, box_y0 + wall, floor_t]) cube([in_x, in_y, in_z + 1]);
        // 開口側の端（全面開口）
        ox = open_end == "right" ? box_x0 + box_x - wall - 1 : box_x0 - 1;
        translate([ox, box_y0 + wall, floor_t]) cube([wall + 2, in_y, in_z + 1]);
        // 反対側の端: 押し出し用の窓
        cx = open_end == "right" ? box_x0 - 1 : box_x0 + box_x - wall - 1;
        translate([cx, box_cy - 20, floor_t + 4]) cube([wall + 2, 40, in_z - 8]);
        // 固定用ストラップのスロット（上下バッテリーの境目の高さ、開口側寄り）
        sx = open_end == "right" ? box_x0 + box_x - wall - 12 : box_x0 + wall + 12 - strap_len;
        for (y = [box_y0 - 1, box_y0 + box_y - wall - 1])
            translate([sx, y, floor_t + bat_z - strap_w / 2 + 0.5]) cube([strap_len, wall + 2, strap_w]);
        // 通気スロット（長辺）
        for (i = [-2 : 2])
            translate([box_cx + i * 12 - 2.5, box_y0 - 1, floor_t + 6]) cube([5, wall + 2, in_z - 12]);
        // 床の肉抜き
        translate([box_x0 + wall + 12, box_y0 + wall + 10, -1]) cube([in_x - 24, in_y - 20, floor_t + 2]);
        // 天板への取付穴
        for (p = mount_holes) translate([p[0], p[1], -1]) cylinder(d = mount_hole_d, h = flange_t + 2);
        // ふた固定用の下穴
        for (p = boss_pts) translate([p[0], p[1], box_h]) screw_hole(3, 12);
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
            for (y = fpc_clip_y) translate([cam_cx, y, lid_t - 0.01]) fpc_clip();
            cam_mount();
            // LiDAR のハーネスと USB ケーブルの固定（ヘッドの柱の間、ふたの後ろ端）
            translate([lidar_cx, box_y0 + 7, lid_t - 0.01]) rotate([0, 0, 90]) tie_mount();
        }
        for (p = boss_pts) translate([p[0], p[1], 0]) clear_hole(3, lid_t);
        for (p = lidar_holes_rel) {
            q = lidar_hole(p);
            translate([q[0], q[1], 0]) clear_hole(2.5, lid_t + lidar_tower_h, lid_t + lidar_tower_h - lidar_floor);
        }
        cam_mount_cut();
    }
}

// カメラを留める板（ふたの座標。板の下の窓にフラットケーブルを通す）
module cam_mount() {
    x0 = cam_cx - cam_plate_w / 2;
    y0 = cam_by - cam_plate_t;
    translate([x0, y0, lid_t - 0.01]) cube([cam_plate_w, cam_plate_t, cam_top - box_h - lid_t + 0.01]);
    for (p = cam_pts()) translate([p[0], cam_by - 0.01, p[1] - box_h]) rotate([-90, 0, 0]) cylinder(d = cam_boss_d, h = cam_boss_l + 0.01);
    for (x = [x0, x0 + cam_plate_w - 2]) translate([x, 0, 0]) rotate([90, 0, 90]) linear_extrude(2)
        polygon([[y0 + 0.01, lid_t - 0.01], [y0 - cam_rib_l, lid_t - 0.01], [y0 + 0.01, lid_t + 18]]);
}

module cam_mount_cut() {
    translate([cam_cx - fpc_w / 2 - 1, cam_by - cam_plate_t - 1, lid_t - 0.02]) cube([fpc_w + 2, cam_plate_t + 2, cam_fpc_gap + 3]);
    for (p = cam_pts()) translate([p[0], cam_by + cam_boss_l, p[1] - box_h]) rotate([-90, 0, 0]) screw_hole(2, 6);
}

// フラットケーブルを下に差し込むブリッジ（すき間 1.5 mm）
module fpc_clip() {
    w = fpc_w + 1.5;
    difference() {
        translate([-w / 2 - 2, -3, 0]) cube([w + 4, 6, 2.7]);
        translate([-w / 2, -4, -0.01]) cube([w, 8, 1.5]);
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
            rrect(wing[0], wing[1], wing[2], wing[3], 4, pi_base_t);
            // 後ろのバンパーの爪を受ける平らな端（前のフランジの縁と同じ位置 Y = plate_l - flange_y1）
            for (a = bump_arm_x) rrect(a[0], plate_l - flange_y1, a[1], plate_l - bump_arm_y0, 1, pi_base_t);
            imu_holder();
            lcd_holder();
        }
        for (p = pi_mount_holes) translate([p[0], p[1], -1]) cylinder(d = mount_hole_d, h = pi_base_t + 2);
        for (p = pi_pts) translate([p[0], p[1], pi_base_t + pi_boss_h]) screw_hole(2.5, 8);
        // 配線通し・通気（後ろ側は台の縁まで開け、天板の開口から上がるモーター線を Pi の後ろへ出す）
        translate([pad[0] + 10, pad[1] - 1, -1]) cube([pad[2] - pad[0] - 20, pad[3] - pad[1] - 9, pi_base_t + 2]);
    }
}

module imu_holder() {
    ix = imu_board[0] + 0.6;
    iy = imu_board[1] + 0.6;
    ox = ix + 2 * imu_wall;
    oy = iy + 2 * imu_wall;
    translate([imu_c[0], imu_c[1], pi_base_t - 0.01]) {
        // 四方の壁（上端は基板の上面より 1 mm 上で、基板の位置決めを兼ねる）。下向きのピンの線は Pi 側の壁の穴から出す
        difference() {
            translate([-ox / 2, -oy / 2, 0]) cube([ox, oy, imu_lift + 1.6 + 1]);
            translate([-ix / 2, -iy / 2, 0.01]) cube([ix, iy, imu_lift + 5]);
            translate([ix / 2 - 1, -imu_cable_hole[0] / 2, imu_cable_hole[1]]) cube([imu_wall + 2, imu_cable_hole[0], imu_cable_hole[2]]);
        }
        // 四隅の受け（上端で基板の角を受ける）
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx > 0 ? ix / 2 - imu_post : -ix / 2, sy > 0 ? iy / 2 - imu_post : -iy / 2, 0])
                cube([imu_post, imu_post, imu_lift]);
    }
}

module lcd_floor_local() {
    W = lcd_board[0]; t = lcd_board[2];
    translate([-W / 2 - 2.3, -2, -2]) cube([W + 4.6, t + 0.3 + 3.2, 2]);
}

module lcd_frame_local() {
    W = lcd_board[0]; H = lcd_board[1]; t = lcd_board[2];
    difference() {
        translate([-W / 2 - 2.3, -2, -2]) cube([W + 4.6, t + 0.3 + 3.2, H + 2]);
        translate([-W / 2 - 0.3, 0, 0]) cube([W + 0.6, t + 0.3, H + 5]);
        translate([-W / 2 + lcd_lip, t, 0]) cube([W - 2 * lcd_lip, 10, H + 5]);
        translate([-W / 2 + 1.5, -10, 0]) cube([W - 3, 10, H + 5]);
    }
}

module lcd_holder() {
    H = lcd_board[1];
    translate([lcd_cx, lcd_y, pi_base_t - 0.01]) {
        rotate([-lcd_tilt, 0, 0]) rotate([0, 0, 180]) translate([0, 0, 2]) lcd_frame_local();
        // 枠の底の下を台まで埋める（底が浮かないように）
        hull() {
            rotate([-lcd_tilt, 0, 0]) rotate([0, 0, 180]) translate([0, 0, 2]) lcd_floor_local();
            linear_extrude(0.01) projection() rotate([-lcd_tilt, 0, 0]) rotate([0, 0, 180]) translate([0, 0, 2]) lcd_floor_local();
        }
        // 左右の支え
        for (sx = [-1, 1]) translate([sx * (lcd_board[0] / 2 + 1.15) - 1.15, 0, 0])
            rotate([90, 0, 90]) linear_extrude(2.3)
                polygon([[0, 0], [(H + 2) * sin(lcd_tilt) + 2, 0], [(H + 2) * sin(lcd_tilt), (H + 2) * cos(lcd_tilt)]]);
    }
}

function tof_c() = [plate_w / 2, plate_l + 9.5];

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
            translate([-tof_board[0] / 2 + tof_lip, -tof_board[1] / 2 + tof_lip, -tof_face_t - 1])
                cube([tof_board[0] - 2 * tof_lip, tof_board[1] - 2 * tof_lip, tof_face_t + 2]);
        }
    }
}

module tof_ghost() {
    for (r = [0, 180]) translate([plate_w / 2, plate_l / 2, 0]) rotate([0, 0, r]) translate([-plate_w / 2, -plate_l / 2, 0])
        tof_at(flange_t) translate([-tof_board[0] / 2, -tof_board[1] / 2, 0.05]) color("black") cube(tof_board);
}

module both_bumpers() {
    bumper();
    translate([plate_w, plate_l, 0]) rotate([0, 0, 180]) bumper();
}

module pi_ghost() {
    z = pi_base_t + pi_boss_h;
    color("green", 0.8) translate([pi_cx - pi_board[0] / 2, pi_cy - pi_board[1] / 2, z]) cube([pi_board[0], pi_board[1], 1.6]);
    color("darkslateblue", 0.5) translate([pi_cx - pi_board[0] / 2, pi_cy - pi_board[1] / 2, z + 1.6]) cube([65, pi_board[1], pi_stack_h - 1.6]);
}

module sensor_ghost() {
    color("purple") translate([imu_c[0] - imu_board[0] / 2, imu_c[1] - imu_board[1] / 2, pi_base_t + imu_lift]) cube([imu_board[0], imu_board[1], 1.6]);
    color("black") translate([lcd_cx, lcd_y, pi_base_t + 2]) rotate([-lcd_tilt, 0, 0])
        translate([-lcd_board[0] / 2, -lcd_board[2] - 0.5, 0]) cube([lcd_board[0], 0.5, lcd_board[1]]);
}

module camera_ghost() {
    color("darkgreen") translate([cam_cx - cam_w / 2, cam_by + cam_boss_l, cam_z0]) cube([cam_w, 1, cam_h]);
    color("black") translate([cam_cx, cam_by + cam_boss_l + 1, cam_z0 + cam_lens_z]) rotate([-90, 0, 0]) cylinder(d = 8, h = 10);
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
            translate([box_x0 + wall + clr, box_y0 + wall + clr, floor_t + i * bat_z])
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

module fpc_path(pts) {
    for (i = [0 : len(pts) - 2]) hull() {
        translate(pts[i]) cube([fpc_w, 0.6, 0.6], center = true);
        translate(pts[i + 1]) cube([fpc_w, 0.6, 0.6], center = true);
    }
}

module cable_ghost() {
    lt = box_h + lid_t;
    cy = cam_by + cam_boss_l - 0.8;
    pz = pi_base_t + pi_boss_h + 1.6;
    // カメラの下端 → 板の下の窓 → ふたの上 → USB / LAN 端子の上 → 45° に折って HAT の下へ → CAM 端子
    color("white") {
        fpc_path([[cam_cx, cy, cam_z0 + 0.5], [cam_cx, cy - 1.5, lt + 1.5], [cam_cx, cam_by - cam_plate_t - 2, lt + 0.8],
                  [cam_cx, box_y0 - 2, lt + 0.8], [cam_cx, pi_cy + pi_board[1] / 2 + 2, pz + 16],
                  [cam_cx, pi_cy - 12, pz + 14.5]]);
        translate([cam_cx - 8, pi_cy - 12 - 8, pz + 14.5]) cube([0.6, 16, 0.6]);
        path([[cam_cx - 8, pi_cy - 13, pz + 14.5], [pi_cx - pi_board[0] / 2 + 52, pi_cy - 13, pz + 2]], 1.5);
    }
    // LiDAR（USB 変換基板）→ Pi の USB
    color("silver") path([[lidar_cx + 10, box_y0 + 20, lt + 3], [lidar_cx, box_y0 + 7, lt + 4], [lidar_cx + 30, box_y0 - 6, lt],
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
    // 液晶 / IMU / 前後の VL53L1X → GPIO ヘッダー（前端）
    hy = pi_cy + pi_board[1] / 2 - 3.5;
    ht = pi_base_t + pi_boss_h + pi_stack_h + 4;
    color("magenta") path([[lcd_cx, lcd_y + 21, pi_base_t + 38], [lcd_cx + 12, hy + 4, ht - 4], [pi_cx - 12, hy, ht]], 3);
    color("purple") path([[imu_c[0], imu_c[1], pi_base_t + imu_lift - 8], [imu_c[0] + imu_board[0] / 2 + 6, imu_c[1], pi_base_t + imu_lift - 8],
                          [30, hy - 2, pi_base_t + 34], [pi_cx - 34, hy + 2, ht]], 2.5);
    color("yellow") {
        path([[plate_w / 2, plate_l + 5, flange_t + 12], [13, plate_l + 4, flange_t + 6], [13, 165, flange_t + 5],
              [13, 135, flange_t + 5], [20, 100, 10], [pi_cx - 25, hy + 2, ht]], 2.5);
        path([[plate_w / 2, -5, flange_t + 12], [plate_w / 2, 10, pi_base_t + pi_boss_h + pi_stack_h + 8],
              [pi_cx - 20, hy, ht + 2]], 2.5);
    }
}


// ---- 上面カバー（天板全体を覆う。左右のフックで天板の端の裏に引っかける。工具不要）----
// LiDAR はカバーの上に出る。LiDAR の外形より大きく抜いてあるので、LiDAR を付けたまま真上へ抜ける
cov_t = 2.4;              // 壁と上面の厚さ
cov_clr = 0.3;            // 天板の端とのすき間
cov_top = 60;             // 上面の裏（Pi 3 段の上端 54 mm の上に 6 mm）
cov_y0 = -1;              // 後ろの壁の内側（後ろのバンパーの上）
cov_y1 = 206.5;           // 前の壁の内側（LiDAR の前端 205 mm の前）
cov_skirt_z = 11;         // 前後の壁の下端。バンパー上面（7 mm）との間に VL53L1X の線を通す
cov_ch = 6;               // 上面の角の面取り
cov_edge = 1;             // 手で触る縁（外側の角、下端、充電口）の面取り
cov_rear = [38.4, 18];    // 後ろの斜面: 後ろの壁の上端の高さ、斜面が上面に届く Y（外側）
cov_hook_y = [45, 193];   // フックの位置（左右とも）
cov_hook_w = 12;
cov_hook_root = 28;       // フックの付け根の高さ（ここから下がたわむ）
cov_hook_lip = 2;         // 天板の裏にかかる深さ
cov_stop_y = [8, 100, 170];  // 天板の上面に載せる受け（左右とも）
cov_rib_x = [[21, 25], [129, 133]];  // ふたの前後の縁をはさんで前後の位置を決めるリブ（右はカメラの板の横）
cov_chg = [113, 185, 5, 38];         // 右側面の充電口（Y0, Y1, Z0, Z1）: バッテリーのポート側の端
cov_cam_w = cam_plate_w + 2;         // カメラの切り欠き（上面から前面まで。カメラは上面より上に出る）
cov_cam_z = cam_z0 + cam_lens_z - 8; // 切り欠きの前面の下端（レンズの下 8 mm）
cov_lcd = [4, 31, 14, 16];           // 液晶の窓（X0, X1, 後ろの壁の下端 Z, 斜面の上端 Y）
cov_vent = [42, 112, 28, 72];        // Pi の上の六角の通気口の範囲

cov_ox0 = -cov_clr - cov_t;
cov_ox1 = plate_w + cov_clr + cov_t;
cov_oy0 = cov_y0 - cov_t;
cov_oy1 = cov_y1 + cov_t;
cov_oz = cov_top + cov_t;

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
            for (x = [0, 1], y = cov_stop_y)
                translate([x ? plate_w : 0, y, 0]) mirror([x, 0, 0])
                    rotate([90, 0, 0]) linear_extrude(8, center = true)
                        polygon([[-cov_clr - 0.01, 0], [2.5, 0], [2.5, 0.8], [-cov_clr - 0.01, 3.6]]);
            // ふたの前後の縁をはさむリブ
            for (r = cov_rib_x, y = [[box_y0 - 5.3, box_y0 - 2.3], [box_y0 + box_y + 2.3, box_y0 + box_y + 5.3]])
                translate([r[0], y[0], box_h + 1]) cube([r[1] - r[0], y[1] - y[0], cov_top - box_h - 0.99]);
        }
        // フックのまわりの切れ目（1 mm）
        for (x = [0, 1], y = cov_hook_y)
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
        // カメラ（上面から前面へ抜いた切り欠き。カメラと板を囲むので、真上へ抜ける）
        translate([0, cov_oy1 + 1, 0]) rotate([90, 0, 0]) linear_extrude(cov_oy1 + 1 - (cam_by - cam_plate_t - 1.5))
            translate([cam_cx - cov_cam_w / 2, cov_cam_z]) offset(r = 3) offset(delta = -3) square([cov_cam_w, cov_oz + 10 - cov_cam_z]);
        // 前面のスリット（飾り）
        for (i = [0 : 2]) translate([plate_w / 2 - 14, cov_y1 - 1, 20 + i * 7]) cube([28, cov_t + 2, 3]);
        // 液晶の窓（後ろ左の角。後ろ向きに倒した画面を斜め後ろ上から見る）
        translate([cov_lcd[0], cov_oy0 - 1, cov_lcd[2]]) cube([cov_lcd[1] - cov_lcd[0], cov_lcd[3] - cov_oy0 + 1, cov_oz]);
        // 六角の通気口
        for (i = [0 : 12], j = [0 : 8]) {
            x = cov_vent[0] + i * 6 + (j % 2) * 3;
            y = cov_vent[2] + j * 5.2;
            if (x <= cov_vent[1] && y <= cov_vent[3]) translate([x, y, cov_top - 1]) cylinder(r = 2.4, h = cov_t + 2, $fn = 6);
        }
        // 文字
        translate([140, 150, cov_oz - 0.8]) rotate([0, 0, 90]) linear_extrude(1)
            text("AI-CAR", size = 9, halign = "center", valign = "center", font = "Liberation Sans:style=Bold");
    }
}

if (part == "box") box();
else if (part == "lid") lid();
else if (part == "pi_base") pi_base();
else if (part == "bumper") translate([0, 0, flange_t + bump_t]) mirror([0, 0, 1]) bumper();
else if (part == "cover") translate([0, 0, cov_oz]) mirror([0, 0, 1]) cover();
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

echo(box_outer = [box_x, box_y, box_h], interior = [in_x, in_y, in_z],
     lidar_holes = [for (p = lidar_holes_rel) lidar_hole(p)],
     lidar_top_z = box_h + lid_t + lidar_tower_h,
     cam_holes_xz = cam_pts(), cam_z = [cam_z0, cam_z0 + cam_h], cam_plate_top_z = cam_top,
     cam_lens_center_z = cam_z0 + cam_lens_z, pi_holes = pi_pts,
     pi_stack_top_z = pi_base_t + pi_boss_h + pi_stack_h,
     pi_stack_front_y = pi_cy + pi_board[1] / 2,
     lidar_head_rear_y = lidar_cy - 35, box_flange_rear_y = flange_y0,
     imu_top_z = pi_base_t + imu_lift + 1.6,
     lcd_top_z = pi_base_t + 2 + (lcd_board[1] + 2) * cos(lcd_tilt),
     tof_front = tof_c(), tof_rear = [plate_w - tof_c()[0], plate_l - tof_c()[1]],
     tof_center_z = flange_t - tof_drop, tof_tilt = tof_tilt,
     screw_mode = screw_mode, tap_d = [tap_d(2), tap_d(2.5), tap_d(3)], insert_d = [insert_d(2), insert_d(2.5), insert_d(3)],
     fpc_x = [cam_cx - fpc_w / 2, cam_cx + fpc_w / 2],
     lidar_tower_x_max = lidar_cx + 28 + lidar_tower_d / 2, lidar_motor_x_max = lidar_cx + lidar_motor_d / 2,
     lidar_motor_bottom_z = box_h + lid_t + lidar_tower_h - lidar_below, lid_top_z = box_h + lid_t,
     lid_screw_head_x_min = boss_pts[1][0] - head_d(3) / 2,
     drok_x = [drok_cx - drok_board[0] / 2, drok_cx + drok_board[0] / 2], drok_front_y = drok_by - drok_h,
     drok_top_z = drok_z0 + drok_board[1],
     bump_out = bump_out, bump_gap = bump_gap, stop_len = bump_gap - bump_travel, spring_strain = spring_strain);
