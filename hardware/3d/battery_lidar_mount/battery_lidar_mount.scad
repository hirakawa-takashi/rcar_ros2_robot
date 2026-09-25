// AI-CAR バッテリー 2 個 + RPLIDAR A1M8 マウント
// 座標: 天板 154 x 200 mm の左下角が原点。X = 横幅方向、Y = 前後方向（Y=200 側が前 / LiDAR 側）
// 天板の穴位置は Y 方向に対称なので、前後どちら向きでも同じ穴に合う。
//
// 出力: openscad -D 'part="box"' -o box.stl battery_lidar_mount.scad
//       openscad -D 'part="lid"' -o lid.stl battery_lidar_mount.scad
//       openscad -D 'part="pi_base"' -o pi_base.stl battery_lidar_mount.scad

part = "assembly"; // "box" | "lid" | "pi_base" | "assembly"

$fn = 48;

// ---- 天板（ユーザー提供の CAD 図面）----
plate_w = 154;
plate_l = 200;
plate_t = 3;
plate_holes = [[11, 11], [143, 11], [11, 189], [143, 189],
               [9, 90], [145, 90], [9, 110], [145, 110]];
mount_holes = [[11, 189], [143, 189], [9, 110], [145, 110]];
mount_hole_d = 4.4;

// ---- バッテリー（CIO SMARTCOBY Pro SLIM 35W）x 2 段重ね ----
bat_x = 97.6;   // 長辺（X 方向に置く。ポートのある短辺が開口側を向く）
bat_y = 69;
bat_z = 16.2;
bat_n = 2;
clr = 0.6;      // 片側のすき間

// ---- 箱 ----
wall = 2.5;
floor_t = 2;
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
boss_pilot_d = 2.5;
boss_pts = [[box_x0 - boss_d / 2 + 1.5, box_y0 + 6],
            [box_x0 + box_x + boss_d / 2 - 1.5, box_y0 + 6],
            [box_x0 - boss_d / 2 + 1.5, box_y0 + box_y - 6],
            [box_x0 + box_x + boss_d / 2 - 1.5, box_y0 + box_y - 6]];

// ---- カメラ（Raspberry Pi Camera Module 3、箱の前面に上下逆向きで取付）----
// 基板 25 x 23.86 mm、M2 穴 21 x 12.5 mm（ケーブル側と反対の端から 2 / 14.5 mm）
// ケーブルを上に出すため上下逆に付ける → ソフト側で 180° 回転が必要
cam_w = 25;
cam_h = 23.86;
cam_hole_dx = 21;
cam_hole_z = [2, 14.5];   // 基板下端（上下逆なのでケーブルと反対側）からの距離
cam_boss_d = 4.5;
cam_boss_l = 4;
cam_pilot_d = 1.8;        // M2 タッピング
cam_top_gap = 1;          // 箱の上端から基板上端まで
cable_notch_w = 20;       // ふた前端のケーブル逃げ

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
pi_pilot_d = 2.2;         // M2.5 タッピング
pi_base_t = 3;
pi_stack_h = 45;          // Pi 5 + AI HAT+ + Motor HAT の 3 段（実測。台のぶん高くなる側で見積もる）
pi_pts = [for (dx = [0, pi_hole_dx], dy = [0, pi_hole_dy])
          [pi_cx - pi_board[0] / 2 + pi_hole_off + dx, pi_cy - pi_hole_dy / 2 + dy]];

// ---- ラズパイ台の左側: IMU（GY-BNO055）と液晶（ZJY-IPS130）----
wing = [4, 18, 34, 80];   // x0, y0, x1, y1
imu_board = [20, 27];     // 実物に合わせて変更（長辺を前後方向に置く）
imu_c = [18, 35];
imu_lift = 5;             // 下向きのピンヘッダーを逃がす台の高さ
imu_wall = 1.6;
lcd_board = [27.5, 39, 1.6];  // 幅 x 高さ x 基板厚（実物に合わせて変更）
lcd_cx = 17.5;
lcd_y = 56;               // 画面の下端（後ろ向き）
lcd_tilt = 30;            // 垂直から後ろへ倒す角度
lcd_lip = 1;              // 基板の左右の縁を押さえる幅

// ---- ふた + LiDAR 台 ----
lid_t = 4;
lid_screw_d = 3.4;
lidar_tower_h = 28;     // LiDAR 取付面より下に出ている部分（約 26.5 mm）をかわす高さ
lidar_tower_d = 7;
lidar_pilot_d = 2.2;    // M2.5 タッピング（インサートなら 3.5 前後に変更）
lidar_motor_front = true; // LiDAR のモーター側（細い側）を前（+Y）に向ける
// RPLIDAR A1M8 取付穴（回転中心基準、データシート Figure 5-2）
//   ヘッド側 2 穴: 中心から 28 mm、間隔 56 mm / モーター側 2 穴: 中心から 42 mm、間隔 40 mm
lidar_holes_rel = [[-28, 28], [28, 28], [-20, -42], [20, -42]];
lidar_cx = box_cx;
lidar_cy = box_cy + (lidar_motor_front ? -7 : 7);

function lidar_hole(p) = lidar_motor_front
    ? [lidar_cx + p[0], lidar_cy - p[1]]
    : [lidar_cx + p[0], lidar_cy + p[1]];

cam_z0 = box_h - cam_top_gap - cam_h;
function cam_pts() = [for (dx = [-cam_hole_dx / 2, cam_hole_dx / 2], z = cam_hole_z) [box_cx + dx, cam_z0 + z]];

module rrect(x0, y0, x1, y1, r, h) {
    hull() for (x = [x0 + r, x1 - r], y = [y0 + r, y1 - r])
        translate([x, y, 0]) cylinder(r = r, h = h);
}

module box() {
    difference() {
        union() {
            rrect(flange_x0, flange_y0, flange_x1, flange_y1, flange_r, flange_t);
            translate([box_x0, box_y0, 0]) cube([box_x, box_y, box_h]);
            for (p = boss_pts) translate([p[0], p[1], 0]) cylinder(d = boss_d, h = box_h);
            for (p = cam_pts()) translate([p[0], box_y0 + box_y - 0.01, p[1]])
                rotate([-90, 0, 0]) cylinder(d = cam_boss_d, h = cam_boss_l);
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
        // カメラ取付の下穴
        for (p = cam_pts()) translate([p[0], box_y0 + box_y + cam_boss_l + 0.01, p[1]])
            rotate([90, 0, 0]) cylinder(d = cam_pilot_d, h = 6);
        // 床の肉抜き
        translate([box_x0 + wall + 12, box_y0 + wall + 10, -1]) cube([in_x - 24, in_y - 20, floor_t + 2]);
        // 天板への取付穴
        for (p = mount_holes) translate([p[0], p[1], -1]) cylinder(d = mount_hole_d, h = flange_t + 2);
        // ふた固定用の下穴
        for (p = boss_pts) translate([p[0], p[1], box_h - 12]) cylinder(d = boss_pilot_d, h = 13);
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
        }
        for (p = boss_pts) translate([p[0], p[1], -1]) cylinder(d = lid_screw_d, h = lid_t + 2);
        for (p = lidar_holes_rel) {
            q = lidar_hole(p);
            translate([q[0], q[1], lid_t + lidar_tower_h - 10]) cylinder(d = lidar_pilot_d, h = 11);
        }
        // カメラケーブルの逃げ（前端中央）
        translate([box_cx - cable_notch_w / 2, box_y0 + box_y - 3, -1]) cube([cable_notch_w, 10, lid_t + 2]);
        // 肉抜き + 配線通し
        translate([lidar_cx - 15, lidar_cy - 15 + (lidar_motor_front ? 7 : -7), -1]) cube([30, 30, lid_t + 2]);
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
            imu_holder();
            lcd_holder();
        }
        for (p = pi_mount_holes) translate([p[0], p[1], -1]) cylinder(d = mount_hole_d, h = pi_base_t + 2);
        for (p = pi_pts) translate([p[0], p[1], pi_base_t + pi_boss_h - 8]) cylinder(d = pi_pilot_d, h = 9);
        // 配線通し・通気
        translate([pad[0] + 10, pad[1] + 10, -1]) cube([pad[2] - pad[0] - 20, pad[3] - pad[1] - 20, pi_base_t + 2]);
    }
}

module imu_holder() {
    ix = imu_board[0] + 0.6;
    iy = imu_board[1] + 0.6;
    h = imu_lift + 1.6 + 1;
    translate([imu_c[0], imu_c[1], pi_base_t - 0.01]) difference() {
        translate([-ix / 2 - imu_wall, -iy / 2 - imu_wall, 0]) cube([ix + 2 * imu_wall, iy + 2 * imu_wall, h]);
        // 基板の収まる部分
        translate([-ix / 2, -iy / 2, imu_lift]) cube([ix, iy, h]);
        // ピンヘッダーの逃げ（四隅の台だけ残す）
        translate([-ix / 2 + 3, -iy / 2 - imu_wall - 1, -1]) cube([ix - 6, iy + 2 * imu_wall + 2, imu_lift + 2]);
        translate([-ix / 2 - imu_wall - 1, -iy / 2 + 3, -1]) cube([ix + 2 * imu_wall + 2, iy - 6, imu_lift + 2]);
        // 配線側（前）の壁を開ける
        translate([-ix / 2 + 2, iy / 2 - 1, imu_lift]) cube([ix - 4, imu_wall + 2, h]);
    }
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
        // 左右の支え
        for (sx = [-1, 1]) translate([sx * (lcd_board[0] / 2 + 1.15) - 1.15, 0, 0])
            rotate([90, 0, 90]) linear_extrude(2.3)
                polygon([[0, 0], [(H + 2) * sin(lcd_tilt) + 2, 0], [(H + 2) * sin(lcd_tilt), (H + 2) * cos(lcd_tilt)]]);
    }
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
    color("darkgreen") translate([box_cx - cam_w / 2, box_y0 + box_y + cam_boss_l, cam_z0]) cube([cam_w, 1, cam_h]);
    color("black") translate([box_cx, box_y0 + box_y + cam_boss_l + 1, cam_z0 + cam_h - 14.4]) rotate([-90, 0, 0]) cylinder(d = 8, h = 10);
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
    }
}

if (part == "box") box();
else if (part == "lid") lid();
else if (part == "pi_base") pi_base();
else {
    plate();
    color("royalblue") box();
    batteries();
    color("orange") translate([0, 0, box_h]) lid();
    lidar_ghost();
    color("seagreen") pi_base();
    pi_ghost();
    camera_ghost();
    sensor_ghost();
}

echo(box_outer = [box_x, box_y, box_h], interior = [in_x, in_y, in_z],
     lidar_holes = [for (p = lidar_holes_rel) lidar_hole(p)],
     lidar_top_z = box_h + lid_t + lidar_tower_h,
     cam_holes_xz = cam_pts(), pi_holes = pi_pts,
     pi_stack_top_z = pi_base_t + pi_boss_h + pi_stack_h,
     pi_stack_front_y = pi_cy + pi_board[1] / 2,
     lidar_head_rear_y = lidar_cy - 35, box_flange_rear_y = flange_y0,
     lcd_top_z = pi_base_t + 2 + (lcd_board[1] + 2) * cos(lcd_tilt));
