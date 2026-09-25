// AI-CAR バッテリー 2 個 + RPLIDAR A1M8 マウント
// 座標: 天板 154 x 200 mm の左下角が原点。X = 横幅方向、Y = 前後方向（Y=200 側が前 / LiDAR 側）
// 天板の穴位置は Y 方向に対称なので、前後どちら向きでも同じ穴に合う。
//
// 出力: openscad -D 'part="box"' -o box.stl battery_lidar_mount.scad
//       openscad -D 'part="lid"' -o lid.stl battery_lidar_mount.scad

part = "assembly"; // "box" | "lid" | "assembly"

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
        for (i = [-2 : 2], y = [box_y0 - 1, box_y0 + box_y - wall - 1])
            translate([box_cx + i * 12 - 2.5, y, floor_t + 6]) cube([5, wall + 2, in_z - 12]);
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
        // 肉抜き + 配線通し
        translate([lidar_cx - 15, lidar_cy - 15 + (lidar_motor_front ? 7 : -7), -1]) cube([30, 30, lid_t + 2]);
    }
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
else {
    plate();
    color("royalblue") box();
    batteries();
    color("orange") translate([0, 0, box_h]) lid();
    lidar_ghost();
}

echo(box_outer = [box_x, box_y, box_h], interior = [in_x, in_y, in_z],
     lidar_holes = [for (p = lidar_holes_rel) lidar_hole(p)],
     lidar_top_z = box_h + lid_t + lidar_tower_h);
