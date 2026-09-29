// AI-CAR の試験台: カバーとバンパーを付けたまま、天板の 4 隅の下面を受けて車輪を床から浮かせる
// 前用と後ろ用の 2 個（同じ形。後ろ用は 180° 回して置く）。1 個に 2 隅の柱と、左右の位置決めの壁
// 座標は天板と同じ（X 左右: 天板の左端 0、Y 前後: 天板の後端 0・+Y が前、Z: 天板の上面 0）
// part = "stand"（印刷用、1 個。2 個印刷する） / "view"（車体と重ねた図）
part = "stand";

use <../battery_lidar_mount/battery_lidar_mount.scad>

// ---- 車体（battery_lidar_mount.scad と同じ値）----
plate_w = 154;
plate_l = 200;
plate_t = 2;
cov_in = 0.3;             // カバーの左右の壁の内側（天板の端から外へ）
cov_side = cov_in + 2.4;  // カバーの左右の壁の外側。壁の下端は天板の下面
cov_hook_y1 = 30 + 6;     // カバーのフック（天板の前後の端から 30 mm、幅 12 mm）の内側の端
bump_face_y = 29.8;       // バンパーの前面の板の裏（天板の端から）。これより手前は天板の下面より下に何もない
wheel_axle = 56;          // 車軸（天板の端から。実測）
wheel_d = 80;             // 車輪の直径（仮定。実測して合わせる）
deck_floor = 70;          // 床から天板の下面まで（実測。車輪が床に着いた状態）

// ---- 試験台 ----
st_h = 80;                // 床から天板の下面（受け面）まで
st_clr = 1;               // 車体とのすき間
pad_x = 25;               // 受け面の左右の幅（天板の端から内側へ）
pad_x0 = st_clr - cov_in;  // 受け面の外側の端（カバーの壁の内側から 1 mm）
pad_y = wheel_axle - wheel_d / 2 - st_clr;  // 受け面の奥行き（天板の端から車輪の手前まで）
wall_t = 3;               // 位置決めの壁の厚さ
wall_up = 6;              // 壁の上端（天板の上面から）。カバーの左右の壁の外側にかぶせる
foot_t = 6;               // 床の板の厚さ
foot_out = 35;            // 床の板を天板の端より外へ出す長さ（バンパーの下。バンパーの下端は床から約 70 mm）
foot_in = 60;             // 床の板を天板の端より内へ入れる長さ（浮いた車輪の下）
edge = 1;

z_floor = -plate_t - st_h;
lift = st_h - deck_floor;  // 車輪が浮く量
x_wall0 = -cov_side - st_clr - wall_t;
x_wall1 = plate_w + cov_side + st_clr + wall_t;
y_in = plate_l - pad_y;   // 受け面の内側の端（前用）
assert(pad_y >= 10, "車輪が大きく、受け面が 10 mm 取れない");
assert(foot_t + st_clr <= lift, "床の板が浮いた車輪に当たる");

module rbox(p0, p1, r = edge) {
    translate([p0[0], p0[1], p0[2]]) hull()
        for (x = [r, p1[0] - p0[0] - r], y = [r, p1[1] - p0[1] - r])
            translate([x, y, 0]) cylinder(r = r, h = p1[2] - p0[2], $fn = 16);
}

// 前用（天板の前端 Y = plate_l 側）。後ろ用は同じものを 180° 回す
module stand() {
    y_out = plate_l + foot_out;
    union() {
        // 床の板
        rbox([x_wall0, plate_l - foot_in, z_floor], [x_wall1, y_out, z_floor + foot_t], 3);
        for (s = [0, 1]) {
            x0 = s ? plate_w - pad_x : x_wall0;
            x1 = s ? x_wall1 : pad_x;
            // 柱（上面が受け面。天板の外はカバーの壁の下端とバンパーから 1 mm 下げる）
            rbox([s ? plate_w - pad_x : pad_x0, y_in, z_floor], [s ? plate_w - pad_x0 : pad_x, plate_l, -plate_t]);
            rbox([x0, y_in, z_floor], [x1, plate_l + 1, -plate_t - st_clr]);
            // 位置決めの壁（カバーの左右の壁の外側に 1 mm あけてかぶせる）
            wx = s ? plate_w + cov_side + st_clr : x_wall0;
            rbox([wx, y_in, z_floor], [wx + wall_t, plate_l, wall_up]);
        }
    }
}

module car() {
    plate();
    color("royalblue") box();
    color("orange") translate([0, 0, 38.4]) lid();
    color("seagreen") pi_base();
    color("dimgray") both_bumpers();
    color("#3c4452", 0.5) cover();
}

module wheels_ghost() {
    for (y = [wheel_axle, plate_l - wheel_axle], s = [0, 1])
        color("#222", 0.7) translate([s ? plate_w + cov_side + 22 : -cov_side - 22, y, -plate_t - deck_floor + wheel_d / 2])
            rotate([0, 90, 0]) cylinder(d = wheel_d, h = 36, center = true);
}

module both_stands() {
    stand();
    translate([plate_w, plate_l, 0]) rotate([0, 0, 180]) stand();
}

if (part == "stand") translate([0, 0, -z_floor]) stand();
if (part == "view") {
    translate([0, 0, lift]) { car(); wheels_ghost(); }
    color("#d98c3a") translate([0, 0, lift]) both_stands();
}
if (part == "view_raised") { car(); wheels_ghost(); color("#d98c3a") both_stands(); }

echo(pad = [pad_x, pad_y], lift = lift, stand_size = [x_wall1 - x_wall0, foot_in + foot_out, st_h + plate_t + wall_up]);
