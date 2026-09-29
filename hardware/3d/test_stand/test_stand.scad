// AI-CAR の試験台: カバーとバンパーを付けたまま、天板の 4 隅の下面を受けて車輪を床から浮かせる
// 1 個の部品: 4 隅の柱と左右の位置決めの壁を、中抜き（斜めの格子）の床板 10 mm でつなぐ
// 座標は天板と同じ（X 左右: 天板の左端 0、Y 前後: 天板の後端 0・+Y が前、Z: 天板の上面 0）
// part = "stand"（印刷用） / "view"（車体と重ねた図）
part = "stand";

use <../battery_lidar_mount/battery_lidar_mount.scad>

// ---- 車体（battery_lidar_mount.scad と同じ値）----
plate_w = 154;
plate_l = 200;
plate_t = 2;
cov_in = 0.3;             // カバーの左右の壁の内側（天板の端から外へ）
cov_side = cov_in + 2.4;  // カバーの左右の壁の外側。壁の下端は天板の下面
wheel_axle = 52;          // 車軸（天板の前後の端から）。OSOYOO のシャーシ 1 段目の図面で、モーターの台（37 mm 用）の穴が Y 37 / 67 mm。台の幅 40 mm の中心に軸
wheel_d = 80.59;          // 車輪の直径（OSOYOO のメカナムホイールの図面）
wheel_w = 39.1;           // 車輪の幅（同じ図面）
deck_floor = 70;          // 床から天板の下面まで（実測。車輪が床に着いた状態。図面からの計算では 40.3 + 27 = 67.3 mm）
wheel_in = 4;             // カバーの左右の壁の外側から車輪の内側の面まで（仮定。実測して合わせる）

// ---- 試験台 ----
st_h = 80;                // 床から天板の下面（受け面）まで
st_clr = 1;               // 車体とのすき間
pad_x = 40;               // 受け面の左右の幅（天板の端から内側へ）。天板の下のモーターの台は Y 32 mm から後ろなので当たらない
pad_x0 = st_clr - cov_in;  // 受け面の外側の端（カバーの壁の内側から 1 mm）
pad_y = wheel_axle - wheel_d / 2 - st_clr;  // 受け面の奥行き（天板の端から車輪の手前まで）
wall_t = 3;               // 位置決めの壁の厚さ
wall_up = 6;              // 壁の上端（天板の上面から）。カバーの左右の壁の外側にかぶせる
base_t = 10;              // 床板の厚さ
base_rim = 8;             // 床板の外周の枠の幅
rib = 5;                  // 床板の斜めの格子のリブの幅
cell = 32;                // 格子のピッチ（ひし形の穴の対角線 + リブ）
gus_l = 22;               // 柱の内側の三角の補強の長さ（Y）と高さ（床板の上から）
gus_t = 6;
edge = 1;
// 天板の 4 隅の穴（Ø4.4。上の部品を留めるネジの先・ナット）を受け面にくぼみでよける
corner_holes = [[11, 11], [plate_w - 11, 11]];
nut_d = 12;               // くぼみの直径（M4 のナットの角 8.1 mm・ワッシャー Ø9 より大きく）
nut_h = 12;               // くぼみの深さ（天板の下面から）

z_floor = -plate_t - st_h;
z_base = z_floor + base_t;
lift = st_h - deck_floor;  // 車輪が浮く量
x_wall0 = -cov_side - st_clr - wall_t;
x_wall1 = plate_w + cov_side + st_clr + wall_t;
y0 = -st_clr;             // 床板の後ろの端（天板の後端より 1 mm 後ろ。後ろのバンパーの下端は床から約 70 mm）
y1 = plate_l + st_clr;
wheel_y = [wheel_axle - wheel_d / 2 - st_clr, wheel_axle + wheel_d / 2 + st_clr];  // 前後の車輪の範囲（天板の端から）
assert(pad_y >= 10, "車輪が大きく、受け面が 10 mm 取れない");
assert(wheel_in >= st_clr, "床板が車輪の下に入る（床板の上面は浮いた車輪の下端とほぼ同じ高さ）");

module rbox(p0, p1, r = edge) {
    translate([p0[0], p0[1], p0[2]]) hull()
        for (x = [r, p1[0] - p0[0] - r], y = [r, p1[1] - p0[1] - r])
            translate([x, y, 0]) cylinder(r = r, h = p1[2] - p0[2], $fn = 16);
}

// 床板の外形（前後の端は左右いっぱい、車輪のある間は天板の幅まで）
module base_outline() {
    offset(r = 3) offset(delta = -3) union() {
        translate([x_wall0, y0]) square([x_wall1 - x_wall0, pad_y - y0]);
        translate([x_wall0, plate_l - pad_y]) square([x_wall1 - x_wall0, y1 - plate_l + pad_y]);
        translate([0, y0]) square([plate_w, y1 - y0]);
    }
}

// 柱の下と補強の下は穴をあけない
module base_solid_zone() {
    for (c = corners()) translate([c[0] - 4, c[1] - 4]) square([c[2] - c[0] + 8, c[3] - c[1] + 8 + gus_l]);
}

// 4 隅の柱の範囲 [x0, y0, x1, y1]（後ろの 2 隅は Y を反転して使う）
function corners() = [[x_wall0, y0, pad_x, pad_y], [plate_w - pad_x, y0, x_wall1, pad_y]];

module base() {
    translate([0, 0, z_floor]) linear_extrude(base_t) difference() {
        base_outline();
        difference() {
            offset(delta = -base_rim) base_outline();
            // 斜めの格子のリブ（ひし形の穴のあいだ）
            for (i = [-8 : 8]) {
                translate([plate_w / 2 + i * cell / sqrt(2), plate_l / 2]) rotate(45) square([rib, 400], center = true);
                translate([plate_w / 2 + i * cell / sqrt(2), plate_l / 2]) rotate(-45) square([rib, 400], center = true);
            }
            // 中心線のリブ（前後）
            translate([plate_w / 2, plate_l / 2]) square([rib, 400], center = true);
            for (m = [0, 1]) translate([0, plate_l / 2]) mirror([0, m]) translate([0, -plate_l / 2]) base_solid_zone();
        }
    }
}

// 後ろの 2 隅（Y が小さい側）
module corner_posts() {
    difference() {
        corner_posts_solid();
        for (h = corner_holes) translate([h[0], h[1], -plate_t - nut_h]) cylinder(d = nut_d, h = nut_h + 0.01, $fn = 48);
    }
}

module corner_posts_solid() {
    for (s = [0, 1]) {
        c = corners()[s];
        // 柱（上面が受け面。天板の外はカバーの壁の下端とバンパーから 1 mm 下げる）
        rbox([s ? plate_w - pad_x : pad_x0, 0, z_floor], [s ? plate_w - pad_x0 : pad_x, pad_y, -plate_t]);
        rbox([c[0], y0, z_floor], [c[2], pad_y, -plate_t - st_clr]);
        // 位置決めの壁（カバーの左右の壁の外側に 1 mm あけてかぶせる）
        wx = s ? plate_w + cov_side + st_clr : x_wall0;
        rbox([wx, y0, z_floor], [wx + wall_t, pad_y, wall_up]);
        // 柱の内側の三角の補強（天板の幅の中、車輪より内側）
        gx = s ? plate_w - pad_x / 2 : pad_x / 2;
        translate([gx - gus_t / 2, pad_y - 0.01, z_base - 0.01]) rotate([90, 0, 90]) linear_extrude(gus_t)
            polygon([[0, 0], [gus_l, 0], [0, gus_l]]);
    }
}

module stand() {
    base();
    corner_posts();
    translate([0, plate_l, 0]) mirror([0, 1, 0]) corner_posts();
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
        color("#222", 0.7) translate([s ? plate_w + cov_side + wheel_in + wheel_w / 2 : -cov_side - wheel_in - wheel_w / 2, y, -plate_t - deck_floor + wheel_d / 2])
            rotate([0, 90, 0]) cylinder(d = wheel_d, h = wheel_w, center = true);
}

if (part == "stand") translate([0, 0, -z_floor]) stand();
if (part == "view") {
    translate([0, 0, lift]) { car(); wheels_ghost(); }
    color("#d98c3a") translate([0, 0, lift]) stand();
}
if (part == "view_raised") { car(); wheels_ghost(); color("#d98c3a") stand(); }

echo(pad = [pad_x, pad_y], lift = lift, stand_size = [x_wall1 - x_wall0, y1 - y0, st_h + plate_t + wall_up]);
