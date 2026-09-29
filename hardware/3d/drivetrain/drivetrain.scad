// AI-CAR の駆動系（購入品）の参考モデル: メカナムホイール・520 モーター・37 mm モーターの台 ×4
// 印刷しない。組み立て図に重ねて表示するためだけの形（寸法は OSOYOO の図面）
// 座標は天板と同じ（X 左右: 天板の左端 0、Y 前後: 天板の後端 0・+Y が前、Z: 天板の上面 0）
// part = "all" / "wheels" / "motors"（モーター・軸・カップリング） / "holders"（モーターの台）
part = "all";

plate_w = 154;
plate_l = 200;
plate_t = 2;
cov_side = 2.7;           // カバーの左右の壁の外側（天板の端から外へ）。test_stand.scad と同じ

// メカナムホイール（OSOYOO 2019018100.jpg）
wheel_d = 80.59;
wheel_w = 39.1;
wheel_hex = 6.5;          // 中心の六角穴
wheel_in = 4;             // カバーの壁の外側から車輪の内側の面まで（仮定。test_stand.scad と同じ）
rollers = 8;
roller_d = 17;            // ローラーの太さ（図面にないので見た目の値）

// 37 mm モーターの台（2018003600.jpg）: L 字、板厚 2、底 42 × 40、立ち上がり 46、軸は底から 27
hold_t = 2;
hold_l = 42;
hold_w = 40;
hold_h = 46;
hold_axis = 27;
hold_hole0 = 9.5;         // 立ち上がりの外側の面から近い方の穴まで（穴の間隔 24 × 30）
hole_x = 12;              // 天板の穴（1st_layer.dwg: X 12 / 36、Y 37 / 67）
wheel_axle = 52;          // 車軸（天板の前後の端から）。穴 Y 37 / 67 の中心

// 520 モーター（2024005900.pdf）: 全長 58（うちギア箱 Ø37 × 24、モーター Ø32.8）、ボス Ø12 × 6、軸 Ø4 × 16
gear_d = 37;
gear_l = 24;
mot_d = 32.8;
mot_l = 58;
boss_d = 12;
boss_l = 6;
shaft_d = 4;
shaft_l = 16;
coup_d = 12;              // カップリング（図面にないので見た目の値）。車輪の中は六角 6.5 mm

face_x = hole_x - hold_hole0;        // 台の立ち上がりの外側の面（左）
motor_x = face_x + hold_t;           // ギア箱の前の面
axle_z = -plate_t - hold_axis;       // 軸の高さ
wheel_x0 = -cov_side - wheel_in;     // 車輪の内側の面（左）

$fn = 48;

// 左前の 1 組を作り、左右・前後に鏡像で並べる
module corners() {
    for (mx = [0, 1], my = [0, 1])
        translate([mx * plate_w, my * plate_l, 0]) mirror([mx, 0, 0]) mirror([0, my, 0])
            translate([0, wheel_axle, 0]) children();
}

module wheel(hand = 1) {
    translate([wheel_x0 - wheel_w / 2, 0, axle_z]) rotate([0, 90, 0]) {
        difference() {
            union() {
                for (s = [-1, 1]) translate([0, 0, s * (wheel_w / 2 - 1.5)]) cylinder(d = wheel_d - 16, h = 3, center = true);
                cylinder(d = 22, h = wheel_w - 2, center = true);
            }
            cylinder(d = wheel_hex / cos(30), h = wheel_w + 2, center = true, $fn = 6);
        }
        intersection() {
            cylinder(d = wheel_d, h = wheel_w, center = true, $fn = 96);
            for (i = [0 : rollers - 1]) rotate([0, 0, i * 360 / rollers])
                translate([wheel_d / 2 - roller_d / 2, 0, 0]) rotate([hand * 45, 0, 0])
                    scale([1, 1, 2.6]) sphere(d = roller_d, $fn = 18);
        }
    }
}

module motor() {
    translate([0, 0, axle_z]) rotate([0, 90, 0]) {
        translate([0, 0, motor_x]) cylinder(d = gear_d, h = gear_l);
        translate([0, 0, motor_x + gear_l]) cylinder(d = mot_d, h = mot_l - gear_l);
        translate([0, 0, motor_x - boss_l]) cylinder(d = boss_d, h = boss_l);
        translate([0, 0, motor_x - shaft_l]) cylinder(d = shaft_d, h = shaft_l);
        translate([0, 0, wheel_x0]) cylinder(d = coup_d, h = motor_x - boss_l - wheel_x0);
        translate([0, 0, wheel_x0 - wheel_w / 2]) cylinder(d = wheel_hex / cos(30) - 0.2, h = wheel_w / 2, $fn = 6);
    }
}

module holder() {
    difference() {
        union() {
            translate([face_x, -hold_w / 2, -plate_t - hold_t]) cube([hold_l, hold_w, hold_t]);
            translate([face_x, 0, 0]) rotate([0, 90, 0]) linear_extrude(hold_t) hull() {
                translate([plate_t, -hold_w / 2]) square([1, hold_w]);
                translate([-axle_z, 0]) circle(r = hold_h - hold_axis);
            }
        }
        translate([face_x - 1, 0, axle_z]) rotate([0, 90, 0]) cylinder(d = boss_d + 1, h = hold_t + 2);
    }
}

if (part == "all" || part == "wheels") color("#474d57") corners() wheel();
if (part == "all" || part == "motors") color("#b9c2cc") corners() motor();
if (part == "all" || part == "holders") color("#2f3440") corners() holder();

echo(face_x = face_x, motor_end = motor_x + mot_l, axle_z = axle_z, wheel_bottom = axle_z - wheel_d / 2);
