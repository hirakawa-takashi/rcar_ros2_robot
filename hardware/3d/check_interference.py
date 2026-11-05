#!/usr/bin/env python3
"""3D 部品どうしがぶつからないかを OpenSCAD で調べる。

battery_lidar_mount.scad を include して、組み立てた位置の部品・買う部品の形（ゴースト）・ネジの頭を
2 つずつ intersection() し、重なりの体積が TOL_MM3 より大きい組を「x」にする。
面がふれるだけ（体積 0）は「o」（表示しない）。わざと重ねている組は ALLOWED に理由つきで書く。

使い方:
    python3 hardware/3d/check_interference.py            # CIO とリポの両方
    python3 hardware/3d/check_interference.py --power lipo
ぶつかりが 1 つでもあれば終了コード 1。
"""
import argparse
import itertools
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "battery_lidar_mount", "battery_lidar_mount.scad")
TOL_MM3 = 0.5

# M3 なべ（Ø6.2 × 2.2）、M4 低頭（ボタンキャップ Ø7.6 × 2.2）。頭の下の面は、ネジが押さえる部品の上の面
HEADS = """
module screw_heads() {
    m4 = [7.6, 2.2];
    cc = lipo ? [] : [for (h = sw_holes) h[0]];  // CIO の形: CC スイッチの台の床の座ぐりの中
    function on_cc(p) = len([for (q = cc) if (q == p) 1]) > 0;
    for (p = boss_pts) translate([p[0], p[1], box_h + lid_t]) cylinder(d = head_d(3), h = 2.2);
    for (i = [0 : 3]) if (!on_cc(plate_holes[i])) translate([plate_holes[i][0], plate_holes[i][1], flange_t + bump_t]) cylinder(d = m4[0], h = m4[1]);
    for (p = mount_holes) if (p[1] > 20 && p[1] < 180) translate([p[0], p[1], flange_t]) cylinder(d = m4[0], h = m4[1]);
    for (p = [pi_mount_holes[2], pi_mount_holes[3]]) if (!on_cc(p) && !(lipo && p[0] > plate_w / 2))
        translate([p[0], p[1], pi_base_t]) cylinder(d = m4[0], h = m4[1]);
    for (p = cc) translate([p[0], p[1], sw_floor[1] - sw_cb[1]]) cylinder(d = m4[0], h = m4[1]);
    if (lipo) for (p = lipo_pads) translate([p[0], p[1], lipo_floor[1] - sw_cb[1] - 0.3]) cylinder(d = m4[0], h = m4[1]);
}
"""

# 名前: (OpenSCAD の式, 使う形 None=両方 / "cio" / "lipo")
SOLIDS = {
    "plate": ("plate();", None),
    "box": ("box();", None),
    "lid": ("translate([0, 0, box_h]) lid();", None),
    "pi_base": ("pi_base();", None),
    "bumpers": ("both_bumpers();", None),
    "cover": ("cover();", None),
    "cc_holder": ("cc_holder();", "cio"),
    "cc_spacer": ("for (i = [0, 1]) translate([sw_holes[i][0][0], sw_holes[i][0][1], sw_holes[i][1]]) cc_spacer_at(i);", "cio"),
    "lipo_tray": ("lipo_tray();", "lipo"),
}
GHOSTS = {
    "lidar": ("lidar_ghost();", None),
    "pi": ("pi_ghost();", None),
    "camera": ("camera_ghost();", None),
    "drok": ("drok_ghost();", None),
    "sensor": ("sensor_ghost();", None),
    "tof": ("tof_ghost();", None),
    "cio_bat": ("batteries();", "cio"),
    "cc": ("cc_ghost();", "cio"),
    "lipo": ("lipo_ghost();", "lipo"),
    # リポを前から差し込む通り道（前へ 180 mm 引き出すまで）
    "lipo_path": ("hull() { lipo_ghost(); translate([0, 180, 0]) lipo_ghost(); }", "lipo"),
    "heads": ("screw_heads();", None),
}

# わざと重ねている組（順不同）: (重なりの上限 mm3, 理由)。形が変わって上限をこえたら x にする
ALLOWED = {
    frozenset(("lid", "lidar")): (360, "A1M8（lidar_model=\"a1m8\"）の柱は LiDAR の下の穴に付く（ゴーストの下の 60x60 の四角は大まかな形）。C1 は柱の上に載るだけで 0"),
    frozenset(("lipo", "lipo_path")): (260000, "通り道はリポ自身をふくむ"),
    frozenset(("cover", "lipo_path")): (3600, "リポはカバーを外して差し込む"),
    frozenset(("box", "cio_bat")): (90, "CIO の電池の右の端が、箱の右の口のふちに 0.7 mm かかる（いま使っている箱の形。ゴーストは公称の長さ）"),
}


def run_scad(power, body):
    """body を OpenSCAD で形にして (体積 mm3, エラー) を返す。空の形は体積 0。"""
    src = f'include <{MAIN}>\n{HEADS}\npart = "none";\npower = "{power}";\n{body}\n'
    with tempfile.TemporaryDirectory() as d:
        scad, stl = os.path.join(d, "c.scad"), os.path.join(d, "c.stl")
        with open(scad, "w") as f:
            f.write(src)
        r = subprocess.run(["openscad", "-o", stl, scad], capture_output=True, text=True)
        log = r.stdout + r.stderr
        # 部品の名前や変数を変えて式が空になると、ぶつかりなしに見えてしまうので失敗にする
        unknown = re.findall(r"WARNING: (?:Ignoring unknown \w+|Unknown variable)[^\n]*", log)
        if unknown:
            return None, unknown[:1]
        errors = re.findall(r"ERROR:[^\n]*", log)
        if errors:
            return None, errors[:1]
        # 重ならない組は空の形になり、OpenSCAD は終了コード 1 を返す
        if "top level object is empty" in log:
            return 0.0, None
        if r.returncode != 0:
            return None, log.strip().splitlines()[-1:] or ["openscad failed"]
        if not os.path.exists(stl):
            return None, ["no STL written"]
        return stl_volume(stl), None


def run_pair(power, a, b, ea, eb):
    vol, err = run_scad(power, f"intersection() {{ union() {{ {ea} }} union() {{ {eb} }} }}")
    return power, a, b, vol, err


def run_part(power, a, ea):
    vol, err = run_scad(power, ea)
    if err is None and vol <= TOL_MM3:
        err = ["形が空（この部品をふくむ組は調べられない）"]
    return power, a, None, vol, err


def stl_volume(path):
    v = [tuple(map(float, m.groups())) for m in
         re.finditer(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", open(path).read())]
    vol = 0.0
    for i in range(0, len(v) - 2, 3):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = v[i], v[i + 1], v[i + 2]
        vol += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6.0
    return abs(vol)


def pairs(power):
    def use(m):
        return m is None or m == power
    solids = [(n, e) for n, (e, m) in SOLIDS.items() if use(m)]
    ghosts = [(n, e) for n, (e, m) in GHOSTS.items() if use(m)]
    out = list(itertools.combinations(solids, 2))
    out += [(s, g) for s in solids for g in ghosts]
    out += list(itertools.combinations(ghosts, 2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--power", choices=["cio", "lipo"], action="append")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 2)
    args = ap.parse_args()
    powers = args.power or ["cio", "lipo"]
    parts = [(p, n, e) for p in powers for n, (e, m) in {**SOLIDS, **GHOSTS}.items() if m is None or m == p]
    jobs = [(p, a, b, ea, eb) for p in powers for (a, ea), (b, eb) in pairs(p)]
    bad = 0
    with ThreadPoolExecutor(args.jobs) as ex:
        results = list(ex.map(lambda j: run_part(*j), parts)) + list(ex.map(lambda j: run_pair(*j), jobs))
    for power, a, b, vol, err in results:
        name = a if b is None else f"{a} x {b}"
        lim, why = ALLOWED.get(frozenset((a, b)), (None, ""))
        if err:
            mark, note = "!", " ".join(err)
            bad += 1
        elif b is None or vol <= TOL_MM3:
            continue
        elif lim is not None and vol <= lim:
            mark, note = "-", why
        else:
            mark, note = "x", f"(上限 {lim} mm3) {why}" if lim is not None else ""
            bad += 1
        print(f"{mark} {power:4} {name}: {vol if vol is not None else '?'} mm3 {note}".rstrip())
    print(f"{len(parts)} parts, {len(jobs)} pairs, {bad} problem(s)")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
