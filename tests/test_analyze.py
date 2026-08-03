"""analyze_ulg 轨迹分析 / 方形度评分 的回归测试。

覆盖三类轨迹：
  - 真实飞行样本（fixture）：应落在 80~100 高分区间
  - 合成完美方形：应 >= 95（接近满分）
  - 合成随机游走：应明显偏低（< 50），证明评分能区分优劣
"""
import csv
import os
import random
import tempfile

import analyze_ulg as au

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE = os.path.join(ROOT, "tests", "fixtures", "sample_local_position.csv")


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "x", "y", "z", "vx", "vy"])
        for r in rows:
            w.writerow(r)


def make_square(side=10.0, per_leg=60, alt=-5.0, speed=1.0):
    corners = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    rows = []
    t0 = 1_785_743_432_974_390
    ti = 0
    for k in range(4):
        x0, y0 = corners[k]
        x1, y1 = corners[k + 1]
        dx = (x1 - x0) / per_leg
        dy = (y1 - y0) / per_leg
        for s in range(per_leg):
            rows.append((t0 + ti * 10000, x0 + dx * s, y0 + dy * s, alt,
                         dx * speed, dy * speed))
            ti += 1
    return rows


def make_random(n=800, seed=1):
    random.seed(seed)
    x = y = z = 0.0
    vx = vy = 0.0
    rows = []
    t0 = 1_785_743_432_974_390
    for i in range(n):
        vx += random.uniform(-0.25, 0.25)
        vy += random.uniform(-0.25, 0.25)
        x += vx * 0.1
        y += vy * 0.1
        z = -5.0 + random.uniform(-0.4, 0.4)
        rows.append((t0 + i * 10000, x, y, z, vx, vy))
    return rows


def _tmp_csv(rows):
    fd, p = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    _write_csv(p, rows)
    return p


def test_real_fixture_scores_in_band():
    score, svg = au.run_analysis(FIXTURE, None)
    assert 80.0 <= score <= 100.0, f"真实样本评分 {score:.1f} 超出 80~100"
    assert "方形度评分" in svg


def test_real_fixture_svg_well_formed():
    _, svg = au.run_analysis(FIXTURE, None)
    assert svg.lstrip().startswith("<svg")
    assert svg.rstrip().endswith("</svg>")


def test_perfect_square_scores_high():
    p = _tmp_csv(make_square())
    try:
        score, _ = au.run_analysis(p, None)
    finally:
        os.remove(p)
    assert score >= 95.0, f"完美方形评分 {score:.1f}，期望 >=95"


def test_degenerate_scores_low():
    p = _tmp_csv(make_random())
    try:
        score, _ = au.run_analysis(p, None)
    finally:
        os.remove(p)
    assert score < 50.0, f"随机游走评分 {score:.1f}，期望 <50"


def test_run_analysis_writes_svg_when_path_given():
    p = _tmp_csv(make_square())
    fd, out = tempfile.mkstemp(suffix=".svg")
    os.close(fd)
    try:
        au.run_analysis(p, out)
        assert os.path.getsize(out) > 0
    finally:
        os.remove(p)
        os.remove(out)
