#!/usr/bin/env python3
# analyze_ulg.py - 从 PX4 ulog 的 vehicle_local_position CSV 分析正方形航线
# 纯标准库实现 (csv/math)，无第三方依赖
import csv, math, sys, os

DEFAULT = r"D:\AirSim\mission\ulg_csv\07_50_35_vehicle_local_position_0.csv"
OUT_SVG = r"D:\AirSim\mission\trajectory_ulg_20260803.svg"

def run_analysis(path, out_svg=None):
    """分析一条 vehicle_local_position CSV，返回 (score, svg_str)。

    out_svg 为 None 时只返回结果不写文件（便于测试/CI 调用）。
    """
    xs, ys, zs, ts, vxs, vys = [], [], [], [], [], []
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            zs.append(float(row['z']))
            ts.append(int(row['timestamp']))
            vxs.append(float(row['vx']))
            vys.append(float(row['vy']))
    n = len(xs)
    hs = [-z for z in zs]          # 高度 (z 是 Down, 向上为负)
    print(f"[i] 样本数 = {n}, 时间跨度 = {(ts[-1]-ts[0])/1e6:.1f}s")
    print(f"[i] 高度范围: {min(hs):.2f} ~ {max(hs):.2f} m")

    # ---- 空中阶段 (h>1m) ----
    air = [i for i in range(n) if hs[i] > 1.0]
    xmin = min(xs[i] for i in air); xmax = max(xs[i] for i in air)
    ymin = min(ys[i] for i in air); ymax = max(ys[i] for i in air)
    north_span = xmax - xmin
    east_span  = ymax - ymin
    print(f"\n[包围盒] 北向跨度 = {north_span:.2f} m, 东向跨度 = {east_span:.2f} m")

    # ---- 闭合误差 ----
    fa, la = air[0], air[-1]
    closure_origin = math.hypot(xs[la], ys[la])          # 最后空中点回到原点距离
    closure_flight = math.hypot(xs[la]-xs[fa], ys[la]-ys[fa])
    print(f"[闭合] 最后空中点距原点 = {closure_origin:.2f} m, 首尾空中点间距 = {closure_flight:.2f} m")

    # ---- 四腿检测 (按航向量化找腿, 过滤抖动段) ----
    legs = detect_legs(xs, ys, hs, ts)
    print(f"\n[四腿] 检测到 {len(legs)} 段 (已过滤 <2m 抖动):")
    leg_lens = []
    for k,(s,e,head) in enumerate(legs):
        L = math.hypot(xs[e]-xs[s], ys[e]-ys[s])
        leg_lens.append(L)
        print(f"   腿 {k+1}: 起点({xs[s]:.2f},{ys[s]:.2f}) -> 终点({xs[e]:.2f},{ys[e]:.2f}) "
              f"长度 {L:.2f} m, 航向 {math.degrees(head):.0f}°")

    # ---- 巡航高度 (四腿覆盖区间, 排除起飞/降落过渡) ----
    cruise = set()
    for (s,e,_) in legs:
        cruise.update(range(s, e+1))
    cruise = sorted(cruise)
    if not cruise:
        cruise = [i for i in range(n) if 4.0 <= hs[i] <= 6.0]
    am = sum(hs[i] for i in cruise)/len(cruise)
    av = sum((hs[i]-am)**2 for i in cruise)/len(cruise)
    astd = math.sqrt(av)
    print(f"[高度] 四腿巡航段 {len(cruise)} 点: 均值 {am:.2f} m, 标准差 {astd:.3f} m, "
          f"min {min(hs[i] for i in cruise):.2f}, max {max(hs[i] for i in cruise):.2f}")

    # ---- 最大水平速度 ----
    spd = [math.hypot(vxs[i], vys[i]) for i in range(n)]
    print(f"[速度] 最大水平速度 = {max(spd):.2f} m/s")

    # ---- 方形度评分 ----
    if len(leg_lens) >= 4:
        leg_err = sum(abs(L-10.0) for L in leg_lens)/len(leg_lens)
    else:
        leg_err = abs(north_span-10)/2 + abs(east_span-10)/2
    pen_side   = leg_err/10.0
    pen_clos   = closure_origin/10.0
    pen_aspect = abs(north_span-east_span)/10.0
    pen_alt    = astd/1.0
    score = 100*(1 - 0.40*pen_side - 0.30*pen_clos - 0.15*pen_aspect - 0.15*pen_alt)
    score = max(0, min(100, score))
    print(f"\n[方形度] 腿长误差 {leg_err:.2f}m, 闭合 {closure_origin:.2f}m, "
          f"长宽差 {abs(north_span-east_span):.2f}m, 高度std {astd:.3f}m")
    print(f"         综合方形度评分 = {score:.1f} / 100")

    # ---- 生成 SVG ----
    svg = build_svg(xs, ys, hs, ts, air, legs, north_span, east_span,
                    am, astd, max(spd), closure_origin, score)
    if out_svg:
        with open(out_svg, 'w') as f:
            f.write(svg)
        print(f"\n[svg] 已写出: {out_svg}")
    return score, svg

def detect_legs(xs, ys, hs, ts):
    """在巡航高度带内，按航向量化切分直飞腿。"""
    cruise = [i for i in range(len(xs)) if 4.0 <= hs[i] <= 6.0]
    if len(cruise) < 20:
        cruise = [i for i in range(len(xs)) if hs[i] > 1.0]
    # 计算每点航向(中央差分)
    heads = []
    for j,idx in enumerate(cruise):
        a = cruise[max(0,j-3)]; b = cruise[min(len(cruise)-1,j+3)]
        dx = xs[b]-xs[a]; dy = ys[b]-ys[a]
        if abs(dx)+abs(dy) < 1e-4:
            heads.append(heads[-1] if heads else 0.0)
        else:
            heads.append(math.atan2(dy, dx))
    # 量化到 4 个主方向
    def quant(h):
        d = math.degrees(h) % 360
        cand = [0,90,180,270]
        return min(cand, key=lambda c: abs(((d-c+180)%360)-180))
    q = [quant(h) for h in heads]
    # 找连续同方向的段
    legs = []
    start = 0
    for j in range(1, len(q)+1):
        if j == len(q) or q[j] != q[start]:
            seg = cruise[start:j]
            if len(seg) > 5:
                si, ei = seg[0], seg[-1]
                L = math.hypot(xs[ei]-xs[si], ys[ei]-ys[si])
                if L > 2.0:   # 过滤起飞/降落/拐角的抖动小段
                    legs.append((si, ei, math.radians(q[start])))
            start = j
    return legs

def build_svg(xs, ys, hs, ts, air, legs, nspan, espan, am, astd, vmax, clos, score):
    W, H = 680, 820
    # 俯视面板
    pad = 60
    top_h = 420
    # 缩放
    allx = [xs[i] for i in air]; ally = [ys[i] for i in air]
    minx, maxx = min(allx+[0]), max(allx+[0])
    miny, maxy = min(ally+[0]), max(ally+[0])
    sx = (W-2*pad)/(maxx-minx) if maxx>minx else 1
    sy = (top_h-2*pad)/(maxy-miny) if maxy>miny else 1
    s = min(sx, sy)
    def px(x): return pad + (x-minx)*s
    def py(y): return pad + (maxy-y)*s   # y 翻转(北向上)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="monospace">']
    parts.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
    parts.append(f'<text x="{W/2}" y="28" text-anchor="middle" font-size="18" fill="#222">PX4 ulg 真实轨迹分析 (俯视 N-E)</text>')
    parts.append(f'<text x="{W/2}" y="50" text-anchor="middle" font-size="13" fill="#d00">方形度评分 {score:.1f}/100</text>')
    # 轴
    parts.append(f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{top_h-pad}" stroke="#999"/>')
    parts.append(f'<line x1="{pad}" y1="{top_h-pad}" x2="{W-pad}" y2="{top_h-pad}" stroke="#999"/>')
    parts.append(f'<text x="{pad-8}" y="{pad+10}" font-size="11" fill="#555">N</text>')
    parts.append(f'<text x="{W-pad+6}" y="{top_h-pad+4}" font-size="11" fill="#555">E</text>')
    # 轨迹线
    d = "M " + " L ".join(f"{px(xs[i]):.1f},{py(ys[i]):.1f}" for i in air)
    parts.append(f'<path d="{d}" fill="none" stroke="#1f77b4" stroke-width="1.5" opacity="0.85"/>')
    # 起降点
    parts.append(f'<circle cx="{px(xs[air[0]]):.1f}" cy="{py(ys[air[0]]):.1f}" r="5" fill="#2ca02c"/>')
    parts.append(f'<circle cx="{px(xs[air[-1]]):.1f}" cy="{py(ys[air[-1]]):.1f}" r="5" fill="#d62728"/>')
    parts.append(f'<text x="{px(xs[air[0]])+8}" y="{py(ys[air[0]])-6}" font-size="11" fill="#2ca02c">起飞</text>')
    parts.append(f'<text x="{px(xs[air[-1]])+8}" y="{py(ys[air[-1]])-6}" font-size="11" fill="#d62728">降落</text>')
    # 四腿标注
    for k,(si,ei,_) in enumerate(legs):
        mx=(xs[si]+xs[ei])/2; my=(ys[si]+ys[ei])/2
        L=math.hypot(xs[ei]-xs[si],ys[ei]-ys[si])
        parts.append(f'<text x="{px(mx):.1f}" y="{py(my):.1f}" font-size="11" fill="#555">L{k+1}:{L:.1f}m</text>')
    # 高度曲线面板
    ay0 = top_h + 30
    ah = H - ay0 - 30
    hmin, hmax = min(hs), max(hs)
    def hx(i): return pad + (ts[i]-ts[air[0]])/(ts[air[-1]]-ts[air[0]]+1)*(W-2*pad)
    def hy(h): return ay0 + (hmax-h)/(hmax-hmin+1e-6)*ah
    parts.append(f'<text x="{W/2}" y="{ay0-10}" text-anchor="middle" font-size="14" fill="#222">高度曲线 (z取反, 巡航 {am:.2f}±{astd:.2f}m)</text>')
    hd = "M " + " L ".join(f"{hx(i):.1f},{hy(hs[i]):.1f}" for i in air)
    parts.append(f'<path d="{hd}" fill="none" stroke="#9467bd" stroke-width="1.2"/>')
    parts.append(f'<line x1="{pad}" y1="{hy(am):.1f}" x2="{W-pad}" y2="{hy(am):.1f}" stroke="#2ca02c" stroke-dasharray="4 3"/>')
    parts.append(f'<text x="{W-pad+4}" y="{hy(am)+4}" font-size="10" fill="#2ca02c">巡航{am:.1f}m</text>')
    # 指标表
    info = [f"北向跨度 {nspan:.2f}m  东向跨度 {espan:.2f}m",
            f"闭合误差 {clos:.2f}m  最大速度 {vmax:.2f}m/s",
            f"真实腿数 {len(legs)}  高度std {astd:.3f}m"]
    for k,t in enumerate(info):
        parts.append(f'<text x="{pad}" y="{H-40+k*16}" font-size="11" fill="#444">{t}</text>')
    parts.append('</svg>')
    return "\n".join(parts)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    run_analysis(path, OUT_SVG)

if __name__ == "__main__":
    main()
