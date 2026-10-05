#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拍屏实验 · 分析工具

对拍到的照片做四项体检，判定 OCR 可行性，并输出标注图。

用法:
    python3 analyze.py shots/                 # 分析目录下所有图
    python3 analyze.py shots/x.png            # 分析单张
    python3 analyze.py shots/ --roi 400,300,900,400   # 只看屏幕区域

四项指标:
    ① 清晰度     拉普拉斯方差（越大越清晰）
    ② 摩尔纹     色度通道频谱峰值比（越小越好，大了 OCR 必废）
    ③ 曝光       过曝像素占比
    ④ 字高       自动检测文字行，报告每行汉字像素高度  ★最关键
"""
import cv2, os, sys, glob, argparse
import numpy as np

# ── 判定阈值（可调）
TH = dict(
    sharp_ok=150, sharp_min=80,
    moire_ok=8,   moire_max=20,   # 峰值突出度：实测无摩尔纹 3.8 / 有摩尔纹 50+
    exp_ok=3.0,   exp_max=10.0,
    char_ok=25,   char_min=20,
)

def load(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        # 支持中文路径
        img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    return img

def save(path, img):
    ext = os.path.splitext(path)[1] or '.png'
    ok, buf = cv2.imencode(ext, img)
    if ok: buf.tofile(path)

# ── ① 清晰度
def sharpness(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

# ── ② 摩尔纹：色度通道频谱里的【孤立窄峰】
#    判据 = 峰值 / 局部背景（峰突出度）
#    ⚠️ 不要用 percentile/median！实测那个指标是【反的】：
#       无摩尔纹图 45.1，加了摩尔纹反而 28~30 —— 完全失效（已踩坑）
#    校准数据（2026-10-05 用合成条纹）：
#       无摩尔纹 3.78 | 弱 57.9 | 中 49.7 | 强 50.2 | 极强 69.9
def moire_score(bgr):
    from scipy import ndimage as ndi
    ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    best = 0.0
    for idx in (1, 2):                       # Cr, Cb
        ch = ycrcb[:, :, idx].astype(np.float64)
        ch -= ch.mean()
        h, w = ch.shape
        if h < 32 or w < 32: continue
        ch = ch * np.hanning(h)[:, None] * np.hanning(w)[None, :]
        F = np.fft.fftshift(np.abs(np.fft.fft2(ch)))
        bg = ndi.median_filter(F, size=15)   # 局部背景
        prom = F / (bg + 1e-9)               # 突出度
        cy, cx = h//2, w//2
        Yg, Xg = np.ogrid[:h, :w]
        r = np.sqrt((Yg-cy)**2 + (Xg-cx)**2)
        rmax = min(cy, cx)
        band = (r > 0.05*rmax) & (r < 0.95*rmax)
        v = prom[band]
        if v.size: best = max(best, float(np.percentile(v, 99.99)))
    return best

# ── ③ 曝光
def exposure_stats(gray):
    """★ 不要用"整幅 >250 占比"当做过曝 —— 屏幕是自发光的，
    白底本来就接近 255，那是正常的，不是过曝。
    真正该问的是：屏幕内【黑字还在不在】。
    返回 (过曝/对比丢失指标, 暗笔画占比, 均值)
      - 暗笔画占比 <0.5%  => 文字被冲掉（真过曝 / 或没字）
      - 暗笔画占比 >60%   => 屏幕太暗/欠曝
    """
    mu = float(gray.mean())
    dark = float((gray < mu * 0.55).mean()) * 100   # 文字笔画占比
    lost = max(0.0, 0.5 - dark) if dark < 0.5 else 0.0   # 只有"丢字"才算过曝
    return lost, dark, mu

# ── ③.5 自动找屏幕区域（最大亮连通域）
def find_screen(bgr, min_area_frac=0.004, debug=False):
    """找屏幕区域。返回 (x,y,w,h) 或 None

    ★ 不能用"最大亮连通域" —— 画面里可能有更亮更大的东西（白衣服/白墙）。
      屏幕的判据是三合一：
        ① 亮    均值 >= 90
        ② 有对比 标准差 >= 30   （白衣服/墙是均匀的，标准差小）
        ③ 有笔画 内部"暗像素"占比 1%~50%  （文字的特征）
      评分 = 均值 x 标准差（又亮又有内容）
    实测（2026-10-05）：
        手机屏幕 227x129 均值226.8 标准差59.5  暗笔画8.0%
        手机屏幕 373x242 均值170.7 标准差110.6 暗笔画31.5%
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape
    if gray.mean() > 200:
        return (0, 0, W, H)
    _, ot = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, lab, st, _ = cv2.connectedComponentsWithStats(ot, 8)
    if n <= 1:
        return None
    best, best_score = None, -1.0
    for i in range(1, n):
        x, y, w, h, area = st[i]
        if area < W*H*min_area_frac: continue
        if w < 50 or h < 40: continue
        if not (0.35 <= w/h <= 6): continue
        roi = gray[y:y+h, x:x+w]
        mu, sd = float(roi.mean()), float(roi.std())
        if mu < 90: continue                       # 不够亮
        if sd < 30: continue                       # 太均匀 => 衣服/墙
        dark = float((roi < mu*0.55).mean()) * 100
        if not (1.0 <= dark <= 50.0): continue     # 没有文字笔画
        score = mu * sd
        if debug:
            print(f"    候选 ({x},{y}) {w}x{h} 均值{mu:.1f} 标准差{sd:.1f} 暗笔画{dark:.1f}% 分{score:.0f}")
        if score > best_score:
            best_score, best = score, (x, y, w, h)
    return best

def clean_screen_bg(gray):
    """★ 关键预处理：屏幕通常是斜的，外接矩形会包进四角的黑背景。
    黑背景在"反相二值化"后会全变成前景 -> 文字行连成一整块。
    做法：取最大亮连通域的【外轮廓并填充】得到实心屏幕 mask，
          只把 mask 外的像素填成屏幕底色。
    ⚠️ 必须用外轮廓填充 —— 直接用连通域会把屏幕内的【文字】当成洞填掉！
    """
    _, lit = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, lab, st, _ = cv2.connectedComponentsWithStats(lit, 8)
    if n <= 1:
        return gray
    big = 1 + int(np.argmax(st[1:, 4]))
    comp = np.where(lab == big, 255, 0).astype(np.uint8)

    cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return gray
    c = max(cnts, key=cv2.contourArea)
    solid = np.zeros_like(comp)
    cv2.drawContours(solid, [c], -1, 255, thickness=cv2.FILLED)   # ★ 填充成实心
    # 稍微内缩，避开屏幕边缘的高光/黑边
    k = max(3, int(min(gray.shape) * 0.01))
    solid = cv2.erode(solid, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))

    on = solid > 0
    if on.sum() < gray.size * 0.02:
        return gray
    fill = int(np.clip(np.median(gray[on]), 0, 255))    # 中位数比均值更抗文字干扰
    return np.where(on, gray, fill).astype(np.uint8)

def _segments(mask, min_len):
    """把布尔数组里的连续 True 区间提取出来"""
    segs, s0 = [], None
    for i, v in enumerate(mask):
        if v and s0 is None: s0 = i
        elif not v and s0 is not None:
            if i - s0 >= min_len: segs.append((s0, i))
            s0 = None
    if s0 is not None and len(mask) - s0 >= min_len: segs.append((s0, len(mask)))
    return segs

def detect_lines(gray):
    """在屏幕区内检测文字行。

    ★ 用【水平膨胀 + 连通域】而不是水平投影：
      投影法在【斜拍/透视】下会失效 —— 文字 y 坐标在屏幕两端不同，
      投影被抹平，行间谷值消失（7 张实拍图全部行数数错的根因）。
      水平膨胀把同一行的字连成扁长条，再按连通域取行，对倾斜鲁棒。
    返回 (lines, bw, char_h)
      lines = [(y0, y1, n_chars, char_w_median), ...]
    """
    gray = clean_screen_bg(gray)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    if bw.mean()/255.0 < 0.003 or bw.mean()/255.0 > 0.6:
        bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 35, 12)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN,
                          cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))
    H, W = bw.shape
    if W < 20 or H < 20:
        return [], bw, 0.0

    # ── 1) 估计字宽：原始连通域宽度的中位数
    n, lab, st, cent = cv2.connectedComponentsWithStats(bw, 8)
    ws = []
    for i in range(1, n):
        x, y, w, h, ar = st[i]
        if h < 4 or w < 3 or ar < 5: continue
        if h > H * 0.5: continue
        if not (0.1 <= w / h <= 5): continue
        ws.append(w)
    cw_est = float(np.median(ws)) if ws else max(8.0, W * 0.03)

    # ── 2) 水平膨胀：把同一行的字连成扁长条
    k = max(3, int(round(cw_est * 1.1)))
    ker = cv2.getStructuringElement(cv2.MORPH_RECT, (k, 1))
    merged = cv2.dilate(bw, ker, iterations=1)

    # ── 3) 连通域 = 行；行必须"扁长"
    n2, lab2, st2, cent2 = cv2.connectedComponentsWithStats(merged, 8)
    lines, hs = [], []
    for i in range(1, n2):
        x, y, w, h, ar = st2[i]
        if w < max(20, cw_est * 1.5) or h < 5: continue
        if w / h < 2.2: continue                    # 行是扁长的
        if h > H * 0.6: continue
        # ── 4) 在原始 bw 上量这一行的字高（笔画的垂直跨度）
        sub = bw[y:y+h, x:x+w]
        rows_any = np.where(sub.sum(axis=1) > 0)[0]
        if rows_any.size == 0: continue
        y0 = y + int(rows_any.min()); y1 = y + int(rows_any.max()) + 1
        ch = y1 - y0
        if ch < 5: continue
        # ── 5) 数字数：行内"够大"的连通域
        n3, _, st3, _ = cv2.connectedComponentsWithStats(sub, 8)
        cnt = 0
        for j in range(1, n3):
            _, _, w3, h3, a3 = st3[j]
            if h3 >= ch * 0.25 and w3 >= 3 and a3 >= 6: cnt += 1
        # 若连通域太少（笔画粘连），用宽度估算
        n_est = max(cnt, int(round((x + w - x) / max(ch, 1))))
        lines.append((y0, y1, max(cnt, 2), cw_est))
        hs.append(ch)
    lines.sort(key=lambda t: t[0])
    char_h = float(np.median(hs)) if hs else 0.0
    return lines, bw, char_h

def verdict(sharp, moire, over, char_h, dark_frac=None):
    # 关键项（任一硬失败 => 直接不能 OCR）
    #   字高：低于 20px OCR 必废
    #   摩尔纹：高于 30 => 周期性干涉，OCR 也必废
    items = [
        ("字高",   char_h, "px", char_h>=TH['char_ok'], char_h>=TH['char_min'], True),
        ("摩尔纹", moire,  "",   moire <=TH['moire_ok'], moire <=TH['moire_max'], True),
        ("清晰度", sharp,  "",   sharp>=TH['sharp_ok'],  sharp>=TH['sharp_min'],  False),
        ("过曝",   over,   "%",  over <=TH['exp_ok'],    over <=TH['exp_max'],    False),
    ]
    crit_bad = sum(1 for *_,ok,mid,crit in items if crit and not ok and not mid)
    crit_mid = sum(1 for *_,ok,mid,crit in items if crit and not ok and mid)
    soft_bad = sum(1 for *_,ok,mid,crit in items if not crit and not ok and not mid)
    soft_mid = sum(1 for *_,ok,mid,crit in items if not crit and not ok and mid)
    if crit_bad >= 1:
        v = "❌ 不能 OCR"
    elif crit_mid >= 1 or soft_bad >= 1 or soft_mid >= 2:
        v = "⚠️ 勉强"
    else:
        v = "✅ 能 OCR"
    return v, [it[:5] for it in items]

def analyze_one(path, roi=None, save_annot=True, outdir=None):
    img = load(path)
    if img is None:
        print(f"  ❌ 读不到 {path}"); return None
    H, W = img.shape[:2]
    if roi:
        x,y,w,h = roi
        x=max(0,x); y=max(0,y); w=min(w, W-x); h=min(h, H-y)
        crop = img[y:y+h, x:x+w].copy(); ox,oy = x,y
    else:
        crop = img.copy(); ox,oy = 0,0
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    # ★ 先自动找屏幕 —— 后面的清晰度/曝光/文字都在屏幕区域内算，
    #   否则整幅的黑背景会把清晰度拉得很低、把过曝稀释掉
    scr = find_screen(crop)
    if scr:
        _x, _y, _w, _h = scr
        gray_roi = gray[_y:_y+_h, _x:_x+_w]
    else:
        gray_roi = gray
    s  = sharpness(gray_roi)
    m  = moire_score(crop)
    over, dark_frac, mean = exposure_stats(gray_roi if scr else gray)
    if scr:
        sx, sy, sw, sh = scr
        scr_frac = sw / crop.shape[1] * 100
        gray_txt = gray[sy:sy+sh, sx:sx+sw]
    else:
        sx = sy = 0; sw, sh = crop.shape[1], crop.shape[0]; scr_frac = 100.0
        gray_txt = gray
        # ★ 没找到屏幕时给出【明确原因】，别让用户以为"字太小"
        gm, gsd = float(gray.mean()), float(gray.std())
        bright = float((gray > 128).mean()) * 100
        if gm < 40 or bright < 2:
            diag = (f"画面太暗（均值 {gm:.0f}，亮像素仅 {bright:.1f}%）"
                    f" -> 提高曝光，或关掉自动曝光后手动加档")
        elif gsd < 25:
            diag = f"画面过于平淡（标准差 {gsd:.0f}）-> 屏幕可能不在视野里"
        else:
            diag = "没找到像屏幕的区域 -> 让屏幕在画面里占更大比例，或用 --roi 手动框选"
        print(f"   ⚠️ {diag}")
    lines_raw, bw, char_h_cc = detect_lines(gray_txt)
    # 把行坐标映射回 crop 坐标系
    lines = [(y0+sy, y1+sy, nc, cw) for (y0, y1, nc, cw) in lines_raw]
    # 字高优先用【连通域量出来的】（投影行带高度会被上下留白撑大）
    char_h = char_h_cc if char_h_cc > 0 else (
        float(np.median([y1-y0 for y0,y1,_,_ in lines])) if lines else 0.0)

    v, items = verdict(s, m, over, char_h, dark_frac=dark_frac)

    print("─"*70)
    print(f"📄 {os.path.basename(path)}   {W}x{H}" + (f"   ROI {roi}" if roi else ""))
    if scr:
        print(f"   🖥  屏幕区域 ({ox+sx},{oy+sy}) {sw}x{sh}   占画面宽 {scr_frac:.0f}%"
              + ("  ✅ ≥1/3" if scr_frac>=33 else "  ⚠️ 偏小，字可能不够大"))
    print(f"   ★ 综合判定：{v}")
    for name, val, unit, ok, mid in items:
        flag = "✅" if ok else ("⚠️" if mid else "❌")
        print(f"     {flag} {name:<6} {val:>8.2f}{unit}")
    if lines:
        print(f"   ── 检测到 {len(lines)} 行文字 ──")
        for i,(y0,y1,nc,cw) in enumerate(lines[:8], 1):
            print(f"      行{i}: y {y0+oy:>4}~{y1+oy:<4}  字高 {y1-y0:>3}px  约 {nc:>3} 字  字宽中位 {cw:.1f}px")
        if len(lines)>8: print(f"      … 还有 {len(lines)-8} 行")
    else:
        print("   ⚠️ 没检测到文字行 —— 可能：① 画面里没有字 ② 曝光/对焦太差 ③ 试试 --roi 框住屏幕")

    if save_annot:
        ann = crop.copy()
        if scr:
            cv2.rectangle(ann,(sx,sy),(sx+sw,sy+sh),(255,180,0),2)
            cv2.putText(ann,"SCREEN",(sx+4,sy+22),cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,(255,180,0),2,cv2.LINE_AA)
        for y0,y1,nc,cw in lines:
            x0 = sx if scr else 0
            x1 = (sx+sw-1) if scr else ann.shape[1]-1
            cv2.rectangle(ann,(x0,y0),(x1,y1),(0,255,0),2)
            cv2.putText(ann,f"{y1-y0}px / ~{nc}chars",(max(4,x0+4),max(14,y0-5)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,0),3,cv2.LINE_AA)
            cv2.putText(ann,f"{y1-y0}px / ~{nc}chars",(max(4,x0+4),max(14,y0-5)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),1,cv2.LINE_AA)
        head=f"{v} | charH={char_h:.0f}px sharp={s:.0f} moire={m:.1f} over={over:.1f}%"
        cv2.putText(ann,head,(6,ann.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,0),4,cv2.LINE_AA)
        cv2.putText(ann,head,(6,ann.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2,cv2.LINE_AA)
        od = outdir or os.path.join(os.path.dirname(path),'annotated')
        os.makedirs(od, exist_ok=True)
        op = os.path.join(od, os.path.basename(path))
        save(op, ann)
        print(f"   标注图 → {op}")

    return dict(file=os.path.basename(path), w=W, h=H, verdict=v,
                char_h=char_h, sharp=s, moire=m, over=over, lines=len(lines))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('target')
    ap.add_argument('--roi', default=None, help='x,y,w,h —— 只看屏幕区域')
    ap.add_argument('--no-annot', action='store_true')
    a=ap.parse_args()
    roi = tuple(int(v) for v in a.roi.split(',')) if a.roi else None

    if os.path.isdir(a.target):
        files = sorted(glob.glob(os.path.join(a.target,'*.png')) +
                       glob.glob(os.path.join(a.target,'*.jpg')))
        files = [f for f in files if 'annotated' not in f]
    else:
        files = [a.target]
    if not files:
        print("没找到图片"); return
    print("="*70); print(f" 拍屏 OCR 可行性分析 —— {len(files)} 张"); print("="*70)
    res=[analyze_one(f, roi, not a.no_annot) for f in files]
    res=[r for r in res if r]
    if len(res)>1:
        print("\n"+"="*70); print(" 汇总"); print("="*70)
        print(f"  {'文件':<34}{'字高px':>8}{'清晰':>8}{'摩尔纹':>8}{'过曝%':>8}  判定")
        for r in res:
            print(f"  {r['file'][:33]:<34}{r['char_h']:>8.0f}{r['sharp']:>8.0f}"
                  f"{r['moire']:>8.1f}{r['over']:>8.1f}  {r['verdict']}")
        best=max(res, key=lambda r:r['char_h'])
        print(f"\n  ★ 最好的一张：{best['file']}（字高 {best['char_h']:.0f}px）")
        print(f"    判定阈值：字高>=25px 好 / 20~25 勉强 / <20 不能")

if __name__=='__main__': main()
