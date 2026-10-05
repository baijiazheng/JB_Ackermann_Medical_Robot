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
    over  = float((gray > 250).mean() * 100)     # 过曝占比 %
    under = float((gray < 5).mean() * 100)       # 欠曝占比 %
    return over, under, float(gray.mean())

# ── ③.5 自动找屏幕区域（最大亮连通域）
def find_screen(bgr, min_area_frac=0.015):
    """屏幕是画面里最亮的一块。返回 (x,y,w,h) 或 None"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape
    if gray.mean() > 200:            # 整张都很亮 => 大概整幅就是屏幕
        return (0, 0, W, H)
    _, ot = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, lab, st, ce = cv2.connectedComponentsWithStats(ot, 8)
    if n <= 1: return None
    cands = []
    for i in range(1, n):
        x, y, w, h, area = st[i]
        if area < W*H*min_area_frac: continue     # 太小
        if w < 60 or h < 40: continue
        if not (0.25 <= w/h <= 8): continue        # 宽高比离谱的丢掉
        # 这块区域的"亮度"要足够（屏幕是亮的）
        if gray[y:y+h, x:x+w].mean() < 100: continue
        cands.append((area, x, y, w, h))
    if not cands: return None
    cands.sort(reverse=True)
    _, x, y, w, h = cands[0]
    return (x, y, w, h)

# ── ④ 字高：自适应二值化 → 水平投影分行 → 行内垂直投影分字
def _segments(mask, min_len):
    segs, s = [], None
    for i, v in enumerate(mask):
        if v and s is None: s = i
        elif not v and s is not None:
            if i - s >= min_len: segs.append((s, i))
            s = None
    if s is not None and len(mask) - s >= min_len: segs.append((s, len(mask)))
    return segs

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

def detect_lines(gray):
    """在【屏幕区域】内检测文字行（亮底黑字 -> Otsu 反相）"""
    gray = clean_screen_bg(gray)              # ★ 先清掉屏幕外的暗背景
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    frac = bw.mean() / 255.0
    if frac < 0.003 or frac > 0.6:            # Otsu 切错则退回自适应
        bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 35, 12)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN,
                          cv2.getStructuringElement(cv2.MORPH_RECT,(2,2)))
    H, W = bw.shape
    proj = bw.sum(axis=1) / 255.0
    thr  = max(W * 0.004, 3)                      # 行内至少有 0.4% 宽度的笔画
    rows = _segments(proj > thr, max(6, H//200))  # 文字行至少 6px 高
    lines = []
    for y0, y1 in rows:
        # 一行文字不会占半张图；也不该 <8px
        if y1 - y0 < 8 or y1 - y0 > H*0.30: continue
        sub = bw[y0:y1, :]
        pv  = sub.sum(axis=0) / 255.0
        cols = _segments(pv > max(1, (y1-y0)*0.06), 2)
        lh = y1 - y0
        # ★ 合并阈值只能用【字内部笔画间隙】的量级（≈5% 字高）。
        #   实测：汉字之间的间距是 8~15px（字高 100px）= 8~15%，
        #   若用 35% 会把相邻汉字并成一个 —— 这是踩过的坑。
        merged, gap_thr = [], max(2, lh * 0.05)
        for c0, c1 in cols:
            if merged and c0 - merged[-1][1] < gap_thr: merged[-1] = (merged[-1][0], c1)
            else: merged.append((c0, c1))
        ws = [c1-c0 for c0,c1 in merged if 3 <= c1-c0]
        if not ws: continue
        cw = float(np.median(ws))
        # 单段宽度 / 字高：汉字≈1、数字≈0.5、标点≈0.5 -> 允许 0.3~2.0
        if not (0.30 <= cw/lh <= 2.0): continue
        # 用宽度估算字数（比数段更准：段宽 3 倍字高说明那里有 3 个字）
        n_chars = int(sum(max(1, round((c1-c0)/lh)) for c0,c1 in merged))
        # 笔画密度：文字区域的墨迹占比
        dens = sub.sum()/255.0/(sub.size+1e-9)
        if not (0.02 <= dens <= 0.75): continue
        lines.append((y0, y1, n_chars, cw))
    return lines, bw

def verdict(sharp, moire, over, char_h):
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

    s  = sharpness(gray)
    m  = moire_score(crop)
    over, under, mean = exposure_stats(gray)

    # ★ 先自动找屏幕，再在屏幕内找文字
    scr = find_screen(crop)
    if scr:
        sx, sy, sw, sh = scr
        scr_frac = sw / crop.shape[1] * 100
        gray_txt = gray[sy:sy+sh, sx:sx+sw]
    else:
        sx = sy = 0; sw, sh = crop.shape[1], crop.shape[0]; scr_frac = 100.0
        gray_txt = gray
    lines_raw, bw = detect_lines(gray_txt)
    # 把行坐标映射回 crop 坐标系
    lines = [(y0+sy, y1+sy, nc, cw) for (y0, y1, nc, cw) in lines_raw]
    char_h = float(np.median([y1-y0 for y0,y1,_,_ in lines])) if lines else 0.0

    v, items = verdict(s, m, over, char_h)

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
            cv2.rectangle(ann,(0,y0),(ann.shape[1]-1,y1),(0,255,0),2)
            cv2.putText(ann,f"{y1-y0}px / ~{nc}chars",(6,max(14,y0-5)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,0),3,cv2.LINE_AA)
            cv2.putText(ann,f"{y1-y0}px / ~{nc}chars",(6,max(14,y0-5)),
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
