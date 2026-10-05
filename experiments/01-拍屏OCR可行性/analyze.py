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
    moire_ok=15,  moire_max=30,
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

# ── ② 摩尔纹：色度通道频谱里有没有"孤立的窄峰"（周期性干涉的标志）
def moire_score(bgr):
    ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    out = []
    for idx, name in ((1,'Cr'), (2,'Cb')):
        ch = ycrcb[:,:,idx].astype(np.float64)
        ch -= ch.mean()
        # 加窗，减少边缘泄漏
        h, w = ch.shape
        wy = np.hanning(h)[:,None]; wx = np.hanning(w)[None,:]
        F = np.fft.fftshift(np.abs(np.fft.fft2(ch * wy * wx)))
        cy, cx = h//2, w//2
        Y, X = np.ogrid[:h, :w]
        r = np.sqrt((Y-cy)**2 + (X-cx)**2)
        rmax = min(cy, cx)
        band = (r > 0.12*rmax) & (r < 0.92*rmax)      # 中高频环带
        vals = F[band]
        if vals.size < 50: out.append(0.0); continue
        out.append(float(np.percentile(vals, 99.95) / (np.median(vals) + 1e-9)))
    return max(out)

# ── ③ 曝光
def exposure_stats(gray):
    over  = float((gray > 250).mean() * 100)     # 过曝占比 %
    under = float((gray < 5).mean() * 100)       # 欠曝占比 %
    return over, under, float(gray.mean())

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

def detect_lines(gray):
    """返回 [(y0,y1, n_chars, char_w_median), ...]"""
    # 屏幕是自发光，用自适应阈值应对亮度不均
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
        if y1 - y0 < 8 or y1 - y0 > H*0.15: continue
        sub = bw[y0:y1, :]
        pv  = sub.sum(axis=0) / 255.0
        cols = _segments(pv > max(1, (y1-y0)*0.06), 2)
        # 合并过近的列段（汉字部件）
        merged, gap_thr = [], max(2, (y1-y0)*0.35)
        for c0, c1 in cols:
            if merged and c0 - merged[-1][1] < gap_thr: merged[-1] = (merged[-1][0], c1)
            else: merged.append((c0, c1))
        if len(merged) < 2: continue                     # 至少 2 个字才算一行
        lh = y1 - y0
        ws = [c1-c0 for c0,c1 in merged if 3 <= c1-c0]
        if not ws: continue
        cw = float(np.median(ws))
        # 汉字宽高比应在 0.35 ~ 2.5 之间；太离谱说明是噪点/背景
        if not (0.35 <= cw/lh <= 2.5): continue
        # 笔画密度：文字区域的墨迹占比通常 5%~60%
        dens = sub.sum()/255.0/(sub.size+1e-9)
        if not (0.02 <= dens <= 0.75): continue
        lines.append((y0, y1, len(merged), cw))
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
    lines, bw = detect_lines(gray)
    char_h = float(np.median([y1-y0 for y0,y1,_,_ in lines])) if lines else 0.0

    v, items = verdict(s, m, over, char_h)

    print("─"*70)
    print(f"📄 {os.path.basename(path)}   {W}x{H}" + (f"   ROI {roi}" if roi else ""))
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
