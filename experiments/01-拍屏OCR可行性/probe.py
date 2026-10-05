#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摄像头探测 —— 列出所有可用设备及其真实支持的分辨率

用法:  python3 probe.py [--dev-max N]
"""
import cv2, sys, argparse

CANDS = [(1920,1080),(1600,1200),(1280,960),(1280,720),(1024,768),(800,600),(640,480)]

def probe(dev):
    """返回该设备真实可用的分辨率列表"""
    out=[]
    for w,h in CANDS:
        cap=cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened(): return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,h)
        aw=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); ah=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ok,_=cap.read()
        cap.release()
        if ok and (aw,ah)==(w,h): out.append((w,h))
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--dev-max', type=int, default=8)
    a=ap.parse_args()
    print("="*66); print(" 摄像头探测"); print("="*66)
    best=None
    for d in range(a.dev_max):
        res=probe(d)
        if res is None:
            continue
        if not res:
            print(f"  /dev/video{d}: ⚠️ 能打开但读不到帧（可能是 metadata 口）"); continue
        mx=max(res, key=lambda x:x[0]*x[1])
        tag=''
        if best is None or mx[0]*mx[1] > best[1][0]*best[1][1]:
            best=(d,mx); tag=' ← ★ 最高'
        print(f"  /dev/video{d}: ✅ 支持 {len(res)} 种分辨率，最高 {mx[0]}x{mx[1]}{tag}")
        for w,h in res: print(f"        {w}x{h}")
    print("="*66)
    if best:
        d,mx=best
        print(f"  ★ 做 OCR 实验请用：")
        print(f"      python3 capture.py --dev {d} --size {mx[0]}x{mx[1]}")
        need = mx[0]*mx[1] >= 1920*1080
        print(f"      分辨率 {'✅ 达到 1080p（200万像素）' if need else '⚠️ 不足 1080p，OCR 会吃力'}")
    else:
        print("  ❌ 没找到可用摄像头")
    print("="*66)

if __name__=='__main__': main()
