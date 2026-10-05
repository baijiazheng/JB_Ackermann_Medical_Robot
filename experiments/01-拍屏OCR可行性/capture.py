#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拍屏实验 · 拍照工具

预览 + 拍照，帮助在【预估比赛距离】上拍出能 OCR 的屏幕照片。

用法:
    python3 capture.py                          # 默认 /dev/video2 @ 1920x1080
    python3 capture.py --dev 0 --size 1280x720
    python3 capture.py --out my_shots

按键:
    空格  拍照保存
    + / - 手动曝光增减（需先按 f 切到手动）
    f     切换 自动/手动 曝光
    r     重置为自动曝光
    s     循环切换分辨率
    g     切换网格（10% 分格，帮助让屏幕占够画面）
    q/ESC 退出
"""
import cv2, os, sys, time, argparse

RES_LIST = [(1920,1080),(1280,960),(1280,720),(800,600),(640,480)]

def put(img, txt, y, color=(0,255,0), scale=0.6, thick=1):
    cv2.putText(img, txt, (10,y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thick+2, cv2.LINE_AA)
    cv2.putText(img, txt, (10,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

def draw_grid(img):
    h,w = img.shape[:2]
    for f in (0.1,0.3,0.5,0.7,0.9):
        cv2.line(img,(int(w*f),0),(int(w*f),h),(0,180,255),1)
        cv2.line(img,(0,int(h*f)),(w,int(h*f)),(0,180,255),1)
    # 中央 100px 见方的"像素尺"：用来直观判断字号
    cx,cy = w//2, h//2
    cv2.rectangle(img,(cx-50,cy-50),(cx+50,cy+50),(0,0,255),2)
    cv2.putText(img,"100px",(cx-48,cy-56),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1,cv2.LINE_AA)
    return img

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--dev', type=int, default=2)
    ap.add_argument('--size', default='1920x1080')
    ap.add_argument('--out', default='shots')
    ap.add_argument('--no-window', action='store_true', help='无窗口：延时后自动拍一张（远程/无显示环境）')
    ap.add_argument('--delay', type=float, default=3.0)
    a=ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    w,h = (int(x) for x in a.size.lower().split('x'))
    ri = RES_LIST.index((w,h)) if (w,h) in RES_LIST else 0

    cap=cv2.VideoCapture(a.dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"❌ 打不开 /dev/video{a.dev}"); sys.exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

    manual=False; exposure=cap.get(cv2.CAP_PROP_EXPOSURE)
    show_grid=True; n=0

    print(f"📷 /dev/video{a.dev}  目标 {w}x{h}")
    if a.no_window:
        print(f"   无窗口模式，{a.delay}s 后自动拍一张…")
        t0=time.time()
        while time.time()-t0 < a.delay: cap.read()
        ok,frame=cap.read()
        if ok:
            aw,ah=frame.shape[1],frame.shape[0]
            fn=os.path.join(a.out, time.strftime("shot_%Y%m%d_%H%M%S.png"))
            cv2.imwrite(fn, frame); print(f"✅ 已存 {fn}  ({aw}x{ah})")
        else: print("❌ 读帧失败")
        cap.release(); return

    print("   空格=拍照  +/-=曝光  f=手动/自动  r=重置  s=分辨率  g=网格  q=退出")
    while True:
        ok,frame=cap.read()
        if not ok: print("读帧失败"); break
        aw,ah=frame.shape[1],frame.shape[0]
        disp = draw_grid(frame.copy()) if show_grid else frame.copy()
        put(disp, f"/dev/video{a.dev}  {aw}x{ah}  {'MANUAL' if manual else 'AUTO'} exp={exposure:.0f}", 22)
        put(disp, "space=shot  +/-=exp  f=manual  r=auto  s=size  g=grid  q=quit", 46, (255,255,0), 0.5)
        cv2.imshow("capture - q to quit", disp)
        k=cv2.waitKey(1)&0xFF
        if k in (ord('q'),27): break
        elif k==ord(' '):
            n+=1
            fn=os.path.join(a.out, time.strftime("shot_%Y%m%d_%H%M%S_")+f"{n:02d}.png")
            cv2.imwrite(fn, frame); print(f"✅ [{n}] {fn}  ({aw}x{ah})")
        elif k==ord('f'):
            manual = not manual
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1 if manual else 3)
            print(f"   曝光 -> {'手动' if manual else '自动'}")
        elif k==ord('r'):
            manual=False; cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,3)
            cap.set(cv2.CAP_PROP_EXPOSURE, -6); print("   已重置为自动曝光")
        elif k in (ord('+'),ord('=')):
            exposure=cap.get(cv2.CAP_PROP_EXPOSURE)+1
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure); print(f"   曝光 {exposure:.0f}")
        elif k in (ord('-'),ord('_')):
            exposure=cap.get(cv2.CAP_PROP_EXPOSURE)-1
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure); print(f"   曝光 {exposure:.0f}")
        elif k==ord('s'):
            ri=(ri+1)%len(RES_LIST); w,h=RES_LIST[ri]
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,h)
            print(f"   请求 {w}x{h}")
        elif k==ord('g'):
            show_grid = not show_grid
    cap.release(); cv2.destroyAllWindows()
    print(f"\n本次拍了 {n} 张，存在 {a.out}/")
    if n: print(f"下一步:  python3 analyze.py {a.out}/")

if __name__=='__main__': main()
