#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""速度环带宽计算器（按当前固件参数算，改参数即可复算）

模型（1 kHz 离散，全部按代码里的实际算法建模）：
  1) 对象：       omega/U = Kv / (1 + tau*s)，Kv≈60 rpm/V，tau≈8.5 ms
  2) 测速：       filteredAngle 的 5 ms 差分  ->  1/s + 2.5 ms 群延迟
  3) 卡尔曼：     Q=0.0005, R=0.02  ->  稳态 kg，等效一阶极点 f_k = -ln(1-kg)/(2*pi*dt)
  4) 控制器：     PI, Kp=0.02 V/rpm, Ki=0.6 V/(rpm*s), dt=1 ms

输出：开环穿越频率、相位裕度、闭环 -3 dB 带宽、阶跃(t90/超调)
"""
import math, cmath

DT   = 1e-3      # 控制周期(s)
Q    = 0.0005    # 卡尔曼过程噪声
R    = 0.02      # 卡尔曼测量噪声
KP   = 0.02      # V/rpm
KI   = 0.6       # V/(rpm*s)
KV   = 60.0      # rpm/V（实测：2V->126rpm）
TAU  = 0.0085    # 机械时间常数(s) = J*R/(Kt*Ke)
WIN  = 5e-3      # 测速差分窗口(s)
UMAX = 15.0      # 电压限幅(V)

def kalman_kg(Q, R, dt, n=50000):
    P = 0.0
    for _ in range(n):
        Pm = P + Q
        kg = Pm / (Pm + R)
        P  = (1 - kg) * Pm
    return kg

def impulse_response(kg, N=6000):
    """u(k) -> speed_est(k) 的单位脉冲响应（含对象/卡尔曼/差分）"""
    w = th = x = 0.0
    hist = [0.0]*int(round(WIN/DT)+1)
    out = []
    for k in range(N):
        u = 1.0 if k == 0 else 0.0
        w  += DT/TAU*(KV*u - w)          # 对象
        th += w*2*math.pi/60*DT          # 角度
        x  += kg*(th - x)                # 卡尔曼
        hist.append(x); hist.pop(0)
        out.append((hist[-1]-hist[0])/WIN*60/(2*math.pi))
    return out

def open_loop(resp, f):
    w = 2*math.pi*f
    z = cmath.exp(1j*w*DT)
    C = KP + KI*DT/(1 - 1/z)                              # PI
    H = sum(resp[k]*cmath.exp(-1j*w*k*DT) for k in range(len(resp)))
    return C*H

def main():
    kg = kalman_kg(Q, R, DT)
    fk = -math.log(1-kg)/DT/(2*math.pi)
    resp = impulse_response(kg)
    # 穿越频率 + 相位裕度
    fc = pm = None
    prev = None
    for i in range(1, 8001):
        f = i*0.01
        L = open_loop(resp, f)
        if prev and abs(prev[1]) > 1 >= abs(L):
            fc, pm = f, 180+math.degrees(cmath.phase(L)); break
        prev = (f, abs(L))
    # 闭环 -3 dB（|T| 首次降到 0.707）
    b3 = None
    for i in range(1, 8001):
        f = i*0.01
        L = open_loop(resp, f)
        if abs(L/(1+L)) < 1/math.sqrt(2):
            b3 = f; break
    # 阶跃（时域线性仿真，不带限幅以外的一切非线性）
    n = int(2.0/DT); w = th = x = integ = 0.0
    hist = [0.0]*int(round(WIN/DT)+1); o = []
    for k in range(n):
        th += w*2*math.pi/60*DT
        x  += kg*(th-x); hist.append(x); hist.pop(0)
        est = (hist[-1]-hist[0])/WIN*60/(2*math.pi)
        err = 100.0 - est
        integ += KI*err*DT
        u = KP*err + integ
        w += DT/TAU*(KV*u - w); o.append(w)
    t90 = next(i*DT for i, v in enumerate(o) if v >= 90)
    print(f"卡尔曼: kg={kg:.4f}  等效带宽={fk:.1f} Hz")
    print(f"开环穿越频率 fc = {fc:.1f} Hz   相位裕度 PM = {pm:.0f} deg")
    print(f"闭环 -3 dB 带宽 = {b3:.1f} Hz")
    print(f"100 rpm 阶跃: 超调 {max(o)-100:+.1f}%  t90={t90*1e3:.0f} ms  (0.35/t90={0.35/t90:.1f} Hz)")

if __name__ == '__main__':
    main()
