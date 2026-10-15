'use client';
// 손목 카메라 블록 검출 화면(web/README 1장 — E-67). 손목 비전(YOLO-seg)이 블록을 찾을 때마다 윤곽 · 글자까지 그려 보낸 JPEG 를
// 그대로 보여 준다(화면은 덧그리지 않음). 그림이 오면 /ws 로 번호만 오고, 그림은 /api/robot/wrist.jpg 로 받는다.
// 10/14 PL: 블록 고르기 순서(우선순위)가 핵심이라 크게 — 설계 3D 왼쪽 반 칸(10/14 저녁) · 확대(×1 ~ ×3, 그림을 누른 곳 중심) · 전체 화면.
// 손목 그림은 세 가지(10/14 22시 PL — 그림은 손목 비전이 그림, 화면은 덧그리지 않음): 공급 구역(block_finder.draw_found — 낱말 FLAT · EDGE ·
// STAND · TILT n, 고른 뒤 우선순위 번호 동그라미 · 고른 블록 칠하기) · 받침 다시 재기(draw_spare — 빨간 목표 상자 · 초록 잰 블록 · dx dy · dyaw) ·
// 작업대(놓은 부품 설계 자리). 그림 아래: 고른 까닭 한 줄(state/2 pick.reason — 그림 속 한글 대신 화면 글, PL) + 접어 둔 '그림 보는 법'.
// 그쪽 글자 · 색이 바뀌면 이 풀이도 같이 고친다.
// 관측 자세가 높아 블록이 작게 보여서 확대한다. 배율 · 중심은 이 브라우저에만 기억(localStorage — 못 쓰면 기억 없이 그대로 동작).
import { useEffect, useState, type MouseEvent } from 'react';
import { wristUrl } from '@/lib/api';
import type { Robot } from '@/lib/ws';

const ZOOMS = [1, 1.5, 2, 3];
const KEY = 'd2.wristZoom';

interface View { z: number; ox: number; oy: number } // 배율 · 확대 중심(그림 안 %, 0 ~ 100)
const PLAIN: View = { z: 1, ox: 50, oy: 50 };

function loadView(): View {
  try {
    const v = JSON.parse(localStorage.getItem(KEY) ?? 'null');
    if (v && ZOOMS.includes(v.z) && Number.isFinite(v.ox) && Number.isFinite(v.oy)) return v;
  } catch {
    /* 저장소를 못 쓰면 기본 보기 */
  }
  return PLAIN;
}

export default function WristCamera({ robot }: { robot: Robot }) {
  const w = robot.wrist;
  const pick = robot.state?.pick; // 작업 관리자가 고른 블록(조립 중 · 그 블록을 놓을 때까지)
  const [now, setNow] = useState(() => Date.now());
  const [view, setView] = useState<View>(PLAIN);
  const [full, setFull] = useState(false);

  useEffect(() => {
    const t = setInterval(() => setNow(Date.now()), 1000); // '몇 초 전' 글을 1초마다 고친다
    return () => clearInterval(t);
  }, []);
  useEffect(() => setView(loadView()), []); // 정적 그림(서버 렌더)과 첫 화면이 같게 — 기억은 그린 뒤 읽는다
  useEffect(() => {
    try {
      localStorage.setItem(KEY, JSON.stringify(view));
    } catch {
      /* 기억 못 해도 보기는 그대로 */
    }
  }, [view]);
  useEffect(() => {
    if (!full) return;
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && setFull(false);
    window.addEventListener('keydown', esc);
    return () => window.removeEventListener('keydown', esc);
  }, [full]);

  /** 그림을 누른 곳을 확대 중심으로 — 보이는 자리(확대된 화면, %) → 원래 그림 자리(%)로 바꿔 옮긴다. ×1 이면 ×2 로 바로 확대 */
  function aim(e: MouseEvent<HTMLDivElement>) {
    const r = e.currentTarget.getBoundingClientRect();
    const vx = ((e.clientX - r.left) / r.width) * 100;
    const vy = ((e.clientY - r.top) / r.height) * 100;
    const clamp = (n: number) => Math.min(100, Math.max(0, n));
    setView((v) => ({ z: v.z === 1 ? 2 : v.z, ox: clamp(v.ox + (vx - v.ox) / v.z), oy: clamp(v.oy + (vy - v.oy) / v.z) }));
  }

  const age = w ? Math.max(0, Math.round((now - w.at) / 1000)) : 0;
  return (
    <section className={`panel wrist-panel${full ? ' wrist-full' : ''}`}>
      <div className="panel-head">
        <h2>손목 카메라</h2>
        <span className="muted head-sub" title={w ? new Date(w.at).toTimeString().slice(0, 8) : undefined}>
          {w ? `블록 검출 — ${age}초 전` : ''}
        </span>
        {w && (
          <div className="zoom-row" role="group" aria-label="확대">
            {ZOOMS.map((z) => (
              <button key={z} className={`head-btn${view.z === z ? ' on' : ''}`} aria-pressed={view.z === z}
                onClick={() => setView((v) => (z === 1 ? PLAIN : { ...v, z }))}>
                ×{z}
              </button>
            ))}
            <button className="head-btn" onClick={() => setFull((f) => !f)} title="Esc 로 닫기">
              {full ? '닫기' : '전체 화면'}
            </button>
          </div>
        )}
      </div>
      {w ? (
        <div className="cam" onClick={aim} title="누른 곳을 중심으로 확대해요(×1 이면 ×2 로)">
          <img src={wristUrl(w.seq)} alt="손목 카메라 블록 검출 그림"
            style={{ transform: `scale(${view.z})`, transformOrigin: `${view.ox}% ${view.oy}%` }} />
        </div>
      ) : (
        <div className="cam cam-idle">블록을 찾을 때 그림이 와요</div>
      )}
      {pick?.reason && (
        <p className="pick-reason">
          <span className="rank-badge" aria-label="1순위">1</span> 먼저 집을 블록 — {pick.reason}
        </p>
      )}
      {w && (
        <details className="cam-help">
          <summary>그림 보는 법</summary>
          <div className="cam-legend">
            <b>공급 구역</b>
            <span><i style={{ borderColor: '#00c800' }} />집을 수 있음</span>
            <span><i style={{ borderColor: '#ffc800' }} />다른 블록 위</span>
            <span><i style={{ borderColor: '#ff0000' }} />덮여 있음</span>
            <span><span className="rank-badge small">1</span> 고르는 순서 · 칠한 블록 = 지금 집을 블록</span>
            <span><b>FLAT</b> 눕힘 · <b>EDGE</b> 옆으로 섬 · <b>STAND</b> 세움 · <b>TILT 8</b> = 8° 기울어짐</span>
          </div>
          <div className="cam-legend">
            <b>받침</b>
            <span><i style={{ borderColor: '#ff0000' }} />맞춰야 할 자리</span>
            <span><i style={{ borderColor: '#00c800' }} />잰 블록</span>
            <span><b>dx dy</b> = 어긋난 거리(mm) · <b>dyaw</b> = 틀어진 각도</span>
          </div>
          <div className="cam-legend">
            <b>작업대</b>
            <span>놓은 부품의 설계 자리 —</span>
            <span><i style={{ borderColor: '#00c800' }} />있음</span>
            <span><i style={{ borderColor: '#ffa500' }} />확인 못 함</span>
            <span><i style={{ borderColor: '#ff0000' }} />없음</span>
          </div>
        </details>
      )}
    </section>
  );
}
