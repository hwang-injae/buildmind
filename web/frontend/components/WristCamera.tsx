'use client';
// 손목 카메라 블록 검출 화면(web/README 1장 — E-67). 손목 비전(YOLO-seg)이 블록을 찾을 때마다 윤곽 · 글자까지 그려 보낸 JPEG 를
// 그대로 보여 준다(화면은 덧그리지 않음). 그림이 오면 /ws 로 번호만 오고, 그림은 /api/robot/wrist.jpg 로 받는다.
// 10/14 PL: 블록 고르기 순서(우선순위)가 핵심이라 크게 — 설계 3D 왼쪽 반 칸(10/14 저녁) · 확대(×1 ~ ×3, 그림을 누른 곳 중심) · 전체 화면.
// 그림 아래 글자 뜻 풀이(10/14 저녁 PL): 손목 비전 block_finder.draw_found 가 쓰는 '번호 자세 첫 글자 각도 [T]'와 윤곽 색 — 그쪽 글자가 바뀌면 같이 고친다.
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
      {w && (
        <div className="cam-legend">
          <span><i style={{ borderColor: '#00c800' }} />집을 수 있음</span>
          <span><i style={{ borderColor: '#ffc800' }} />다른 블록 위</span>
          <span><i style={{ borderColor: '#ff0000' }} />덮여 있음</span>
          <span title="손목 비전이 매긴 높이 순번이에요. 실제로 집는 블록은 로봇이 이 중에서 골라요(높은 층 눕힌 블록 → 세운 블록)">
            글자 <b>1 T +12</b> = 높은 순번 · 자세(<b>T</b> 눕힘 · <b>W</b> 옆으로 섬 · <b>L</b> 세움) · 돌아간 각도, 끝에 <b>T</b> 하나 더 = 기울어짐
          </span>
        </div>
      )}
    </section>
  );
}
