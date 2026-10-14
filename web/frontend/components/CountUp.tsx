'use client';
// 숫자가 바뀔 때 앞 값에서 새 값까지 자연스럽게 올라가게(10/14 저녁 PL) — 진행도 · 일치율 · 점 수 · 그리퍼 폭.
// 0.6초 동안 끝에서 느려지는 곡선(ease-out). 도는 중에 값이 또 바뀌면 지금 보이는 값에서 이어 간다. 움직임 줄이기 설정이면 바로 바뀐다.
import { useEffect, useRef, useState } from 'react';

/** 보이는 값(소수) — target 이 바뀌면 ms 동안 따라간다. 바깥 영향 없음 */
export function useCountUp(target: number, ms = 600): number {
  const [shown, setShown] = useState(target);
  const now = useRef(target); // 지금 보이는 값 — 도는 중에 target 이 바뀌면 여기서 다시 출발
  useEffect(() => {
    const start = now.current;
    const still = typeof window !== 'undefined' && window.matchMedia?.('(prefers-reduced-motion: reduce)').matches;
    if (start === target || still || !Number.isFinite(start)) {
      now.current = target;
      setShown(target);
      return;
    }
    const t0 = performance.now();
    let raf = 0;
    const step = (t: number) => {
      const k = Math.min(1, (t - t0) / ms);
      now.current = start + (target - start) * (1 - (1 - k) ** 3);
      setShown(now.current);
      if (k < 1) raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, [target, ms]);
  return shown;
}

/** 올라가는 숫자 하나(정수로 보임, 천 단위 쉼표) */
export default function CountUp({ value }: { value: number }) {
  return <span className="num">{Math.round(useCountUp(value)).toLocaleString()}</span>;
}
