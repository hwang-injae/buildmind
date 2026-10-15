'use client';
// 위쪽 가운데 진행도 — 놓은 블록 / 전체 = %, DONE 이면 100 %(web/README 1장 ⑧). 진행표가 지금 조립(run_id) 것이 아니면 '진행 없음'
// (liveProgress — 10/15 PL '가++': task 를 다시 켜도 지난 조립 진행표가 남았다).
// 이름은 사람이 읽는 말로(lib/names — '벤치 V001' · '다리 1-3'). 숫자 · 막대는 자연스럽게 올라간다(CountUp · .fill transition, 10/14 저녁 PL).
import { partName, designName } from '@/lib/names';
import { liveProgress, type Robot } from '@/lib/ws';
import CountUp from './CountUp';

export default function ProgressTop({ robot }: { robot: Robot }) {
  const st = robot.state;
  const p = liveProgress(robot);
  const total = p?.parts.length ?? 0;
  const present = p?.parts.filter((b) => b.state === 'present').length ?? 0;
  const pct = st?.state === 'DONE' && total ? 100 : total ? Math.round((present / total) * 100) : 0;

  return (
    <div className="top-progress" aria-label="진행도">
      <div className="top-progress-head">
        <span className="muted">{st?.design_id ? designName(st.design_id) : '설계 없음'}</span>
        <strong>{total ? <><CountUp value={present} /> / {total} · <CountUp value={pct} />%</> : <span className="muted">진행 없음</span>}</strong>
      </div>
      <div className="bar">
        <div className="fill" style={{ width: `${pct}%` }} />
      </div>
      <div className="top-progress-sub muted">{st?.part_id ? `지금 ${partName(st.part_id, st.design_id)}` : ' '}</div>
    </div>
  );
}
