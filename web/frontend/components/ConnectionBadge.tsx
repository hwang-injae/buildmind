'use client';
// 연결 표시 — 화면 ↔ backend(/ws) · backend ↔ 브로커 · 로봇 PC 다리(연결 신호 3초, IRD 10.3).
// 점 3개는 위 띠에, 끊김 배너는 그 아래 크게(로봇 PC 가 끊기면 [조립] · [스캔] 막음 — 정지는 Ctrl+C · 펜던트 — 키로 정지를 보내는 프로그램은 없음, 10/14 PL).
import type { Rules } from '@/lib/types';
import type { Robot } from '@/lib/ws';

function Dot({ ok, label }: { ok: boolean; label: string }) {
  return (
    <span className={`dot ${ok ? 'ok' : 'bad'}`} title={ok ? '연결됨' : '끊김'}>
      <i /> {label}
    </span>
  );
}

/** 점 3개 + 저장소 글(10/14 PL — 설계 · 조립 기록을 DB 에 쓰는지 JSON 폴더에 쓰는지 사람이 보게, 연결 상태가 아니라 점 없음) */
export function ConnectionDots({ robot, store }: { robot: Robot; store?: Rules['store'] }) {
  return (
    <div className="badges">
      <Dot ok={robot.ws} label="웹 서버" />
      <Dot ok={robot.ws && robot.broker} label="브로커" />
      <Dot ok={robot.ws && robot.bridge} label="로봇 PC" />
      {store && (
        <span className="dot" title="설계 · 조립 기록을 저장하는 곳(웹 PC .env DATABASE_URL 로 고름)">
          저장소: {store === 'postgres' ? 'DB(PostgreSQL)' : 'JSON 폴더'}
        </span>
      )}
    </div>
  );
}

export function ConnectionBanners({ robot }: { robot: Robot }) {
  const robotPcLost = robot.ws && robot.broker && !robot.bridge;
  return (
    <>
      {!robot.ws && <div className="banner warn">웹 서버(backend) 연결 끊김 — 다시 붙는 중. 정지는 로봇 PC Ctrl+C · 펜던트로</div>}
      {robot.ws && !robot.broker && <div className="banner warn">브로커 연결 끊김 — 웹 PC 의 mosquitto 를 확인하세요</div>}
      {robotPcLost && (
        <div className="banner danger">로봇 PC 연결 끊김 — [조립] · [스캔]을 막았어요. 정지는 로봇 PC Ctrl+C · 펜던트로</div>
      )}
    </>
  );
}
