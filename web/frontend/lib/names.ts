// 화면에 보일 이름 한 곳 — 코드 이름(설계 ID · 블록 이름 · 상태 코드 · 정지 이유 · 검사 오류)을 사람이 읽는 말로 바꾼다
// (10/10 황인재: 코드 같은 글 · 긴 글은 화면에 안 보이게). 모양(Template) 이름은 backend 한 곳(GET /api/designs/templates)에서 받는다.

let templateNames: Record<string, string> = {};

/** backend 가 준 모양 이름을 기억한다(page 가 처음 붙을 때 한 번) */
export function setTemplateNames(list: { id: string; name: string }[]) {
  templateNames = Object.fromEntries(list.map((t) => [t.id, t.name]));
}

const DESIGN_RE = /^(\d{3}(?:_[A-Z]+)+)_V(\d{3})$/; // <Template ID>_V<3자리>(E-84)

let displayNames: Record<string, string> = {}; // 사람이 붙인 표시 이름(W197) — 설계 ID → 이름

/** backend 설계 목록의 표시 이름을 기억한다(page 가 처음 붙을 때 · 목록을 받을 때마다) */
export function setDisplayNames(list: { design_id: string; display_name?: string | null }[]) {
  displayNames = Object.fromEntries(list.filter((d) => d.display_name).map((d) => [d.design_id, d.display_name as string]));
}

/** 이름 고치기 뒤 한 설계만 바꾼다(null = 지움) */
export function setDisplayName(id: string, name: string | null) {
  if (name) displayNames[id] = name;
  else delete displayNames[id];
}

/** '001_CHAIR_BENCH_V001' → '벤치 V001', V000 → '벤치 (기본)', 표시 이름이 있으면 '젠가 탑 4층 · 벤치 V001'(10/14 PL '나' — 원래 판도 같이).
 *  목록 · 3D 제목 · 로봇 패널 · 진행 줄 · 생성 · 스캔 글이 모두 이 한 곳을 쓴다. 꼴이 다르면 받은 그대로 */
export function designName(id?: string | null): string {
  if (!id) return '';
  const m = DESIGN_RE.exec(id);
  if (!m) return id;
  const name = templateNames[m[1]] ?? m[1];
  const base = m[2] === '000' ? `${name} (기본)` : `${name} V${m[2]}`;
  return displayNames[id] ? `${displayNames[id]} · ${base}` : base;
}

/** 역할 → 한국어 · 3D 색(설계 3D · 후보 미리보기 · 블록 이름이 같이 쓴다) */
export const ROLE: Record<string, { ko: string; color: string }> = {
  LEG: { ko: '다리', color: '#b7794a' },
  SEAT: { ko: '좌판', color: '#e3b341' },
  BACK: { ko: '등받이', color: '#5b8fd6' },
  BEAM: { ko: '보', color: '#8f7ad6' },
  TOP: { ko: '상판', color: '#e8a33c' },
  BASE: { ko: '바닥 받침', color: '#6fa36b' },
  COLUMN: { ko: '기둥', color: '#d0705f' },
};
export const roleOf = (role?: string) => (role ?? '').split('_')[0]; // LEG_WHEEL → LEG (옵션은 나누지 않음)

/** '001_CHAIR_BENCH_V001_LEG_001_03' → '다리 1-3'(부품 그룹 1의 3번째 부품) */
export function partName(partId?: string | null, designId?: string | null): string {
  if (!partId) return '';
  const rest = designId && partId.startsWith(designId + '_') ? partId.slice(designId.length + 1) : partId;
  const m = /^([A-Z]+)(?:_[A-Z]+)?_(\d{3})_(\d{2})$/.exec(rest);
  return m ? `${ROLE[m[1]]?.ko ?? m[1]} ${Number(m[2])}-${Number(m[3])}` : rest;
}

const PART_IN_TEXT = /(\d{3}(?:_[A-Z]+)+_V\d{3})_([A-Z]+(?:_[A-Z]+)?_\d{3}_\d{2})/g;
const DESIGN_IN_TEXT = /\d{3}(?:_[A-Z]+)+_V\d{3}/g;

/** 로봇 · backend 가 보낸 글 속 부품 · 설계 ID 를 사람이 읽는 이름으로(작업 관리자 글은 고치지 않고 화면에서만 —
 *  '001_CHAIR_BENCH_V000_LEG_001_03 를 집으러 갑니다' → '다리 1-3 를 집으러 갑니다', 10/14 17시 PL: 화면에 코드 안 씀) */
export const readable = (text?: string | null): string =>
  (text ?? '').replace(PART_IN_TEXT, (_, d: string, p: string) => partName(`${d}_${p}`, d)).replace(DESIGN_IN_TEXT, (d) => designName(d));

export const MADE_KO: Record<string, string> = { cad: '도면', web: '웹 글', voice: '음성', scan: '스캔' };

const STATE_KO: Record<string, string> = {
  IDLE: '대기', READY: '출발 대기', CHECK: '관측 중', SELECT: '다음 블록 고르는 중', PICK_PLACE: '집고 놓는 중',
  VERIFY: '놓은 블록 확인 중', WAIT_SUPPLY: '블록 공급 기다림', WAIT_HMI: '웹 연결 기다림', SCAN_MOVE: '스캔 — 이동',
  SCAN_CAPTURE: '스캔 — 촬영', SCAN_INFER: '스캔 — 계산', SCAN_REVIEW: '스캔 결과 확인', STOPPED: '멈춤',
  RECOVER: '다시 시작 중', ERROR: '오류', DONE: '완성',
};

// A4 놓은 블록 확인에서 사람을 기다림 — 작업 관리자는 WAIT_SUPPLY 를 쓰되 알림 id 를 공급 대기와 다르게 낸다(10/15 PL '가', 한석형).
// 이 id 면 상태 · [계속] 글을 공급 말 대신 이것으로. 받침 경유 공급 대기(no_match_block · tilted_block 등)는 그대로 '블록 공급 기다림'
export const PLACED_CHECK_ID = 'placed_check';
export const PLACED_CHECK = '놓은 블록 확인 — 손으로 고친 뒤 계속';

/** 작업 관리자 상태 코드 → 한국어(모르는 코드는 그대로). messageId(state/2 message_id)를 주면 WAIT_SUPPLY 중 A4 놓은 블록 확인을 가른다 */
export const stateName = (s?: string | null, messageId?: string | null) =>
  s === 'WAIT_SUPPLY' && messageId === PLACED_CHECK_ID ? PLACED_CHECK : s ? STATE_KO[s] ?? s : '';

/** 정지 이유(IRD 7장) → 누가 · 왜 */
export function stopReasonKo(reason: string): string {
  if (reason.startsWith('ROBOT_ALARM:')) return '로봇 알람으로 멈췄어요';
  const ko: Record<string, string> = {
    STOP_WEB: '화면의 정지 버튼으로 멈췄어요', STOP_KEY: '로봇 PC 키로 멈췄어요', STOP_TASK: '작업 관리자가 멈췄어요',
    TIMEOUT: '시간 초과 — 작업 관리자가 먼저 세웠어요', CTRL_C: '프로그램 종료로 멈췄어요',
    VISION_LOST: '카메라 신호가 끊겨 멈췄어요',   // IRD 7장 · SDD — 정지 노드 연결은 나중이지만 이름은 정함(10/14 PL A4 ②)
    STOP_VOICE: '음성 "정지"로 멈췄어요',        // voice.py 정지 요청(source voice → 정지 노드가 STOP_VOICE, W046 · 10/14 PL A4) — 대표 낱말 '정지'(10/14 PL), '멈춰' · '스톱' · '그만'도 받음
  };
  return ko[reason] ?? '멈췄어요';
}

/** 검사 오류 한 줄 → 짧은 말(자세한 글은 마우스를 올리면 보이게 부르는 쪽이 title 로) */
export function checkReasonKo(detail: string): string {
  const table: [string, string][] = [
    ['공중', '받침 없는 블록'], ['여유', '넘어질 수 있음'], ['겹침', '블록이 겹침'], ['grasp', '로봇이 잡기 어려움'],
    ['잡을 면', '로봇이 잡기 어려움'], ['작업영역', '작업 공간을 넘음'], ['작업대', '작업 공간을 넘음'], ['역할', '이름 규칙 위반'],
    ['group', '부품 그룹 규칙 위반'], ['형식', '형식 오류'], ['상한', '블록이 너무 많음'],
  ];
  return table.find(([k]) => detail.includes(k))?.[1] ?? '규칙 위반';
}

// ---------- 거절 · 실패 글 — '무엇이 문제 — 무엇을 할지' 짧은 한 줄(10/14 17시 PL: 명령 · 프로그램 이름 · 코드는 화면에 안 씀) ----------
// 화면이 직접 만드는 글만 여기. 작업 관리자(state/2 · 응답 message)가 보낸 글은 고치지 않고 그대로 보인다.
export const URGENT_STOP = '급하면 펜던트 비상정지';
// 시간 초과 뒤 정지 확인 전 ERROR(state/2 message_id stop_pending — 10/14 PL 안 A): 작업 관리자가 stopped 신호 전엔 [다시 시작]을 무시한다.
// [정지](또는 펜던트)로 stopped 를 받게 하면 STOPPED → 그 뒤 [다시 시작]이 된다
export const STOP_PENDING = '정지 확인 전이에요 — [정지]를 누른 뒤 [다시 시작]';
// 정지가 아닌데 그리퍼가 블록을 쥐고 있음 — 작업 관리자가 시작 점검에서 거절한다. [그리퍼 열기]는 정지(STOPPED)에서만 받는다(W183)
export const GRIP_HELD = '그리퍼가 블록을 쥐고 있어요 — 블록을 받치고 [정지] → [그리퍼 열기]';
// 음성 시동어(#171 voice.py — 마이크 버튼 뒤 이 말을 들어야 녹음을 시작한다)
export const WAKE_WORD = 'hello rokey'; // 영어로 보임(10/14 PL)
export const NO_WEB = '웹 서버와 연결이 안 돼요 — 웹 PC가 켜져 있는지 보세요';

/** 거절 까닭 코드 → 안내(응답에 message 가 없을 때). '' = 맞는 코드 없는 거절 — 까닭 글은 작업 관리자가 상태 칸에 보냄(E-111 A3) */
const REJECT_HINT: Record<string, string> = {
  STOPPED: '멈춤 중이에요 — 원인을 없애고 [다시 시작]',
  BUSY: '로봇이 다른 일을 하는 중이에요 — 끝난 뒤 다시',
  TIMEOUT: '로봇 PC 답이 늦어요 — 상태 칸이 바뀌었는지 보세요',
  ERROR: '로봇 PC에서 실패했어요 — 상태 칸을 보세요',
  '': '로봇이 받지 않았어요 — 상태 칸을 보세요',
};

/** 버튼 하나의 거절 한 줄. message 가 있으면 그대로(보낸 쪽 글), 없으면 까닭 코드 안내(코드 자체는 부르는 쪽 title 로만). 입력: 버튼 이름 · 응답(success false) */
export function rejectText(label: string, r: { reason?: string | null; message?: string | null }): string {
  if (r.message) return `${label} 안 됨 — ${readable(r.message)}`;
  return `${label} 안 됨 — ${REJECT_HINT[r.reason ?? ''] ?? REJECT_HINT['']}`;
}
