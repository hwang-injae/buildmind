# -*- coding: utf-8 -*-
"""VoiceListener — 웹 PC 호스트 음성: 화면 마이크 버튼 → 시동어 → 요청 듣기 → 받아 적기 → MQTT (E-105, W045 · W120 · W046).

흐름(10/13 PL E-105): 화면 마이크 버튼 → backend 가 d2/hmi/voice/listen(voice_listen/1) → 여기서 듣기 대기 →
시동어 "hello rokey"(openWakeWord) → 높은 '삑' → 요청 녹음(말이 끊기면 끝 — Silero VAD) → 낮은 '삑' → 받아 적기(OpenAI STT) →
  · '멈춰 · 정지 · 스톱 · 그만' 짧은 말 → d2/safety/stop/req {source: voice, reason: ''} → 다리 → 정지 노드(STOP_VOICE, 보조 정지 W046)
  · 그 밖 → d2/hmi/intent {schema intent/1, intent: request_design, text, confidence} → backend → 화면이 글 창에 넣고 바로 생성(W045 · W120)
음성으로 설계 고르기 · 조립 출발은 하지 않는다(E-96 ⑥ · E-105 ③). 진짜 정지는 펜던트 · 화면 [정지] · Ctrl+C 그대로.

인식을 높이려고 넣은 것(10/14 민범진):
  ① 말 끝 자동 감지 — 말이 END_SILENCE_S 끊기면 녹음 끝(최대 MAX_RECORD_S). 말소리가 없으면 받아 적기를 안 부른다
     (STT 는 무음에 '시청해 주셔서 감사합니다' 같은 자막식 문장이나 힌트를 그대로 내는 일이 있다 — 그런 글도 버림)
  ② 받아 적기 = gpt-4o-mini-transcribe(실패하면 whisper-1 한 번) + language ko + 가구 낱말 힌트(HINT)
  ③ '멈춰' 빠른 길 — 정지 낱말로만 된 말('멈춰' · '정지요' · '스톱')은 받아 적자마자 바로 정지 요청('그만큼 높게'는 요청)
  ④ '들었어요' 소리 — 시동어 들음(높은 '삑') · 녹음 끝(낮은 '삑'). 말하는 TTS 가 아니다(TTS 는 없앤 기능). 소리 장치가 없으면 소리만 건너뜀
  ⑤ 확신도 — gpt-4o 계열 logprobs 평균을 intent/1 confidence 칸에(0 ~ 1, 못 구하면 null). design_gen 이 오인식 바로잡기에 쓸 수 있게

실행(웹 PC 호스트 — 컨테이너 밖, 마이크 · 스피커가 있는 곳):
  python3 web/backend/voice.py [--wakeword <hello_rokey_8332_32.tflite>] [--threshold 0.3] [--mic <장치 번호>]
  설정: web/.env(있으면) · 환경 변수 — OPENAI_API_KEY(필수) · MQTT_HOST · MQTT_PORT(웹 PC 는 compose 의 D2_WEB_IPV4 주소) ·
        D2_WAKEWORD_MODEL(--wakeword 대신) · D2_STT_MODEL(비우면 gpt-4o-mini-transcribe)
  준비 점검: python3 web/backend/voice.py --check — 마이크 · 스피커 · 시동어 모델 · 키 · 브로커를 ✅ ❌ 로
바깥 영향: 마이크 녹음(누를 때만) · 스피커 '삑' · OpenAI STT 호출 · MQTT 발행(d2/hmi/intent · d2/safety/stop/req). 로봇은 정지 요청만.
실패 때: 시동어가 WAKE_WAIT_S 안에 안 들리면 · 말이 없으면 · 받아 적기가 실패하면 아무것도 보내지 않고 로그만(화면은 20초 뒤 안내).
"""
import argparse
import json
import math
import os
import queue
import re
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'd2_bridge'))
from d2_bridge.bridge_codec import req_topic, res_topic  # noqa: E402  (다리 · backend 와 같은 토픽 규칙)

SAMPLE_RATE = 16000          # openWakeWord · Silero VAD 입력(16 kHz mono int16)
FRAME = 1280                 # 80 ms — openWakeWord 권장 단위(강의 wakeup_word.py 와 같음)
FRAME_S = FRAME / SAMPLE_RATE
WAKE_THRESHOLD = 0.3         # 강의 값. VAD_GATE 와 같이 써서 소음 오작동을 줄인다 — 실측으로 --threshold 조정
VAD_GATE = 0.5               # 시동어는 말소리(VAD ≥ 이 값)일 때만 인정(openWakeWord vad_threshold)
VAD_SPEECH = 0.5             # 녹음 중 이 값 이상이면 말소리 프레임
WAKE_WAIT_S = 6.0            # 누른 뒤 시동어를 기다리는 시간. 화면은 누른 뒤 20초 안에 문장이 와야 한다(GeneratePanel LISTEN_MS)
SPEECH_START_WAIT_S = 3.0    # 시동어 뒤 이 안에 말을 시작하지 않으면 그만
END_SILENCE_S = 0.8          # 말이 시작된 뒤 이만큼 조용하면 녹음 끝
MAX_RECORD_S = 7.0           # 녹음 상한(시동어 6 + 삑 0.3 + 녹음 7 + 받아 적기 몇 초 < 화면 기다림 20초)
MIN_SPEECH_S = 0.3           # 말소리 프레임이 이보다 적으면 받아 적기를 안 부른다
BEEP_SKIP_S = 0.3            # '삑' 낸 뒤 이만큼은 녹음에서 뺀다(마이크가 '삑'(0.15초) + 출력 지연을 듣는다 — 사람은 '삑' 뒤에 말한다)
BEEP_HIGH_HZ, BEEP_LOW_HZ, BEEP_S = 1320, 660, 0.15
STT_MODELS = ('gpt-4o-mini-transcribe', 'whisper-1')   # 앞이 실패하면 뒤 한 번
STT_TIMEOUT_S = 8.0          # 받아 적기 한 번 상한(OpenAI 기본 600초면 인터넷이 막힐 때 몇 분 멈춘다)
NO_FALLBACK = ('AuthenticationError', 'PermissionDeniedError', 'RateLimitError')   # 키 · 권한 · 크레딧 — 다른 모델도 똑같이 실패
HINT = ('젠가 블록으로 만드는 가구 설계 요청이다. 나올 수 있는 낱말: 의자, 책상, 벤치, 등받이, 다리, 좌판, 상판, 기둥, 받침, 블록, '
        '높게, 낮게, 넓게, 좁게, 길게, 짧게, 한 층, 두 층.')   # 정지 낱말은 넣지 않는다 — STT 가 힌트 끝을 그대로 내면 잘못된 정지가 된다
# 정지 말 = 정지 낱말(+ 요 · 줘 · 라 · 주세요)만으로 된 글(반복 가능). '그만큼 높게' · '정지해 있는 의자' 같은 요청은 정지가 아니다
STOP_RE = re.compile(r'^(?:(?:멈춰|멈추어|정지|스톱|스탑|그만|stop)(?:해)?(?:요|줘|라|주세요)?)+$')
HALLUCINATIONS = ('시청해주셔서감사합니다', '시청해주셔서고맙습니다', '구독과좋아요', '구독좋아요', '다음영상에서만나요',
                  '감사합니다', 'mbc뉴스')   # 무음 · 잡음에 STT 가 지어내는 자막식 문장(공백 · 문장부호 뺀 꼴)
HALLUCINATION_HEADS = ('자막제공', '자막by', '자막은')   # 이렇게 시작하면 자막 꼬리글
HINT_ECHO_CHARS = 10         # 받아 적은 글(이 길이 이상)이 힌트 안에 그대로 있으면 힌트를 되읽은 것 — 버림
INTENT_TOPIC = 'd2/hmi/intent'
LISTEN_TOPIC = 'd2/hmi/voice/listen'
STOP_SERVICE = '/d2/safety/stop'


def squash(text):
    """비교용 — 소문자로, 공백 · 문장부호를 뺀다. 예: '멈춰!' → '멈춰', 'Stop.' → 'stop'."""
    return re.sub(r'[\s\.,!?~·…"\'()\[\]-]+', '', (text or '').lower())


def is_stop(text):
    """받아 적은 글이 '멈춰' 말인가 — 정지 낱말로만 된 글(STOP_RE)일 때만. 입력 글자, 출력 bool."""
    return bool(STOP_RE.match(squash(text)))


def is_hallucination(text):
    """버릴 글인가 — 자막식 문장(목록과 같거나 자막 꼬리글로 시작) 또는 힌트를 되읽은 글. 요청 속 '감사합니다'는 살림."""
    s = squash(text)
    return (s in HALLUCINATIONS or s.startswith(HALLUCINATION_HEADS)
            or (len(s) >= HINT_ECHO_CHARS and s in squash(HINT)))


def confidence_from_logprobs(logprobs):
    """STT logprobs(토큰마다 logprob) → 확신도 0 ~ 1 = exp(평균 logprob). 없으면 None. 바깥 영향 없음."""
    vals = [lp.get('logprob') if isinstance(lp, dict) else getattr(lp, 'logprob', None) for lp in (logprobs or [])]
    vals = [v for v in vals if isinstance(v, (int, float)) and math.isfinite(v)]
    return round(math.exp(sum(vals) / len(vals)), 3) if vals else None


def intent_payload(text, confidence):
    """intent/1 request_design JSON 글자(IRD 6장 — design_id · mode 는 null). 화면은 마이크를 누른 뒤 온 것만 받는다."""
    return json.dumps({'schema': 'intent/1', 'stamp': round(time.time(), 3), 'intent': 'request_design', 'text': text,
                       'design_id': None, 'confidence': confidence, 'mode': None}, ensure_ascii=False)


def load_env_file(path):
    """web/.env 를 읽어 환경 변수에 없는 값만 채운다(KEY=VALUE 줄, # 주석 무시). 파일이 없으면 아무것도 안 함. 키는 로그에 쓰지 않는다."""
    try:
        lines = Path(path).read_text(encoding='utf-8').splitlines()
    except OSError:
        return
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#') and '=' in line:
            k, v = line.split('=', 1)
            if v.strip():
                os.environ.setdefault(k.strip(), v.strip())


class VoiceListener:
    """버튼 한 번 = 한 번 듣기. 마이크 · 시동어 · VAD · 받아 적기 · 발행 · 소리를 바깥에서 받아(시험은 가짜) 순서만 정한다.

    입력(모두 바깥에서 넣음):
      open_mic() → mic(read() = int16 FRAME 개 numpy 배열, close()) · new_wake() → wake(score(frame) = 0 ~ 1)
      vad(frame) → 0 ~ 1 · stt(wav_bytes) → (text, confidence, model) · publish(topic, payload) · beep(hz) · log(글자)
    바깥 영향: listen_once 가 마이크를 열고 닫는다 · publish 로 intent 또는 정지 요청 · beep 로 소리.
    실패: 각 단계가 안 되면 그 자리에서 끝내고 {'kind': 'none', 'why': …} — 아무것도 발행하지 않는다.
    """

    def __init__(self, open_mic, new_wake, vad, stt, publish, beep, log=print, threshold=WAKE_THRESHOLD):
        """부품을 받는다. threshold = 시동어 점수 문턱(0 ~ 1)."""
        self.open_mic, self.new_wake, self.vad, self.stt = open_mic, new_wake, vad, stt
        self.publish, self.beep, self.log, self.threshold = publish, beep, log, threshold

    def listen_once(self):
        """한 번 듣기 → {'kind': 'intent' | 'stop' | 'none', 'text', 'why', 'times'}. 마이크는 끝에 늘 닫는다."""
        t0 = time.monotonic()
        wake = self.new_wake()                       # 모델을 먼저 읽고 마이크를 연다 — 읽는 동안 소리가 쌓여 넘치지 않게
        reset = getattr(self.vad, 'reset', None)
        if reset is not None:
            reset()                                  # 지난 누름의 VAD 상태를 지운다
        mic = self.open_mic()
        try:
            if not self._wait_wake(mic, wake):
                return self._none('시동어를 못 들었어요', t0)
            t_wake = time.monotonic()
            self._beep(BEEP_HIGH_HZ)
            audio, speech_s = self._record(mic)
        finally:
            mic.close()
        self._beep(BEEP_LOW_HZ)
        if speech_s < MIN_SPEECH_S:
            return self._none(f'말소리가 없어요({speech_s:.1f}초)', t0)
        try:
            text, conf, model = self.stt(to_wav(audio))
        except Exception as e:                       # 키 · 크레딧 · 인터넷 — 화면은 20초 뒤 '못 받았어요'
            return self._none(f'받아 적기 실패: {type(e).__name__}: {e}', t0)
        text = (text or '').strip()
        times = {'wake_s': round(t_wake - t0, 2), 'speech_s': round(speech_s, 2), 'total_s': round(time.monotonic() - t0, 2)}
        if not text or is_hallucination(text):
            return self._none(f'받아 적은 글을 버림({text!r})', t0)
        if is_stop(text):
            self.publish(req_topic(STOP_SERVICE), json.dumps({'source': 'voice', 'reason': '', 'req_id': str(uuid.uuid4())}))
            self.log(f'[음성] 정지 요청 보냄 — "{text}" ({model}, {times})')
            return {'kind': 'stop', 'text': text, 'why': '', 'times': times}
        self.publish(INTENT_TOPIC, intent_payload(text, conf))
        self.log(f'[음성] 요청 보냄 — "{text}" (확신도 {conf}, {model}, {times})')
        return {'kind': 'intent', 'text': text, 'why': '', 'times': times}

    def _wait_wake(self, mic, wake):
        """시동어를 WAKE_WAIT_S 동안 기다린다(wake 는 누를 때마다 새 모델 — 지난 누름의 소리가 남아 바로 다시 걸리지 않게, 강의 방식)."""
        for _ in range(int(WAKE_WAIT_S / FRAME_S)):
            if wake.score(mic.read()) >= self.threshold:
                return True
        return False

    def _record(self, mic):
        """요청 녹음 — '삑'이 섞인 앞 BEEP_SKIP_S 는 버리고, 말이 시작된 뒤 END_SILENCE_S 조용하면 끝(최대 MAX_RECORD_S).
        말이 SPEECH_START_WAIT_S 안에 시작하지 않으면 그만. 반환: (int16 프레임 목록, 말소리 초)."""
        for _ in range(math.ceil(BEEP_SKIP_S / FRAME_S - 1e-9)):
            mic.read()
        frames, speech, quiet, started = [], 0, 0, False
        for i in range(int(MAX_RECORD_S / FRAME_S)):
            f = mic.read()
            frames.append(f)
            if self.vad(f) >= VAD_SPEECH:
                speech, quiet, started = speech + 1, 0, True
            elif started:
                quiet += 1
                if quiet * FRAME_S >= END_SILENCE_S:
                    break
            elif (i + 1) * FRAME_S >= SPEECH_START_WAIT_S:
                break
        return frames, speech * FRAME_S

    def _beep(self, hz):
        """'삑' — 실패해도 듣기는 계속(소리 장치가 없을 수 있다)."""
        try:
            self.beep(hz)
        except Exception as e:
            self.log(f'[음성] 소리 못 냄({type(e).__name__}) — 듣기는 계속')

    def _none(self, why, t0):
        """보낼 것 없음 — 로그만."""
        self.log(f'[음성] {why} — 보낸 것 없음 ({time.monotonic() - t0:.1f}초)')
        return {'kind': 'none', 'text': '', 'why': why, 'times': {}}


def to_wav(frames):
    """int16 프레임 목록 → WAV 바이트(16 kHz mono). 바깥 영향 없음."""
    import io
    import wave
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(b''.join(f.astype('<i2').tobytes() for f in frames))
    return buf.getvalue()


# ---------------- 실제 장치 · 서비스(시험에서는 가짜를 넣는다) ----------------
class Transcriber:
    """OpenAI 받아 적기 — STT_MODELS 순서로 해 보고 처음 된 것. 입력 WAV 바이트, 출력 (text, confidence, model).
    실패: 모든 모델이 실패하면 마지막 예외를 그대로 올린다(VoiceListener 가 로그 · 아무것도 안 보냄)."""

    def __init__(self, models=STT_MODELS, client=None):
        """client 는 시험 때 가짜. 비우면 첫 받아 적기 때 OpenAI()(OPENAI_API_KEY) — 키가 없어도 시동어 · 녹음 시험은 되게."""
        self.client, self.models = client, models

    def __call__(self, wav_bytes):
        """WAV → 글. gpt-4o 계열은 문장 힌트 + logprobs(확신도), whisper-1 은 힌트만. 키가 없으면 OpenAIError(→ 로그 · 안 보냄)."""
        if self.client is None:
            from openai import OpenAI
            self.client = OpenAI(timeout=STT_TIMEOUT_S, max_retries=1)
        last = None
        for model in self.models:
            try:
                kw = {'model': model, 'file': ('voice.wav', wav_bytes, 'audio/wav'), 'language': 'ko', 'prompt': HINT}
                if model.startswith('gpt-4o'):
                    kw.update(response_format='json', include=['logprobs'])
                r = self.client.audio.transcriptions.create(**kw)
                return r.text, confidence_from_logprobs(getattr(r, 'logprobs', None)), model
            except Exception as e:
                if type(e).__name__ in NO_FALLBACK:      # 키 · 권한 · 크레딧 문제는 다른 모델로도 안 된다
                    raise
                last = e
        raise last


class Mic:
    """마이크 입력 — 누를 때만 연다(sounddevice 블로킹 읽기, 강의 wakeup_word 와 같은 16 kHz · int16 · 1280)."""

    def __init__(self, device=None):
        """device = sounddevice 입력 장치 번호(비우면 기본)."""
        import sounddevice as sd
        self.stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16', blocksize=FRAME, device=device)
        self.stream.start()

    def read(self):
        """80 ms 를 읽어 1차원 int16 배열로."""
        data, _ = self.stream.read(FRAME)
        return data.flatten()

    def close(self):
        """멈추고 닫는다(여러 번 불러도 됨)."""
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None


class Wake:
    """openWakeWord 시동어 점수 — VAD_GATE(말소리일 때만 인정)를 같이 켠다."""

    def __init__(self, model_path):
        """model_path = hello_rokey_8332_32.tflite. 못 읽으면 예외(--check 가 알려 줌)."""
        from openwakeword.model import Model
        self.model = Model(wakeword_models=[str(model_path)], vad_threshold=VAD_GATE)
        self.name = Path(model_path).stem

    def score(self, frame):
        """80 ms → 시동어 점수 0 ~ 1."""
        return float(self.model.predict(frame)[self.name])


class Vad:
    """Silero VAD(openWakeWord 에 들어 있음) — 80 ms 를 640 샘플 둘로 나눠 평균 점수."""

    def __init__(self):
        """모델을 한 번 읽는다."""
        from openwakeword.vad import VAD
        self.vad = VAD()

    def __call__(self, frame):
        """80 ms → 말소리 점수 0 ~ 1."""
        return float(self.vad.predict(frame, frame_size=640))

    def reset(self):
        """누를 때마다 상태(LSTM h · c)를 지운다."""
        self.vad.reset_states()


def beep(hz, seconds=BEEP_S):
    """사인파 '삑'(끝을 살짝 줄여 딸깍 소리 없게). 기다리지 않고 바로 돌아온다. 소리 장치가 없으면 예외."""
    import numpy as np
    import sounddevice as sd
    t = np.arange(int(SAMPLE_RATE * seconds)) / SAMPLE_RATE
    wave_ = 0.3 * np.sin(2 * np.pi * hz * t) * np.minimum(1.0, (seconds - t) / 0.02)
    sd.play(wave_.astype('float32'), SAMPLE_RATE)


class MqttLink:
    """브로커 연결 — d2/hmi/voice/listen 을 받아 큐에 넣고, intent · 정지 요청을 보낸다. 정지 응답은 로그만."""

    def __init__(self, host, port, log=print):
        """연결 · 구독. 실패하면 paho 가 다시 붙는다."""
        import paho.mqtt.client as mqtt
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id='d2_voice')
        except AttributeError:
            self.client = mqtt.Client(client_id='d2_voice')
        self.presses, self.log, self.sent = queue.Queue(), log, set()   # sent = 내가 보낸 정지 요청 req_id
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect_async(host, int(port), keepalive=5)
        self.client.loop_start()

    def _on_connect(self, client, _u, _f, rc):
        """붙을 때마다 다시 구독(브로커 재시작 뒤에도)."""
        self.log(f'[음성] 브로커 연결 rc={rc}')
        if rc == 0:
            client.subscribe([(LISTEN_TOPIC, 1), (res_topic(STOP_SERVICE), 1)])

    def _on_message(self, _c, _u, msg):
        """누름은 큐로, 정지 응답은 로그로."""
        if msg.topic == LISTEN_TOPIC:
            self.presses.put(time.monotonic())
            return
        try:
            body = json.loads(msg.payload)
        except ValueError:
            return
        if isinstance(body, dict) and body.get('req_id') in self.sent:   # 화면 [정지] 응답은 안 찍는다
            self.sent.discard(body['req_id'])
            self.log(f'[음성] 정지 응답: success={body.get("success")} {body.get("message", "")}')

    def publish(self, topic, payload):
        """QoS 1 · retained 아님(IRD 10.1 — intent · 요청). 정지 요청이면 req_id 를 기억해 그 응답만 로그에 남긴다."""
        if topic == req_topic(STOP_SERVICE):
            self.sent.add(json.loads(payload)['req_id'])
        self.client.publish(topic, payload, qos=1)


def check(args):
    """준비 점검 — 항목마다 ✅ / ❌(할 일). 받아 적기는 부르지 않는다(키 칸만 본다). 종료 코드 0 = 다 됨."""
    ok = True

    def row(good, name, detail):
        nonlocal ok
        ok &= good
        print(f'{"✅" if good else "❌"} {name} — {detail}')
    try:
        import sounddevice as sd
        row(True, '마이크', sd.query_devices(args.mic if args.mic is not None else sd.default.device[0])['name'])
        row(True, '스피커', sd.query_devices(sd.default.device[1])['name'] + ' (없어도 듣기는 됨)')
    except Exception as e:
        row(False, '소리 장치', f'{type(e).__name__}: {e} → USB 마이크 연결 · sounddevice')
    try:
        Wake(args.wakeword)
        row(True, '시동어 모델', str(args.wakeword))
    except Exception as e:
        row(False, '시동어 모델', f'{args.wakeword}: {type(e).__name__} → --wakeword 또는 D2_WAKEWORD_MODEL')
    row(bool(os.environ.get('OPENAI_API_KEY')), 'OpenAI 키', '있음' if os.environ.get('OPENAI_API_KEY') else 'web/.env OPENAI_API_KEY 비어 있음')
    try:
        import socket
        socket.create_connection((args.host, int(args.port)), timeout=2).close()
        row(True, '브로커', f'{args.host}:{args.port}')
    except OSError as e:
        row(False, '브로커', f'{args.host}:{args.port} {e} → MQTT_HOST(웹 PC 는 D2_WEB_IPV4) · compose mosquitto')
    print('준비 끝' if ok else '❌ 를 고친 뒤 다시 --check')
    return 0 if ok else 1


def main():
    """설정을 읽고 누름을 기다리며 한 번씩 듣는다. 듣는 동안 온 누름은 버린다(한 번에 하나). Ctrl+C 로 끝."""
    load_env_file(ROOT / 'web' / '.env')
    p = argparse.ArgumentParser(description='D2 음성(웹 PC 호스트) — 마이크 버튼 → 시동어 → 받아 적기 → MQTT')
    p.add_argument('--wakeword', default=os.environ.get('D2_WAKEWORD_MODEL') or str(Path(__file__).parent / 'models' / 'hello_rokey_8332_32.tflite'))
    p.add_argument('--threshold', type=float, default=WAKE_THRESHOLD)
    p.add_argument('--mic', type=int, default=None)
    p.add_argument('--host', default=os.environ.get('MQTT_HOST') or '127.0.0.1')
    p.add_argument('--port', default=os.environ.get('MQTT_PORT') or '1883')
    p.add_argument('--check', action='store_true')
    args = p.parse_args()
    if args.check:
        return check(args)
    models = tuple(dict.fromkeys([os.environ.get('D2_STT_MODEL') or STT_MODELS[0], *STT_MODELS]))
    try:
        Wake(args.wakeword)                          # 첫 누름에서 죽지 않게 시작할 때 한 번 읽어 본다
    except Exception as e:
        print(f'[음성] 시동어 모델을 못 읽음: {args.wakeword} ({type(e).__name__}: {e}) — --wakeword 또는 D2_WAKEWORD_MODEL, --check')
        return 2
    vad, link = Vad(), MqttLink(args.host, args.port)
    listener = VoiceListener(lambda: Mic(args.mic), lambda: Wake(args.wakeword), vad, Transcriber(models), link.publish, beep,
                             threshold=args.threshold)
    print(f'[음성] 준비 — 시동어 {Path(args.wakeword).name} 문턱 {args.threshold} · 받아 적기 {models} · 브로커 {args.host}:{args.port}')
    try:
        while True:
            link.presses.get()
            try:
                listener.listen_once()
            except Exception as e:                   # 마이크가 빠지거나 바빠도 프로그램은 살아서 다음 누름을 받는다
                print(f'[음성] 듣기 실패({type(e).__name__}: {e}) — 다음 누름을 기다림')
            while not link.presses.empty():          # 듣는 동안 눌린 것은 버린다 — 옛 누름으로 다시 녹음하지 않게
                link.presses.get_nowait()
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
