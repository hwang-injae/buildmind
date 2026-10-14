# -*- coding: utf-8 -*-
"""voice.py(웹 PC 음성, E-105) — 마이크 · 시동어 모델 · OpenAI · 브로커 없이 가짜 부품으로 듣기 순서와 보낼 MQTT 내용을 본다."""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_bridge'):
    sys.path.insert(0, str(p))

import voice  # noqa: E402
from voice import (FRAME, FRAME_S, INTENT_TOPIC, Transcriber, VoiceListener, confidence_from_logprobs,  # noqa: E402
                   is_hallucination, is_stop, load_env_file, to_wav)

SPEECH, QUIET = 0.9, 0.05


class FakeMic:
    """read() 마다 0 프레임을 주고 몇 번 읽었는지 센다."""

    def __init__(self):
        self.reads, self.closed = 0, False

    def read(self):
        self.reads += 1
        return np.zeros(FRAME, np.int16)

    def close(self):
        self.closed = True


class FakeWake:
    """wake_at 번째 프레임에서 점수 1.0(그 전엔 0). None 이면 끝까지 0."""

    def __init__(self, wake_at):
        self.n, self.wake_at = 0, wake_at

    def score(self, _frame):
        self.n += 1
        return 1.0 if self.wake_at is not None and self.n >= self.wake_at else 0.0


def vad_seq(speech_frames, quiet_frames=40, lead_quiet=2):
    """녹음 구간 VAD 점수: 앞 조용 → 말 → 뒤 조용(끝은 계속 조용)."""
    seq = [QUIET] * lead_quiet + [SPEECH] * speech_frames + [QUIET] * quiet_frames
    it = iter(seq)
    return lambda _f: next(it, QUIET)


def make(wake_at=3, speech_frames=15, stt=None, beep_fail=False, **kw):
    """가짜 부품으로 VoiceListener + 기록."""
    rec = SimpleNamespace(published=[], beeps=[], logs=[], stt_calls=0, mic=FakeMic())

    def fake_stt(wav):
        rec.stt_calls += 1
        rec.wav = wav
        return stt(wav) if callable(stt) else (stt if stt is not None else ('의자 다리를 높게 만들어 줘', 0.91, 'gpt-4o-mini-transcribe'))

    def fake_beep(hz):
        if beep_fail:
            raise OSError('소리 장치 없음')
        rec.beeps.append(hz)
    vl = VoiceListener(lambda: rec.mic, lambda: FakeWake(wake_at), vad_seq(speech_frames, **kw), fake_stt,
                       lambda t, p: rec.published.append((t, p)), fake_beep, log=rec.logs.append)
    return vl, rec


def test_요청은_intent_request_design_으로_보낸다():
    vl, rec = make()
    out = vl.listen_once()
    assert out['kind'] == 'intent' and rec.stt_calls == 1 and rec.mic.closed
    (topic, payload), = rec.published
    body = json.loads(payload)
    assert topic == INTENT_TOPIC
    assert body['schema'] == 'intent/1' and body['intent'] == 'request_design' and body['text'] == '의자 다리를 높게 만들어 줘'
    assert body['confidence'] == 0.91 and body['design_id'] is None and body['mode'] is None
    assert rec.beeps == [voice.BEEP_HIGH_HZ, voice.BEEP_LOW_HZ]          # 시동어 들음 → 녹음 끝


def test_멈춰는_정지_요청_source_voice():
    vl, rec = make(stt=('멈춰!', 0.8, 'gpt-4o-mini-transcribe'))
    assert vl.listen_once()['kind'] == 'stop'
    (topic, payload), = rec.published
    body = json.loads(payload)
    assert topic == 'd2/safety/stop/req' and body['source'] == 'voice' and body['reason'] == '' and body['req_id']


def test_시동어가_없으면_마이크만_닫고_아무것도_안_보낸다():
    vl, rec = make(wake_at=None)
    out = vl.listen_once()
    assert out['kind'] == 'none' and '시동어' in out['why']
    assert rec.published == [] and rec.stt_calls == 0 and rec.beeps == [] and rec.mic.closed
    assert rec.mic.reads == int(voice.WAKE_WAIT_S / FRAME_S)


def test_시동어_뒤_말이_없으면_받아_적기를_안_부른다():
    vl, rec = make(speech_frames=0)
    out = vl.listen_once()
    assert out['kind'] == 'none' and rec.stt_calls == 0 and rec.published == []
    # 시동어 3 + '삑' 건너뛰기 + 말 시작 기다림(3초)까지만 읽고 그만
    assert rec.mic.reads <= 3 + math.ceil(voice.BEEP_SKIP_S / FRAME_S) + int(voice.SPEECH_START_WAIT_S / FRAME_S) + 1


def test_말이_끊기면_녹음이_끝난다():
    vl, rec = make(speech_frames=10)
    vl.listen_once()
    skip = math.ceil(voice.BEEP_SKIP_S / FRAME_S - 1e-9)
    end_quiet = int(np.ceil(voice.END_SILENCE_S / FRAME_S - 1e-9))
    assert rec.mic.reads == 3 + skip + 2 + 10 + end_quiet                  # 최대 8초까지 가지 않는다


def test_너무_짧은_소리는_버린다():
    vl, rec = make(speech_frames=2)                                        # 0.16 초 < MIN_SPEECH_S
    assert vl.listen_once()['kind'] == 'none' and rec.stt_calls == 0


@pytest.mark.parametrize('text', ['시청해 주셔서 감사합니다.', '구독과 좋아요', '  ', ''])
def test_무음_환각_글은_안_보낸다(text):
    vl, rec = make(stt=(text, None, 'whisper-1'))
    assert vl.listen_once()['kind'] == 'none' and rec.published == []


def test_받아_적기_실패면_안_보내고_로그():
    def boom(_):
        raise RuntimeError('insufficient_quota')
    vl, rec = make(stt=boom)
    out = vl.listen_once()
    assert out['kind'] == 'none' and 'insufficient_quota' in out['why'] and rec.published == []


def test_소리_장치가_없어도_듣기는_계속():
    vl, rec = make(beep_fail=True)
    assert vl.listen_once()['kind'] == 'intent' and any('소리 못 냄' in m for m in rec.logs)


def test_녹음은_16kHz_wav_로_넘긴다():
    vl, rec = make(speech_frames=5)
    vl.listen_once()
    assert rec.wav[:4] == b'RIFF' and rec.wav[24:28] == (16000).to_bytes(4, 'little')


@pytest.mark.parametrize('text, stop', [
    ('멈춰', True), ('멈춰라!', True), ('정지.', True), ('스톱', True), ('그만', True), ('Stop', True),
    ('멈춰요', True), ('정지해 주세요', True), ('멈춰 멈춰', True),
    ('멈춰 있는 다리를 더 높게 해 줘', False), ('의자 만들어 줘', False), ('', False),
    ('그만큼 높게', False), ('정지해 있는 의자', False)])
def test_멈춰_판단은_짧은_정지_말만(text, stop):
    assert is_stop(text) is stop


def test_요청_속_감사합니다는_살린다():
    assert is_hallucination('감사합니다') and not is_hallucination('의자 만들어 줘 감사합니다')


def test_힌트를_되읽은_글과_자막_꼬리글은_버리고_짧은_낱말은_살린다():
    assert is_hallucination('젠가 블록으로 만드는 가구 설계 요청이다.') and is_hallucination('자막 제공: 아무개')
    assert not is_hallucination('의자') and not is_hallucination('책상 넓게')
    assert '멈춰' not in voice.HINT and '정지' not in voice.HINT        # 힌트 끝을 되읽어 잘못된 정지가 나가지 않게


def test_누를_때마다_시동어_모델을_마이크보다_먼저_만들고_VAD_를_지운다():
    order = []
    vad = vad_seq(10)

    class ResetVad:
        def __call__(self, f):
            return vad(f)

        def reset(self):
            order.append('vad_reset')
    mic = FakeMic()

    def new_wake():
        order.append('wake')
        return FakeWake(3)

    def open_mic():
        order.append('mic')
        return mic
    VoiceListener(open_mic, new_wake, ResetVad(), lambda w: ('의자', None, 'm'), lambda t, p: None, lambda hz: None,
                  log=lambda m: None).listen_once()
    assert order == ['wake', 'vad_reset', 'mic']


def test_확신도는_logprob_평균의_exp():
    assert confidence_from_logprobs([{'logprob': 0.0}, {'logprob': 0.0}]) == 1.0
    assert confidence_from_logprobs([SimpleNamespace(logprob=-0.6931)]) == 0.5
    assert confidence_from_logprobs(None) is None and confidence_from_logprobs([{'logprob': float('nan')}]) is None


def test_받아_적기는_앞_모델이_실패하면_whisper_1_한_번():
    calls = []

    def create(**kw):
        calls.append(kw)
        if kw['model'] != 'whisper-1':
            raise RuntimeError('model_not_found')
        return SimpleNamespace(text='책상 넓게')
    client = SimpleNamespace(audio=SimpleNamespace(transcriptions=SimpleNamespace(create=create)))
    text, conf, model = Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))
    assert (text, conf, model) == ('책상 넓게', None, 'whisper-1')
    assert [c['model'] for c in calls] == ['gpt-4o-mini-transcribe', 'whisper-1']
    assert calls[0]['include'] == ['logprobs'] and 'include' not in calls[1]
    assert all(c['language'] == 'ko' and '의자' in c['prompt'] for c in calls)


def test_키_권한_크레딧_오류는_다른_모델로_넘기지_않는다():
    class RateLimitError(Exception):
        pass
    calls = []

    def create(**kw):
        calls.append(kw['model'])
        raise RateLimitError('insufficient_quota')
    client = SimpleNamespace(audio=SimpleNamespace(transcriptions=SimpleNamespace(create=create)))
    with pytest.raises(RateLimitError):
        Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))
    assert calls == ['gpt-4o-mini-transcribe']


def test_env_파일은_빈_값과_이미_있는_값을_덮지_않는다(tmp_path, monkeypatch):
    f = tmp_path / '.env'
    f.write_text('# 주석\nD2_T1=a\nD2_T2=\nD2_T3=c\n', encoding='utf-8')
    monkeypatch.delenv('D2_T1', raising=False)
    monkeypatch.delenv('D2_T2', raising=False)
    monkeypatch.setenv('D2_T3', 'keep')
    load_env_file(f)
    import os
    assert os.environ['D2_T1'] == 'a' and 'D2_T2' not in os.environ and os.environ['D2_T3'] == 'keep'
    load_env_file(tmp_path / 'none')                                       # 없는 파일은 그냥 지나감


class PermissionDeniedError(Exception):
    """openai.PermissionDeniedError 와 같은 이름 · code 칸을 가진 가짜(실제 API 호출 없이)."""

    def __init__(self, msg, code=None):
        super().__init__(msg)
        self.code = code


def fake_client(script):
    """script[모델] = 돌려줄 글 또는 던질 예외. 부른 모델 순서를 calls 에 남긴다."""
    calls = []

    def create(**kw):
        calls.append(kw['model'])
        out = script[kw['model']]
        if isinstance(out, Exception):
            raise out
        return SimpleNamespace(text=out)
    return SimpleNamespace(audio=SimpleNamespace(transcriptions=SimpleNamespace(create=create))), calls


def test_모델_권한_없음_403_model_not_found_는_whisper_1_로_넘긴다():
    client, calls = fake_client({'gpt-4o-mini-transcribe': PermissionDeniedError('no access to model', code='model_not_found'),
                                 'whisper-1': '다리를 한 층 더 높여줘'})
    text, conf, model = Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))
    assert (text, conf, model) == ('다리를 한 층 더 높여줘', None, 'whisper-1')
    assert calls == ['gpt-4o-mini-transcribe', 'whisper-1']


def test_code_가_body_에만_있어도_모델_권한_없음으로_본다():
    e = PermissionDeniedError('no access')
    e.body = {'code': 'model_not_found'}
    client, calls = fake_client({'gpt-4o-mini-transcribe': e, 'whisper-1': '책상'})
    assert Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))[2] == 'whisper-1'


def test_일반_권한_오류는_다른_모델로_넘기지_않는다():
    client, calls = fake_client({'gpt-4o-mini-transcribe': PermissionDeniedError('project is disabled', code='permission_denied'),
                                 'whisper-1': '안 불려야 함'})
    with pytest.raises(PermissionDeniedError):
        Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))
    assert calls == ['gpt-4o-mini-transcribe']


def test_두_모델_다_실패하면_마지막_오류에_앞_모델_까닭이_붙고_로그에_둘_다_남는다():
    first = PermissionDeniedError('no access to model', code='model_not_found')
    client, calls = fake_client({'gpt-4o-mini-transcribe': first, 'whisper-1': ConnectionError('인터넷 끊김')})
    tr = Transcriber(client=client)
    with pytest.raises(ConnectionError) as info:
        tr(to_wav([np.zeros(FRAME, np.int16)]))
    assert info.value.__cause__ is first and calls == ['gpt-4o-mini-transcribe', 'whisper-1']
    vl, rec = make(stt=tr)                                       # 듣기 흐름에서도: 아무것도 안 보내고 두 까닭을 로그로
    out = vl.listen_once()
    assert out['kind'] == 'none' and rec.published == []
    assert 'ConnectionError: 인터넷 끊김' in out['why'] and '앞 모델: PermissionDeniedError: no access to model' in out['why']


def test_모델_권한_없음_뒤_whisper_1_이_키_오류면_그_오류를_올리고_앞_까닭을_붙인다():
    class AuthenticationError(Exception):
        pass
    first = PermissionDeniedError('no access to model', code='model_not_found')
    client, calls = fake_client({'gpt-4o-mini-transcribe': first, 'whisper-1': AuthenticationError('invalid key')})
    with pytest.raises(AuthenticationError) as info:
        Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))
    assert info.value.__cause__ is first


@pytest.mark.parametrize('body', [{'error': 'model_not_found'}, {'error': None}, 'model_not_found', None, {'code': 3}])
def test_body_형식이_예상_밖이면_code_없음으로_넘기지_않는다(body):
    e = PermissionDeniedError('no access')
    e.body = body
    assert voice.error_code(e) is None
    client, calls = fake_client({'gpt-4o-mini-transcribe': e, 'whisper-1': '안 불려야 함'})
    with pytest.raises(PermissionDeniedError):
        Transcriber(client=client)(to_wav([np.zeros(FRAME, np.int16)]))
    assert calls == ['gpt-4o-mini-transcribe']


def test_body_error_dict_의_code_를_읽는다():
    e = PermissionDeniedError('no access')
    e.body = {'error': {'code': 'model_not_found'}}
    assert voice.error_code(e) == 'model_not_found'
