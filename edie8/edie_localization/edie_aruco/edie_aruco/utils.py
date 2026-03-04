import math
import time
from collections import deque
import numpy as np

# utils.py (추가)
class Utils:
    # -------------------- One Euro Filter -------------------- #
    class OneEuro:
        """
        실시간 저지연 스무딩 (Casiez et al., 2012)
        사용:
            oe = Utils.OneEuro(freq=30, min_cutoff=1.2, beta=0.004)
            x̂  = oe.filt(x_raw, timestamp)
        """
        def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
            self.freq, self.min_cutoff, self.beta, self.d_cutoff = \
                float(freq), float(min_cutoff), float(beta), float(d_cutoff)
            self.last_t = None
            self.x_prev = None
            self.dx_prev = 0.0

        def _alpha(self, cutoff):
            te  = 1.0 / self.freq
            tau = 1.0 / (2.0 * math.pi * cutoff)
            return 1.0 / (1.0 + tau / te)

        def filt(self, x, t=None):
            if t is None: t = time.time()
            if self.last_t is None:             # 첫 샘플
                self.last_t, self.x_prev = t, x
                return x

            # 실측 주파수 업데이트
            dt = max(t - self.last_t, 1e-6)
            self.freq = 1.0 / dt
            self.last_t = t

            # 1) 속도 추정 + 저역통과
            dx       = (x - self.x_prev) * self.freq
            a_d      = self._alpha(self.d_cutoff)
            dx_hat   = a_d * dx + (1.0 - a_d) * self.dx_prev

            # 2) 적응 cutoff
            cutoff   = self.min_cutoff + self.beta * abs(dx_hat)

            # 3) 신호 필터
            a        = self._alpha(cutoff)
            x_hat    = a * x + (1.0 - a) * self.x_prev

            self.x_prev, self.dx_prev = x_hat, dx_hat
            return x_hat

    # -------------------- Median Filter -------------------- #
    class MedianFilter:
        """
        창 크기 N 프레임의 중앙값 필터.
        사용:
            mf  = Utils.MedianFilter(window=5)
            x̂   = mf.filt(x_raw)   # 중앙값
        """
        def __init__(self, window=11):
            self.window = int(window)
            self.buf = deque(maxlen=self.window)

        def filt(self, x):
            self.buf.append(x)
            return float(np.median(list(self.buf)))

    # -------------------- Low Pass (EMA) Filter -------------------- #
    class LowPass:
        """
        1차 저역통과(지수이동평균) 필터.
        두 가지 방식 지원:
          1) cutoff(Hz) 기반 동적 α (권장)  -> 샘플 dt에 따라 α 자동 조정
          2) 고정 alpha(0~1)               -> 간단/예측가능

        사용:
            # 동적 α (cutoff와 초기 freq 지정)
            lp = Utils.LowPass(cutoff=3.0, freq=30.0)
            y  = lp.filt(x, timestamp)

            # 고정 α
            lp = Utils.LowPass(alpha=0.2)       # 0.2면 많이 부드럽고 지연↑
            y  = lp.filt(x)                     # timestamp 생략 가능
        """
        def __init__(self, cutoff: float = None, freq: float = 30.0, alpha: float = None):
            # alpha가 주어지면 고정-알파 모드, 아니면 cutoff 기반 동적 모드
            self.use_fixed_alpha = (alpha is not None)
            self.alpha_fixed = float(alpha) if alpha is not None else None

            self.cutoff = float(cutoff) if cutoff is not None else 3.0
            self.freq   = float(freq)
            self.last_t = None
            self.y_prev = None

        def reset(self, y0=None):
            """내부 상태 초기화 (필요시 시작값 지정)"""
            self.last_t = None
            self.y_prev = y0

        def _alpha_dynamic(self):
            # 실측 주파수 기반으로 α 계산
            te  = 1.0 / max(self.freq, 1e-6)
            tau = 1.0 / (2.0 * math.pi * max(self.cutoff, 1e-6))
            return 1.0 / (1.0 + (tau / te))

        def filt(self, x, t=None):
            if t is None:
                t = time.time()

            # 첫 샘플 처리
            if self.last_t is None or self.y_prev is None:
                self.last_t = t
                self.y_prev = x
                return x

            # 실측 주파수 갱신
            dt = max(t - self.last_t, 1e-6)
            self.freq = 1.0 / dt
            self.last_t = t

            # 알파 결정
            a = self.alpha_fixed if self.use_fixed_alpha else self._alpha_dynamic()
            a = float(np.clip(a, 0.0, 1.0))

            # EMA
            y = a * x + (1.0 - a) * self.y_prev
            self.y_prev = y
            return y
