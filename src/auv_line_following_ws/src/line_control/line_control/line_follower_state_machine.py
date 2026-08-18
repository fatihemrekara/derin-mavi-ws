"""LineFollower görev durum makinesi.

Durumlar:
    SEARCHING   -> Henüz çizgi bulunamadı (başlangıç durumu)
    TRACKING    -> Çizgi takip ediliyor (normal çalışma)
    LINE_LOST   -> Çizgi az önce kaybedildi, arama manevrası yapılıyor
    HOLD        -> Uzun süreli kayıp; güvenlik için araç sabit tutuluyor

ROS'a bağımlılığı yoktur -> birim testi kolaydır
(bkz. test/test_state_machine.py).
"""

from enum import Enum, auto


class LineFollowerState(Enum):
    SEARCHING = auto()
    TRACKING = auto()
    LINE_LOST = auto()
    HOLD = auto()


class LineFollowerStateMachine:
    def __init__(self, hold_timeout_sec: float = 8.0):
        self.state = LineFollowerState.SEARCHING
        self.hold_timeout_sec = hold_timeout_sec
        self._lost_since = None

    def update(self, is_line_lost: bool, now: float) -> LineFollowerState:
        if not is_line_lost:
            self.state = LineFollowerState.TRACKING
            self._lost_since = None
            return self.state

        if self.state == LineFollowerState.TRACKING:
            self._lost_since = now
            self.state = LineFollowerState.LINE_LOST
            return self.state

        if self.state in (LineFollowerState.LINE_LOST, LineFollowerState.HOLD):
            if self._lost_since is not None and (now - self._lost_since) > self.hold_timeout_sec:
                self.state = LineFollowerState.HOLD
            return self.state

        # SEARCHING durumunda çizgi hâlâ yok -> SEARCHING'de kal
        return self.state
