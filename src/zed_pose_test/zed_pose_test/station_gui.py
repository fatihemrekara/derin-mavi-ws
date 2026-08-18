import sys
import time
import threading
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Joy, CompressedImage
from mavros_msgs.msg import VfrHud, State
from std_msgs.msg import String
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout,
                             QHBoxLayout, QWidget, QFrame, QGraphicsDropShadowEffect,
                             QSizePolicy)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette, QImage, QPixmap, QFontDatabase


# ==========================================
# AYAR: Mini ROV kamerasi
# Pi 5 uzerindeki yayincinin topic'i ile AYNI olmali.
# ==========================================
MINI_CAM_TOPIC = '/mini_rov/camera/image_raw/compressed'
MINI_CAM_UI_FPS = 15      # arayuzde cozulecek maksimum kare/sn (None = sinirsiz)


# ==========================================
# 0. TEK KAMERA AKISI: ham JPEG -> QImage (ayri thread, frame-drop'lu)
# ==========================================
class JpegStream:
    """
    Bir CompressedImage topic'i icin decode isci si.

    ROS callback'inde SIFIR is yapilir: sadece en son JPEG saklanir.
    Islenmemis onceki kare ustune yazilir -> otomatik frame drop.
    Bu sayede kamera hizli gelse bile GUI ve spin thread'i kasmaz.
    """

    def __init__(self, name, signal, logger, max_fps=None):
        self.name = name
        self.signal = signal
        self.logger = logger
        # Mini kamera icin FPS sinirlamak istersen (orn. 15) CPU daha da duser.
        self.min_period = (1.0 / max_fps) if max_fps else 0.0
        self._last_emit = 0.0

        self._latest = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def push(self, msg):
        with self._lock:
            self._latest = msg.data
        self._event.set()

    def _loop(self):
        while self._running:
            if not self._event.wait(timeout=0.5):
                continue
            self._event.clear()

            with self._lock:
                data = self._latest
                self._latest = None
            if data is None:
                continue

            # Istege bagli FPS kisitlama (fazla kareyi hic decode etme)
            if self.min_period:
                now = time.monotonic()
                if now - self._last_emit < self.min_period:
                    continue
                self._last_emit = now

            try:
                np_arr = np.frombuffer(data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR
                if cv_image is None:
                    continue
                h, w, ch = cv_image.shape
                # cvtColor YOK: Qt'ye dogrudan BGR888 veriyoruz
                qt_img = QImage(cv_image.data, w, h, ch * w,
                                QImage.Format_BGR888).copy()
                self.signal.emit(qt_img)
            except Exception as e:
                self.logger.error(f"{self.name} goruntu cozme hatasi: {e}")

    def stop(self):
        self._running = False
        self._event.set()


# ==========================================
# 1. ROS 2 ARKA PLAN ISCISI (THREAD)  --  MANTIK DEGISMEDI
# ==========================================
class RosThread(QThread):
    # is_main_active, mode, is_armed
    update_signal = pyqtSignal(bool, str, bool)
    telemetry_signal = pyqtSignal(float, int)
    image_main_signal = pyqtSignal(QImage)
    image_mini_signal = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = rclpy.create_node('gui_listener_node')

        self.sub_hud = self.node.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.hud_callback, 10)

        # GERCEK arac durumu (arm/mode)
        self.sub_state = self.node.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_active = self.node.create_subscription(
            String, '/active_vehicle', self.active_callback, latched_qos)

        # MINI ROV durumu. Format: "ARMED;HAZIR" / "DISARMED;JOY YOK" ...
        # Mini aktifken ARM karti ANA aracin degil MINI'nin durumunu
        # gostersin diye. Aksi halde operator ekranda "DISARMED" gorup
        # mini araci arm sanip suya sokabilir.
        self.sub_mini = self.node.create_subscription(
            String, '/mini_rov/status', self.mini_status_callback, latched_qos)
        self.mini_armed = False
        self.mini_text = "BAGLANTI YOK"
        self.mini_last = 0.0

        # Kamera QoS: BEST_EFFORT + depth=1 -> eski kareler birikmez, hep en yeni gelir.
        # (Kamera yayincisi da BEST_EFFORT oldugu icin bu eslesme dogru olandir.)
        cam_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_cam_main = self.node.create_subscription(
            CompressedImage,
            '/zed/zed_node/rgb/color/raw/image/compressed',
            self.cam_main_callback,
            cam_qos)

        # MINI ROV kamerasi (Raspberry Pi 5 uzerindeki OBSBOT Meet SE)
        self.sub_cam_mini = self.node.create_subscription(
            CompressedImage,
            MINI_CAM_TOPIC,
            self.cam_mini_callback,
            cam_qos)

        self.depth = 0.0
        self.heading = 0
        self.is_main_active = True
        self.current_mode = "MANUAL"
        self.is_armed = False

        # --- Kamera decode'u AYRI thread'lerde: spin thread'ini bloke etmez ---
        log = self.node.get_logger()
        self.stream_main = JpegStream("ZED2", self.image_main_signal, log)
        # Mini kamera kucuk panelde gosteriliyor: 15 FPS fazlasiyla yeterli,
        # gereksiz decode yaparak Pi/istasyon CPU'sunu yakmayalim.
        self.stream_mini = JpegStream("MINI CAM", self.image_mini_signal, log,
                                      max_fps=MINI_CAM_UI_FPS)

        self.timer = self.node.create_timer(0.5, self.publish_telemetry)

    def cam_main_callback(self, msg):
        self.stream_main.push(msg)

    def cam_mini_callback(self, msg):
        self.stream_mini.push(msg)

    def hud_callback(self, msg):
        self.depth = abs(msg.altitude)
        self.heading = msg.heading

    def state_callback(self, msg):
        self.is_armed = msg.armed
        mode = msg.mode if msg.mode else "?"
        self.current_mode = mode.replace("_", " ")
        self._emit()

    def active_callback(self, msg):
        self.is_main_active = (msg.data == "MAIN")
        self._emit()

    def mini_status_callback(self, msg):
        parts = msg.data.split(';')
        self.mini_armed = (parts[0].strip() == "ARMED")
        self.mini_text = parts[1].strip() if len(parts) > 1 else parts[0].strip()
        self.mini_last = time.monotonic()
        self._emit()

    def _emit(self):
        """
        Hangi arac aktifse ARM/durum bilgisi ONUN olsun.
        Mini aktifken /mavros/state'i gostermek yaniltici olurdu: o,
        pasif duran ANA aracin durumudur.
        """
        if self.is_main_active:
            self.update_signal.emit(True, self.current_mode, self.is_armed)
        elif time.monotonic() - self.mini_last > 3.0:
            # Pi'den 3 saniyedir haber yok: ekranda "her sey yolunda"
            # goruntusu birakma.
            self.update_signal.emit(False, "BAGLANTI YOK", False)
        else:
            self.update_signal.emit(False, self.mini_text, self.mini_armed)

    def publish_telemetry(self):
        self.telemetry_signal.emit(self.depth, self.heading)
        if not self.is_main_active:
            self._emit()   # mini bagi koparsa 3 sn icinde ekrana yansisin

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        # Decode thread'lerini temiz durdur
        self.stream_main.stop()
        self.stream_mini.stop()
        rclpy.shutdown()


# ==========================================
# GORSEL YARDIMCI: Etiketli gosterge karti
# ==========================================
class StatCard(QFrame):
    """Ust satirda kucuk etiket, altta buyuk deger tutan cam kart."""
    def __init__(self, caption, value, mono=False):
        super().__init__()
        self.setObjectName("card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 12, 18, 14)
        lay.setSpacing(2)

        self.caption = QLabel(caption)
        self.caption.setObjectName("caption")
        self.caption.setAlignment(Qt.AlignCenter)

        self.value = QLabel(value)
        self.value.setObjectName("valueMono" if mono else "value")
        self.value.setAlignment(Qt.AlignCenter)

        lay.addWidget(self.caption)
        lay.addWidget(self.value)

    def set_value(self, text):
        self.value.setText(text)

    def set_caption(self, text):
        self.caption.setText(text)


# ==========================================
# 2. ANA ARAYUZ (GUI)  --  SADECE GORSEL
# ==========================================
class StationGUI(QMainWindow):
    # Palet: sualtı laciverti + amber (Ana) / camgobegi (Mini)
    COL_BG_TOP = "#0d1b2a"
    COL_BG_BOT = "#050a12"
    COL_MAIN = "#f0a202"      # Ana ROV - amber
    COL_MINI = "#26c6da"      # Mini ROV - camgobegi
    COL_ARM = "#ff3b3b"       # ARMED - kirmizi
    COL_DISARM = "#2ecc71"    # DISARMED - yesil
    COL_TEXT = "#e6edf3"
    COL_MUTED = "#5b6b7a"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Gorev Kontrol Istasyonu")
        self.setGeometry(100, 100, 1200, 850)

        self.is_main_active_gui = True
        self.is_armed_gui = False

        self._build_ui()
        self._apply_stylesheet()

        # --- ARMED pulse animasyonu (sadece gorsel) ---
        self._pulse_val = 0.0
        self._pulse_dir = 1
        self._pulse_timer = QTimer(self)
        self._pulse_timer.timeout.connect(self._pulse_tick)

        # ROS 2 Thread  (MANTIK AYNI)
        self.ros_thread = RosThread()
        self.ros_thread.update_signal.connect(self.update_control_ui)
        self.ros_thread.telemetry_signal.connect(self.update_telemetry_ui)
        self.ros_thread.image_main_signal.connect(self.update_main_camera_ui)
        self.ros_thread.image_mini_signal.connect(self.update_mini_camera_ui)
        self.ros_thread.start()

        self.update_control_ui(True, "MANUAL", False)

    # ---------- UI KURULUMU ----------
    def _build_ui(self):
        central = QWidget()
        central.setObjectName("root")
        self.setCentralWidget(central)

        outer = QVBoxLayout(central)
        outer.setContentsMargins(24, 22, 24, 24)
        outer.setSpacing(16)

        # Ince baslik seridi
        header = QLabel("SUALTI GOREV KONTROL")
        header.setObjectName("appTitle")
        header.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        outer.addWidget(header)

        # === UST PANEL: arac / arm / mod ===
        top = QHBoxLayout()
        top.setSpacing(16)
        self.vehicle_card = StatCard("AKTIF ARAC", "ANA ROV")
        self.arm_card = StatCard("MOTOR DURUMU", "DISARMED")
        self.mode_card = StatCard("UCUS MODU", "MANUAL")
        for c in (self.vehicle_card, self.arm_card, self.mode_card):
            self._shadow(c)
            top.addWidget(c, 1)
        outer.addLayout(top)

        # === ORTA PANEL: telemetri ===
        mid = QHBoxLayout()
        mid.setSpacing(16)
        self.depth_card = StatCard("DERINLIK", "0.0 m", mono=True)
        self.heading_card = StatCard("PUSULA", "0", mono=True)
        for c in (self.depth_card, self.heading_card):
            self._shadow(c)
            mid.addWidget(c, 1)
        outer.addLayout(mid)

        # === ALT PANEL: kameralar ===
        self.cam_layout = QHBoxLayout()
        self.cam_layout.setSpacing(16)

        self.cam_main = QLabel("ANA ROV  ·  SINYAL BEKLENIYOR")
        self.cam_main.setObjectName("camMain")
        self.cam_main.setAlignment(Qt.AlignCenter)
        # Pixmap'e gore KENDINI buyutmesin: layout ne verirse onu kullansin.
        self.cam_main.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.cam_main.setMinimumSize(320, 240)

        self.cam_mini = QLabel("MINI ROV  ·  SINYAL BEKLENIYOR")
        self.cam_mini.setObjectName("camMini")
        self.cam_mini.setAlignment(Qt.AlignCenter)
        self.cam_mini.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.cam_mini.setMinimumSize(160, 120)

        self.cam_layout.addWidget(self.cam_main, 3)
        self.cam_layout.addWidget(self.cam_mini, 1)
        outer.addLayout(self.cam_layout, 1)

    def _shadow(self, w):
        eff = QGraphicsDropShadowEffect(self)
        eff.setBlurRadius(24)
        eff.setXOffset(0)
        eff.setYOffset(6)
        eff.setColor(QColor(0, 0, 0, 140))
        w.setGraphicsEffect(eff)

    def _apply_stylesheet(self):
        self.setStyleSheet(f"""
            QWidget#root {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 {self.COL_BG_TOP}, stop:1 {self.COL_BG_BOT});
            }}
            QLabel#appTitle {{
                color: {self.COL_MUTED};
                font-family: 'Consolas','JetBrains Mono',monospace;
                font-size: 13px;
                font-weight: 700;
                letter-spacing: 6px;
                padding: 2px 4px;
            }}
            QFrame#card {{
                background-color: rgba(255,255,255,0.035);
                border: 1px solid rgba(255,255,255,0.08);
                border-radius: 14px;
            }}
            QLabel#caption {{
                color: {self.COL_MUTED};
                font-family: 'Consolas','JetBrains Mono',monospace;
                font-size: 11px;
                font-weight: 700;
                letter-spacing: 3px;
            }}
            QLabel#value {{
                color: {self.COL_TEXT};
                font-family: 'Arial';
                font-size: 26px;
                font-weight: 800;
            }}
            QLabel#valueMono {{
                color: {self.COL_TEXT};
                font-family: 'Consolas','JetBrains Mono',monospace;
                font-size: 28px;
                font-weight: 800;
                letter-spacing: 1px;
            }}
            QLabel#camMain, QLabel#camMini {{
                background-color: #0a1420;
                border-radius: 12px;
                color: {self.COL_MUTED};
                font-family: 'Consolas','JetBrains Mono',monospace;
                font-size: 13px;
                font-weight: 700;
                letter-spacing: 2px;
            }}
            QLabel#camMain {{ border: 2px solid {self.COL_MAIN}; }}
            QLabel#camMini {{ border: 2px solid rgba(255,255,255,0.10); }}
        """)

    # ---------- KAMERA (MANTIK AYNI) ----------
    def update_main_camera_ui(self, qt_img):
        w = self.cam_main.width()
        h = self.cam_main.height()
        if w < 10 or h < 10:
            return  # layout henuz boyut vermediyse atla
        pixmap = QPixmap.fromImage(qt_img)
        # Kenarlik (2px) + ic pay icin kucuk margin birak; boylece taşıp titremez
        scaled = pixmap.scaled(w - 8, h - 8,
                               Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.cam_main.setPixmap(scaled)

    def update_mini_camera_ui(self, qt_img):
        w = self.cam_mini.width()
        h = self.cam_mini.height()
        if w < 10 or h < 10:
            return
        pixmap = QPixmap.fromImage(qt_img)
        scaled = pixmap.scaled(w - 8, h - 8,
                               Qt.KeepAspectRatio, Qt.FastTransformation)
        self.cam_mini.setPixmap(scaled)

    # ---------- TELEMETRI (MANTIK AYNI) ----------
    def update_telemetry_ui(self, depth, heading):
        if not self.is_main_active_gui:
            return
        self.depth_card.set_value(f"{depth:.1f} m")
        self.heading_card.set_value(f"{heading}\u00b0")

    # ---------- KONTROL (MANTIK AYNI, sadece gorsel) ----------
    def update_control_ui(self, is_main_active, current_mode, is_armed):
        self.is_main_active_gui = is_main_active
        self.is_armed_gui = is_armed
        accent = self.COL_MAIN if is_main_active else self.COL_MINI

        # --- ARM karti ---
        if is_armed:
            self.arm_card.set_value("ARMED")
            self._style_card_value(self.arm_card, self.COL_ARM)
            if not self._pulse_timer.isActive():
                self._pulse_timer.start(40)
        else:
            self.arm_card.set_value("DISARMED")
            self._style_card_value(self.arm_card, self.COL_DISARM)
            self._pulse_timer.stop()
            self.arm_card.setStyleSheet("")  # pulse rengini temizle

        # --- Arac karti ---
        if is_main_active:
            self.vehicle_card.set_value("ANA ROV")
        else:
            self.vehicle_card.set_value("MINI ROV")
        self._style_card_value(self.vehicle_card, accent)

        # --- Mod karti ---
        if is_main_active:
            self.mode_card.set_caption("UCUS MODU")
            self.mode_card.set_value(current_mode)
            self._style_card_value(self.mode_card, "#2ecc71")
        else:
            # Mini aktif: ANA arac zaten pasif. Bu kart artik MINI'nin
            # canli durumunu gosteriyor (HAZIR / BEKLEMEDE / JOY YOK /
            # ARDUINO YOK / BAGLANTI YOK).
            self.mode_card.set_caption("MINI DURUM")
            self.mode_card.set_value(current_mode)
            uyari = current_mode in ("BAGLANTI YOK", "ARDUINO YOK",
                                     "ARDUINO SESSIZ", "JOY YOK")
            self._style_card_value(self.mode_card,
                                   self.COL_ARM if uyari else self.COL_MINI)

        # --- Kamera vurgu + stretch ---
        if is_main_active:
            self.cam_main.setStyleSheet(
                f"QLabel#camMain{{background:#0a1420;border:2px solid {self.COL_MAIN};"
                f"border-radius:12px;color:{self.COL_MAIN};"
                f"font-family:'Consolas',monospace;font-weight:700;letter-spacing:2px;}}")
            self.cam_mini.setStyleSheet(
                "QLabel#camMini{background:#0a1420;border:2px solid rgba(255,255,255,0.10);"
                f"border-radius:12px;color:{self.COL_MUTED};"
                "font-family:'Consolas',monospace;font-weight:700;letter-spacing:2px;}")
            self.cam_layout.setStretch(0, 3)
            self.cam_layout.setStretch(1, 1)
        else:
            self.cam_main.setStyleSheet(
                "QLabel#camMain{background:#0a1420;border:2px solid rgba(255,255,255,0.10);"
                f"border-radius:12px;color:{self.COL_MUTED};"
                "font-family:'Consolas',monospace;font-weight:700;letter-spacing:2px;}")
            self.cam_mini.setStyleSheet(
                f"QLabel#camMini{{background:#0a1420;border:2px solid {self.COL_MINI};"
                f"border-radius:12px;color:{self.COL_MINI};"
                f"font-family:'Consolas',monospace;font-weight:700;letter-spacing:2px;}}")
            self.cam_layout.setStretch(0, 1)
            self.cam_layout.setStretch(1, 3)

        # --- Telemetri pasifken solsun ---
        if not is_main_active:
            self.depth_card.set_value("--")
            self.heading_card.set_value("--")
            self._style_card_value(self.depth_card, self.COL_MUTED)
            self._style_card_value(self.heading_card, self.COL_MUTED)
        else:
            self._style_card_value(self.depth_card, self.COL_TEXT)
            self._style_card_value(self.heading_card, self.COL_TEXT)

    def _style_card_value(self, card, color):
        obj = card.value.objectName()
        fam = "'Consolas','JetBrains Mono',monospace" if obj == "valueMono" else "'Arial'"
        size = "28px" if obj == "valueMono" else "26px"
        card.value.setStyleSheet(
            f"color:{color};font-family:{fam};font-size:{size};font-weight:800;")

    # ---------- ARMED PULSE (sadece gorsel) ----------
    def _pulse_tick(self):
        self._pulse_val += self._pulse_dir * 0.06
        if self._pulse_val >= 1.0:
            self._pulse_val = 1.0
            self._pulse_dir = -1
        elif self._pulse_val <= 0.0:
            self._pulse_val = 0.0
            self._pulse_dir = 1
        # 0->1 arasi kirmizi glow'u kartin kenarinda nabiz gibi degistir
        alpha = int(60 + self._pulse_val * 140)
        glow = int(40 + self._pulse_val * 40)
        self.arm_card.setStyleSheet(
            f"QFrame#card{{background-color:rgba({glow},20,20,0.55);"
            f"border:2px solid rgba(255,59,59,{alpha/255:.2f});border-radius:14px;}}")

    def closeEvent(self, event):
        self._pulse_timer.stop()
        self.ros_thread.stop()
        self.ros_thread.quit()
        self.ros_thread.wait(2000)
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    window = StationGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
