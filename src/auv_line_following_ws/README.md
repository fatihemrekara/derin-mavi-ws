# AUV Su Altı Hat Takip Sistemi (ROS 2 + YOLO11-seg + OpenCV)

Bu workspace, bir AUV'a takılı kameradan gelen görüntüyü YOLO11 segmentasyon
modeliyle işleyip, OpenCV ile geometrik hataya (x_error, angle_error)
dönüştürerek, PID tabanlı bir kontrolcü ile aracı 6 eksende (surge, sway,
heave, roll, pitch, yaw) yönlendiren modüler bir ROS 2 sistemidir.

## Mimari

```
Kamera --(sensor_msgs/Image)--> [yolo_segmentation_node]
                                        | (binary mask, sensor_msgs/Image)
                                        v
                              [line_extraction_node]  (OpenCV)
                                        | (line_interfaces/LineError)
                                        v
                          [line_follower_controller_node]  (PID + durum makinesi)
                                        | (geometry_msgs/Twist, 6 eksen)
                                        v
                              Thruster tahsisi (harici / mevcut sistem)
```

Sohbetteki diyagramda bu akış görsel olarak da gösterildi.

## Paketler

| Paket              | Tip           | Sorumluluk                                                        |
|--------------------|---------------|---------------------------------------------------------------------|
| `line_interfaces`  | ament_cmake   | `LineError.msg` mesaj tanımı                                       |
| `line_perception`  | ament_python  | YOLO11-seg segmentasyonu + OpenCV ile x_error/angle_error çıkarımı |
| `line_control`     | ament_python  | PID kontrolcüleri, durum makinesi, 6 eksen `Twist` üretimi         |
| `auv_line_bringup` | ament_cmake   | Tüm sistemi tek launch dosyasıyla başlatma                         |

Her paket bağımsız olarak test edilebilir ve değiştirilebilir; örneğin YOLO
modelini değiştirmek `line_perception` dışına hiçbir etki etmez, kontrol
mantığını değiştirmek `line_control` dışına dokunmaz.

## Kurulum

```bash
# ROS 2 (Humble/Iron/Jazzy) ve colcon kurulu olmalı
sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv

cd auv_line_following_ws
pip install -r requirements.txt --break-system-packages

colcon build --symlink-install
source install/setup.bash
```

Eğittiğiniz YOLO11-seg modelini `models/` klasörüne koyun ve
`line_perception/config/perception_params.yaml` içindeki `model_path`
alanını güncelleyin (bkz. `models/README.md`).

## Çalıştırma

```bash
# Tüm sistem (algılama + kontrol)
ros2 launch auv_line_bringup line_following_system.launch.py

# Sadece algılama katmanını test etmek için
ros2 launch line_perception perception.launch.py

# Sadece kontrol katmanını test etmek için (sahte LineError yayınlayarak)
ros2 launch line_control control.launch.py
```

Debug görüntüleri:
- `/line_perception/annotated_image` — YOLO11 tespit çizimi
- `/line_follower/debug_image` — kontur, ağırlık merkezi ve yön oku

Durum takibi: `/line_follower/state` (`SEARCHING`, `TRACKING`, `LINE_LOST`,
`HOLD`).

## Birim testleri

`line_control` içindeki `PIDController` ve `LineFollowerStateMachine`
sınıfları ROS'a bağımlı değildir, bu yüzden saf `pytest` ile test edilebilir:

```bash
cd src/line_control
python3 -m pytest test/ -v
```

## Tasarım kararları / varsayımlar

1. **`LineError.msg`'ye `header` eklendi.** Sizin verdiğiniz 3 alan
   (`x_error`, `angle_error`, `is_line_lost`) aynen korundu; ek olarak
   `std_msgs/Header header` eklendi. Bu, kontrol düğümündeki watchdog'un
   "mesaj hiç gelmiyor" (düğüm çökmüş) durumunu "çizgi görülmüyor"
   (`is_line_lost=true`) durumundan ayırt etmesini sağlıyor. İstemezseniz
   tek satır silerek kaldırabilirsiniz.
2. **`angle_error` derece cinsinden**, dikey eksene göre; pozitif değer
   çizginin üst ucunun sağa yattığını belirtir. Radyan tercih ederseniz
   değişiklik tek noktada (`line_extraction_node._analyze_contour`) yapılır.
3. **Çıkış mesajı `geometry_msgs/Twist`** seçildi (özel bir "6DOF komut"
   mesajı yerine). Bu, mevcut/gelecek herhangi bir thruster tahsis
   düğümüyle (ör. ArduSub/mavros, kendi mixer'ınız) doğrudan uyumlu, yaygın
   bir arayüzdür.
4. **Roll/pitch/heave bilinçli olarak 0 (pass-through)** bırakıldı; bu
   eksenlerin genelde ayrı bir derinlik/IMU stabilizasyon katmanına ait
   olduğu varsayıldı. Gerekirse `line_follower_controller_node` içine
   `depth_setpoint` / `attitude_setpoint` girişleri eklemek kolaydır.
5. **`is_line_lost` debounce'lu**: tek karelik gürültüyle kaybolmayı
   önlemek için `lost_frame_threshold` kadar ardışık kare gerekiyor.
6. **Watchdog**: `LineError` akışı `message_timeout_sec` süresince
   kesilirse araç otomatik olarak durur (HOLD).

## Güvenlik ve test notları (gerçek suya inmeden önce)

- Yaw işaret kuralını (`angle_error` pozitifken aracın hangi yöne dönmesi
  gerektiğini) kendi aracınızın gerçek dönüş yönüyle mutlaka doğrulayın;
  ters ise `yaw_kp` işaretini çevirin.
- İlk testleri havuzda düşük `nominal_surge_speed` ve düşük PID
  kazançlarıyla yapın, kademeli artırın.
- Bir kill-switch / acil durdurma mekanizmasının bağımsız olarak (bu
  yazılım yığınının dışında) çalıştığından emin olun.
- `hold_timeout_sec` ve `message_timeout_sec` değerlerini görev alanınızın
  risk toleransına göre ayarlayın.
