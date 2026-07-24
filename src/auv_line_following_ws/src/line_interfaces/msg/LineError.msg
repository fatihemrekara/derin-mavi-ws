# LineError.msg
#
# Su altı hat takip görevinde algılama katmanından (YOLO11-seg + OpenCV)
# kontrol katmanına iletilen hata bilgisi.
#
# NOT: Orijinal tasarıma ek olarak "header" alanı eklendi. Bu alan;
#   - Kontrol düğümündeki watchdog'un mesaj yaşını (gecikme) ölçmesini,
#   - RViz / rqt gibi araçlarla zaman senkronizasyonlu debug yapılmasını
#   sağlar. is_line_lost, kameranın "çizgiyi görmediğini" bildirir;
#   header ise "düğümün hiç mesaj göndermediği" durumları (ör. çökme)
#   ayırt etmek için kullanılır.
std_msgs/Header header

# Çizginin merkeze olan piksel uzaklığı (Sağ pozitif, Sol negatif)
float32 x_error

# Çizginin merkez ile yapmış olduğu açı (derece cinsinden).
# Kural: Görüntüdeki dikey eksene (aracın ileri yönü) göre ölçülür.
#   0        -> çizgi tam dikey (araç hizalı)
#   pozitif  -> çizginin üst ucu sağa yatık (saat yönünde düzeltme gerekir)
#   negatif  -> çizginin üst ucu sola yatık (saat yönü tersine düzeltme gerekir)
float32 angle_error

# Çizgi tamamen kaybolduğunda veya görev bittiğinde sistemi uyarmak için
bool is_line_lost
