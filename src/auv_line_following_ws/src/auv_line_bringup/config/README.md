# Bringup config

Bu klasör bilinçli olarak boş bırakıldı. Parametreler her katmanın kendi
paketinde tutulur (bkz. `line_perception/config/perception_params.yaml`
ve `line_control/config/control_params.yaml`).

Sahaya özel (ör. yarışma pisti, havuz derinliği) bir parametre seti
oluşturmak isterseniz, bu klasöre kendi `.yaml` dosyanızı ekleyip
`line_following_system.launch.py` içine `DeclareLaunchArgument` ile
bağlayabilirsiniz.
