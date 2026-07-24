# models/

Eğittiğiniz YOLO11-seg `.pt` dosyasını bu klasöre koyun, örneğin:

```
models/line_seg_yolo11.pt
```

Ardından `line_perception/config/perception_params.yaml` içindeki
`model_path` parametresini bu dosyanın yoluna göre güncelleyin
(mutlak yol kullanmanız önerilir, örn. `/home/user/.../models/line_seg_yolo11.pt`).
