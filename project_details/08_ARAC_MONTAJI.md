# Araç Montajı — Tüm Parçaların Birleştirilmesi (`vehicle_nx`)

Tüm alt sistemleri (motor, sürücü/inverter, aktarma organı, süspansiyon köşeleri,
şasi) tek bir **araç montajına** birleştiren üst katman. Her alt sistem kendi yerel
çerçevesinde ayrı bir `.prt` olarak kurulur; bu paket her parçanın araç içindeki
**konum + yönelimini** hesaplar ve NX journal'ı her parçayı konumlandırılmış
**component** olarak üst montaja ekler.

> Kaynak gerçeği koddur (`vehicle_nx/params.py`, `assembly.py`).
> `python -m vehicle_nx.cli report | validate | plan` çıktısına güven.

## Araç koordinat çerçevesi (ISO 8855)
- **+X** ileri (ön dingele doğru), **+Y** sol, **+Z** yukarı.
- Zemin z = 0; tekerlek/göbek merkezi z = lastik yarıçapı. Orijin şasi merkezi, dingil-ortası.

## Katmanlar

| Modül | Görev |
|---|---|
| `params.py` | Araç yerleşimi (dingil mesafesi, iz, lastik yarıçapı, tahrik düzeni) + parça dosyaları |
| `assembly.py` | NX'siz yerleşim matematiği: her parça için **(parça dosyası + orijin + 3×3 yönelim)** planı + `validate()` |
| `nx_assembler.py` | Siemens NX journal: her alt sistemi kur, sonra `Assemblies.AddComponent` ile konumlandır |
| `cli.py` | `report / validate / plan` |

## Yerel çerçeve → araç çerçevesi dönüşümleri
- **driveline / motor:** yerel +Z = dönme ekseni → araçta dingil sol-sağ (+Y); `Rx(-90°)`.
- **inverter:** kendi kutu çerçevesi, motorun üstüne dik biner (birim matris).
- **süspansiyon köşesi:** yerel X/Y/Z = araç eksenleri; sol köşe birim, sağ köşe `Rz(180°)`.
- **şasi:** platform olarak orijine (birim matris).

Tam cıvata-deliği eşleşmesi (mating) sonraki bir NX kısıt adımıdır; bu plan her parçayı
parametrik olarak konumlandırır, böylece montaj açılır açılmaz yerleşmiş olur — alt
sistem blueprint'lerindeki "temsilî" felsefenin aynısı.

## Varsayılan yerleşim (arka tahrik, 4 köşe)
```
CHASSIS          @ (    0,    0,    0)
DRIVELINE_REAR   @ (-1438,    0,  335)   Rx(-90)
MOTOR_REAR       @ (-1498,    0,  445)   Rx(-90)
INVERTER_REAR    @ (-1498,    0,  625)
SUSPENSION_FL/FR @ (+1438, ±790,  335)
SUSPENSION_RL/RR @ (-1438, ±790,  335)
```

## İleri seçenekler
- `layout.drive_layout`: `rear` | `front` | **`awd`** (AWD → iki e-aks: ön + arka).
- `layout.suspension_corners`: 2 (yalnız tahrikli dingil) | 4 (tüm köşeler).
- `parts.include_inverter` / `include_chassis` toggle.

## Kullanım
```bash
python -m vehicle_nx.cli report                # yerleşim + konumlar
python -m vehicle_nx.cli validate              # tutarlılık
python -m vehicle_nx.cli plan -o vehicle.json  # montaj planı JSON

# Siemens NX içinde (headless) — her parçayı kurar, sonra birleştirir:
"%UGII_ROOT_DIR%\run_journal.exe" vehicle_nx\nx_assembler.py -args vehicle.json vehicle.prt build
# parçalar zaten kuruluysa: ... vehicle.prt nobuild
```

Not: `nx_assembler.py`, `Assemblies.AddComponent` çağrısını NX 1900..2506 API
imza-kaymalarına karşı savunmacı yazılmıştır; gerçek bir NX oturumunda "smoke run"
ile teyit edilmesi önerilir (montaj/üretim özelliklerindeki not gibi).
