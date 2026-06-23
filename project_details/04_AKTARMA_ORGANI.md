# Aktarma Organı — Diferansiyel + Yarım Aks + Tekerlek Göbeği (`driveline_nx`)

Bu doküman, motora bağlanan **aktarma organını** anlatır: tek-kademeli redüksiyon
+ diferansiyel, iki yarım aks (CV mafsallı) ve tekerlek göbekleri (bijon bağlantı
elemanlarıyla). Paket, `motor_nx` ile **birebir aynı katman mimarisini** kullanır
ve aynı NX builder geometri motorunu yeniden çağırır — yeni NXOpen kodu yoktur.

> Kaynak gerçeği koddur (`driveline_nx/params.py`, `engineering.py`, `blueprint.py`).
> Çelişki görürsen `python -m driveline_nx.cli report | validate | bearing`
> çıktısına güven, sonra burayı güncelle.

## Katmanlar

| Modül | Görev |
|---|---|
| `driveline_nx/params.py` | Parametreler (dataclass, JSON) — diferansiyel, yarım aks, göbek, malzeme |
| `driveline_nx/engineering.py` | Oran / tork kapasitesi / mil burulma gerilimi / rulman ömrü + `validate()` |
| `driveline_nx/blueprint.py` | NX'ten bağımsız geometri (build-step listesi); `motor_nx.BuildStep`'i kullanır |
| `driveline_nx/nx_builder.py` | Siemens NX journal — `motor_nx`'in sertleştirilmiş NXOpen motorunu yeniden kullanır |
| `driveline_nx/cli.py` | NX'siz komut satırı: `report / validate / blueprint / bearing` |
| `tests/test_driveline.py` | Birim testleri (params round-trip, mühendislik, geometri bütünlüğü) |

## Koordinat ve düzen

- **Z = tekerlek dönme ekseni.** Diferansiyel merkezi z = 0.
- Sağ yarım aks +Z'ye, sol yarım aks −Z'ye büyür (simetrik). Yarım akslar diferansiyel
  taşıyıcı ekseniyle eş eksenlidir; ayna dişli (ring gear) bunların etrafında oturur.
- Dişli ve spline dişleri **pitch/major çapta blank** olarak modellenir (hobbing /
  broşlama ile kesilir) — `motor_nx`'in sargıları "envelope" ile temsil etmesiyle aynı
  felsefe. Üretim arayüzleri (flanşlar, civata daireleri, delikler, kama yuvaları) ise
  birebir modellenir.

## Parçalar (varsayılan değerlerle, motorun 440 Nm tepe torkuna göre)

### Diferansiyel + tek-kademeli redüksiyon
- **Tip seçilebilir** (`differential.type`): `open` | `elsd` | **`torque_vectoring`** (varsayılan) | `spool`.
  Ayrıca **`disconnect`** (boşta sürüklenmeyi kesen kavrama) opsiyonu.
- Final oran **9:1** → 3960 Nm ayna dişli torku; tekerlek başına pay diferansiyel
  tipine göre (açık 0.50, e-LSD/TV 0.60, spool 1.0 — en kötü hal mil boyutlaması için).
- Taşıyıcı (carrier) tüp blank + iki yan dişli (side gear) çıkış göbeği.
- **Giriş pinyon + motor bağlantı flanşı** (paralel-eksen, ofset): merkezdeki delik
  motorun **Ø32 kamalı çıkış miline** birebir oturur (DIN 6885-A kama yuvası dahil) +
  flanş civata dairesi.

### Yarım akslar (her iki taraf)
- **Ø34 mm**, içi boş **Ø18 mm** delikli (hafif + yüksek burulma modu/NVH) — burulma
  emniyet katsayısı SF ≈ 1.26 (allow 420 MPa).
- İçeride **tripod** (dalmalı), dışarıda **Rzeppa** (sabit, ~47° eklem) CV mafsalları —
  çan gövdeleri (bell) blank olarak.

### Tekerlek göbeği — "tekerlek bağlantı elemanları"
- **Gen-3 göbek rulman ünitesi** (OD 84 × W 39 mm).
- **Bijon delik takımı:** 5 × PCD 114.3, Ø14 saplama (veya `single_centre_nut=True`
  ile motorsport tek-merkez-somun).
- **Merkez pilot deliği** (hub-centric, Ø64.1), **fren diski pilot yüzeyi**,
  **ABS/tekerlek-hız enkoder halkası**.

## İleri/yeni nesil özellikler

- **Tork vektörleme eDiff** (twin-clutch) ve **e-LSD** tipleri parametrik.
- **Disconnect/decoupler** — menzil için sürüklenmesiz serbest dönüş.
- **İçi boş hafifletilmiş yarım aks** opsiyonu.
- **ABS enkoder halkası** ve **tek-merkez-somun** (motorsport) opsiyonları.
- Tüm yükler motorun tepe torkundan türetilir; `validate()` her özelliği geometrik +
  mukavemet (SF) açısından NX builder çalışmadan önce denetler.

## Kullanım

```bash
python -m driveline_nx.cli report                 # tasarım özeti + dereceler
python -m driveline_nx.cli validate               # buildability (exit kodu)
python -m driveline_nx.cli blueprint -o dl.json   # NX build-step JSON
python -m driveline_nx.cli bearing --load 6000    # göbek rulmanı L10 ömür tahmini

# Siemens NX içinde (headless):
"%UGII_ROOT_DIR%\run_journal.exe" driveline_nx\nx_builder.py -args dl.json out.prt step parts
```

`nx_builder.py` argümansız çalışırsa varsayılan EV aktarma organını kurar; `parts`
her bileşeni (Differential, Ring_Gear, Halfshaft_L/R, CV_Joints, Wheel_Hub_L/R) ayrı
STEP olarak parça-parça üretim için dışa aktarır.
