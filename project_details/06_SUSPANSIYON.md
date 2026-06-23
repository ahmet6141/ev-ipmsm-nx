# Süspansiyon Sistemi (`suspension_nx`)

Tekerlek göbeğini (driveline_nx) taşıyan köşe süspansiyonu. `motor_nx` ile aynı
katman mimarisini izler. Varsayılan arka **multi-link** (ayrıca `double_wishbone`
ve `macpherson`). Yerel çerçeve: X boyuna, Y yanal (dış = +Y), Z düşey.

> Kaynak gerçeği koddur (`suspension_nx/params.py`, `engineering.py`).
> `python -m suspension_nx.cli report | validate | rates` çıktısına güven.

## Katmanlar

| Modül | Görev |
|---|---|
| `params.py` | Geometri / yay / amortisör / kol / denge çubuğu / aks taşıyıcı / kütle parametreleri |
| `engineering.py` | Tekerlek oranı, sürüş frekansı, yalpa rijitliği, sönüm oranı, tekerlek-hop frekansı + `validate()` |
| `blueprint.py` | Aks taşıyıcı + salıncaklar + yay + amortisör + denge çubuğu + burç/rotil geometrisi |
| `nx_builder.py` | Siemens NX journal (motor_nx motorunu yeniden kullanır) |
| `cli.py` | `report / validate / blueprint / rates` |

## Varsayılan tasarım — temel sonuçlar
- **Tip:** multi-link; iz 1580 mm, sürüş yüksekliği 140 mm.
- **Direksiyon ekseni:** KPI 8°, kaster 5°, kamber −1.5°, scrub 15 mm.
- **Kollar:** alt 380 / üst 300 / toe 320 mm.
- **Yay:** 45 N/mm, hareket oranı (MR) 0.62 → **tekerlek oranı 17.3 N/mm**.
- **Sürüş frekansı:** **1.02 Hz** (yaylı kütle 420 kg) — 0.8–2.0 Hz konfor bandında.
- **Amortisör:** Ø46×420 mm, adaptif (CDC); sönüm oranı ~0.65 (bump).
- **Denge çubuğu:** Ø24, 480 Nm/deg → **yalpa rijitliği ~857 Nm/deg**.
- **Aks taşıyıcı (knuckle):** 180×120×40 mm, göbek deliği **Ø84** (driveline Gen-3 hub rulmanıyla uyumlu) + kaliper bağlantısı.

## İleri/yeni nesil özellikler
- Adaptif (yarı-aktif/CDC) amortisör, çok-bağlantılı (multi-link) kinematik.
- `corners="axle"` ile aynalı çift köşe modelleme.
- Geometri parametrik: kamber/kaster/KPI/scrub doğrudan ayarlanır, `validate()` her özelliği geometrik + frekans bandı açısından denetler.

## Kullanım
```bash
python -m suspension_nx.cli report
python -m suspension_nx.cli rates              # tekerlek oranı / frekans / yalpa
python -m suspension_nx.cli blueprint -o sus.json
```
