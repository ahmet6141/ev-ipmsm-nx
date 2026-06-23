# Şasi / Çerçeve (`chassis_nx`)

Bataryayı barındıran ve ön/arka alt-şasi ile e-aks'ı (motor + aktarma organı)
taşıyan **skateboard** EV platformu. `motor_nx` ile aynı katman mimarisini izler.
Araç çerçevesi: X boyuna (ön +X), Y yanal, Z düşey.

> Kaynak gerçeği koddur (`chassis_nx/params.py`, `engineering.py`).
> `python -m chassis_nx.cli report | validate | mass` çıktısına güven.

## Katmanlar

| Modül | Görev |
|---|---|
| `params.py` | Çerçeve / batarya tepsisi / alt-şasi / gövde bağlantısı / malzeme parametreleri |
| `engineering.py` | Kesit alanı + atalet momentleri, kütle dökümü, burulma rijitliği tahmini, ağırlık dağılımı + `validate()` |
| `blueprint.py` | Kutu kiriş raylar + çapraz elemanlar + batarya tepsisi + alt-şasi bossları + bağlantı delikleri |
| `nx_builder.py` | Siemens NX journal (motor_nx motorunu yeniden kullanır) |
| `cli.py` | `report / validate / blueprint / mass` |

## Varsayılan tasarım — temel sonuçlar
- **Yerleşim:** skateboard, dingil mesafesi 2875 mm, iz 1580/1580 mm (ön/arka), toplam uzunluk 4690 mm.
- **Çerçeve rayı (kutu kiriş):** 70×120 mm, 4 mm cidar (iç açıklık 1100 mm) → kesit A 1456 mm², Ix 2.82e6 mm⁴.
- **Çapraz elemanlar:** 5 × 60×90 mm.
- **Batarya tepsisi:** 2400×1450×110 mm, 4 mm cidar, 6 çapraz takviye, sızdırmaz.
- **Alt-şasi:** ön+arka, 4× Ø14 cıvata, 3 motor bağlantısı; **gövde bağlantısı** 10× Ø12, ön/arka crush-can (çarpışma yapısı).
- **Malzeme:** 6082-T6 ekstrüzyon alüminyum (MIG + yapısal yapıştırıcı + FDS).
- **Kütle (yapısal):** **142.9 kg** (raylar 36.9 + çapraz 12.8 + tepsi 93.2).
- **Burulma rijitliği:** ~20.558 Nm/deg (KABA tek-tüp tahmini — FEA ile doğrulanmalı).
- **Ağırlık dağılımı / CG:** ~50/50, CG yüksekliği ~115 mm (batarya alçak).

## İleri/yeni nesil özellikler
- Skateboard mimari (alçak CG, düz batarya tepsisi), alüminyum ekstrüzyon + yapısal yapıştırma.
- Sızdırmaz batarya tepsisi, ön/arka crush-can çarpışma yapıları.
- Parametrik dingil/iz/kesit; `validate()` rayların iz içine sığması, tepsi-ray uyumu, cidar oranları gibi kuralları NX builder çalışmadan denetler.

## Kullanım
```bash
python -m chassis_nx.cli report
python -m chassis_nx.cli mass                  # kütle dökümü + burulma rijitliği
python -m chassis_nx.cli blueprint -o chassis.json
```
