# Sürücü / Inverter — Traksiyon Motor Sürücüsü (`inverter_nx`)

Motoru süren güç elektroniği + kontrol "beyni". `motor_nx` ile aynı katman
mimarisini izler; motorun gerçek performans değerlerinden (Vdc, tepe akım,
geri-EMK, taban/maks devir) sürücüyü boyutlandırır.

> Kaynak gerçeği koddur (`inverter_nx/params.py`, `engineering.py`).
> `python -m inverter_nx.cli report | validate | thermal` çıktısına güven.

## Katmanlar

| Modül | Görev |
|---|---|
| `params.py` | Bara / motor-arayüz / güç-katı / DC-link / rejen / kontrol / soğutma / muhafaza parametreleri |
| `engineering.py` | Anahtar gerilim-akım sınıfı, alan-zayıflatma oranı, DC-link ripple, rejen sınırı, kayıp/ısı akısı + `validate()` |
| `blueprint.py` | Muhafaza + soğutma plakası + DC-link kondansatör + 6 SiC modül geometrisi (build-step) |
| `nx_builder.py` | Siemens NX journal (motor_nx motorunu yeniden kullanır) |
| `cli.py` | `report / validate / blueprint / thermal` |

## Varsayılan tasarım (400 V bara) — temel sonuçlar

- **Güç katı:** SiC MOSFET, **750 V** sınıf (Vdc×1.8'e göre +30 V pay), SVPWM, **12 kHz**.
- **Akım:** tepe faz 273 A rms → 386 A tepe; anahtar derecelendirmesi **502 A** (×1.30 pay).
- **Alan zayıflatma:** 18000 rpm'de geri-EMK **1406 V** (LL) vs 245 V SVPWM tavanı → **oran 5.74** (FW zorunlu, MTPV bölgesi).
- **DC-link:** 500 µF film, ripple ~232 A rms (kondansatör ≥ 278 A).
- **Rejeneratif fren:** 70 kW etkin (batarya kabul sınırı), harmanlı fren (blended braking).
- **Kontrol:** FOC + MTPA + alan zayıflatma, resolver + sensörsüz yedek, **ASIL-C**, STO + aktif kısa-devre (ASC), CAN-FD.
- **Termal:** tepe kayıp 2.51 kW → 220×180 mm soğutma plakasında 6.3 W/cm².

## İleri/yeni nesil özellikler
- 400 V / **800 V mimari** seçimi (1200 V SiC), tork-vektörleme uyumu.
- Fonksiyonel güvenlik (ISO 26262 ASIL-C, STO, ASC), CAN-FD/Otomotiv Ethernet.
- Adaptif termal kontrol, spread-spectrum PWM (EMC/NVH), MTPA/FW tabloları motor modelinden türetilebilir.

## Kullanım
```bash
python -m inverter_nx.cli report      # boyutlandırma özeti
python -m inverter_nx.cli validate    # sağlamlık + buildability
python -m inverter_nx.cli thermal     # kayıp / ısı akısı
python -m inverter_nx.cli blueprint -o inv.json
```
