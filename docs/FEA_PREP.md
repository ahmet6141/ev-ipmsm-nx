# FEA hazırlık ve doğrulama planı — EV_IPMSM_traction

Bu doküman, parametrik olarak üretilen IPMSM geometrisini **sonlu elemanlar (FEA)**
çalışmasına hazır hâle getirir: geometri/malzeme/sınır-koşulu/sargı/uyarım girdileri
ve EM + termal + yapısal **doğrulama matrisi** + kabul kriterleri.

> **Önemli:** Buraya kadar üretilen her şey *birinci-mertebe* (analitik) tasarımdır.
> Kesin tork, kayıp, verim, demagnetizasyon ve gerilme değerleri **FEA'dan** gelir.
> Bu doküman o FEA'yı kurmak için gereken tüm girdileri ve adımları verir.
>
> Makine-okunur girdiler `python -m motor_nx.cli fea` ile `fea/` klasörüne üretilir:
> `fea_spec.json` (spec), `cross_section.dxf` (2D kesit, malzeme katmanlı), `winding.csv`
> (slot→faz). 3D geometri zaten `*_ap242.stp` / `*.x_t` olarak dışa aktarılıyor.

---

## 1. Geometri devri (hand-off)

| | |
|---|---|
| 3D katı | STEP AP242 (`*_ap242.stp`) + Parasolid (`*.x_t`) — builder üretir |
| 2D kesit | `fea/cross_section.dxf` — malzeme katmanlı (STATOR_STEEL, SLOT_AIR, ROTOR_STEEL, POCKET_AIR, MAGNET, COPPER, SHAFT) |
| Stator OD / bore | 225 / 161 mm |
| Rotor OD / hava aralığı | 159.6 mm / 0.7 mm |
| Aktif paket / şaft | 134 mm / Ø45 mm |
| Oluk / kutup | 54 / 6 |

**Simetri (önemli — çözüm süresini ~6× kısaltır):**
- **1 kutup sektörü (60°)** modelle, iki radyal kesim yüzeyinde **anti-periyodik (tek)**
  master/slave sınır koşulu (alan her kutupta işaret değiştirir).
- Alternatif: **1 kutup-çifti (120°)**, **periyodik (çift)** master/slave.
- Stator dış çapında **A = 0** (manyetik vektör potansiyeli) — akı içeride.

---

## 2. Malzemeler

### 2.1 Lamine çeliği — 0.27 mm NO silisli çelik (M250-27 sınıfı)
- Paketleme faktörü 0.96, yoğunluk 7650 kg/m³.
- **BH eğrisi (temsili NO silisli çelik — nihai koşu için tedarikçi datasheet'i kullan):**

| H (A/m) | 0 | 50 | 100 | 150 | 200 | 300 | 500 | 1000 | 2000 | 5000 | 10000 | 30000 | 80000 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B (T) | 0 | 0.55 | 0.95 | 1.18 | 1.33 | 1.49 | 1.60 | 1.71 | 1.80 | 1.90 | 1.96 | 2.05 | 2.20 |

- **Demir kaybı:** ~2.3 W/kg @ 1.5 T, 50 Hz. FEA'da Bertotti (histerezis+eddy+excess)
  veya Steinmetz katsayılarını datasheet'ten kalibre et.

### 2.2 Mıknatıs — NdFeB N42SH
| Özellik | Değer |
|---|---|
| Br (20 °C) | 1.28 T |
| Hcb / Hcj | 915 / 1592 kA/m |
| Geri-dönüş geçirgenliği μ_rec | 1.05 |
| Br sıcaklık katsayısı | −0.12 %/°C |
| Hcj sıcaklık katsayısı | −0.55 %/°C |
| Maks servis sıcaklığı | 150 °C |
| Yoğunluk / özdirenç | 7500 kg/m³ / 1.4 µΩ·m |
| Eksenel segment | 4 (eddy kaybı için) |

> **Demag kontrolü:** B-H 2. bölge dizini (knee), *sıcak* çalışma sıcaklığında (150 °C)
> ve *tepe* zıt-akıda değerlendirilir — mıknatıs B'si knee'nin altına düşmemeli.

### 2.3 İletken / şaft / housing
- **Bakır:** σ = 5.96×10⁷ S/m (20 °C), sıc. katsayısı 0.00393 /°C, H sınıf yalıtım, dolum 0.62.
- **Şaft:** alaşımlı çelik (ör. 42CrMo4), akma ~750 MPa.
- **Housing/ceket:** döküm alüminyum (su ceketi).

---

## 3. Sargı

**Tip:** çift-katman tam-adım (full-pitch) tam-oluklu lap hairpin sargı.
- Faz 3, q = 3 oluk/kutup/faz, oluk başına 8 hairpin bar, 2 paralel yol.
- Seri sarım/faz = 36, coil pitch = 9 oluk, sargı faktörü kw ≈ 0.96.
- **d-ekseni:** referans kutup için rotor V-mıknatıs simetri ekseni **+X**'te.

**Slot→faz haritası (60° faz-kuşağı deseni, her kuşak q=3 oluk):**

| Kuşak (3'er oluk) | 0–2 | 3–5 | 6–8 | 9–11 | 12–14 | 15–17 | (18'de tekrar) |
|---|---|---|---|---|---|---|---|
| Faz | **A+** | **C−** | **B+** | **A−** | **C+** | **B−** | ↺ |

54 oluk için bu desen 3 kez tekrarlar. Tam liste: `fea/winding.csv`.
(Tam-adım olduğundan her oluğun iki katmanı da aynı fazı taşır.)

---

## 4. Uyarım ve çalışma noktaları

| | |
|---|---|
| DC bara | 350 V |
| Nominal akım | 130 A rms |
| Tepe akım | 282 A rms |
| Akım ilerleme açısı (q-ekseninden β) | 0–45° tara → **MTPA** noktasını bul |
| Taban hız / maks hız | 4307 / 18000 rpm |
| Köşe torku (tepe/sürekli) | 473 / 218 Nm (analitik hedef) |

IPM çıkıklığı **relüktans torku** ekler; MTPA tipik olarak β ≈ 15–35°'de.

---

## 5. Ağ (mesh) önerisi
- **Hava aralığı:** en az **3 radyal katman** (eleman ~0.23 mm); hareketli bant (moving band).
- **İnce/kritik bölgeleri sıklaştır:** dış köprü (outer_bridge 1 mm), merkez post (1 mm),
  diş uçları, mıknatıs köşeleri — buralar doygunluk ve gerilme açısından kritik.
- Global eleman ~2 mm.

---

## 6. EM analiz matrisi (2D)

| # | Analiz | Çıktı / amaç |
|---|---|---|
| 1 | **Cogging** | Akımsız, ince rotor adımlarıyla cogging torku (1 oluk-adımı boyunca) |
| 2 | **Geri-EMK** | Açık devre faz akı-bağı ve EMK (taban hızda); büyüklük vs Vdc + harmonik (THD) |
| 3 | **Ld/Lq** | (id, iq) taramasıyla akı haritaları → Ld, Lq, çıkıklık oranı |
| 4 | **Tork–açı** | Nominal & tepe akımda β taraması → ortalama tork, **MTPA** lokusu |
| 5 | **Tork dalgalanması** | MTPA'da rotor adımlarıyla anlık tork → ripple % |
| 6 | **Demagnetizasyon** | Tepe zıt-akı + 150 °C → min mıknatıs B'si vs knee |
| 7 | **Kayıplar** | Demir (stator/rotor), mıknatıs eddy (segmentli), AC bakır (hairpin skin/proximity) |
| 8 | **Verim haritası** | Tork-hız zarfında verim (taban üstü alan zayıflatma) |

---

## 7. Termal plan
- **Soğutma:** 12 eksenel kanallı su ceketi. Girdi: EM kayıpları (bakır+demir+mıknatıs),
  soğutucu (su-glikol), debi, giriş sıcaklığı.
- **Sıcak noktalar:** sargı (oluk + uç-sargı) ve **mıknatıs** (150 °C SH sınırı — demag riski).
- **Sürekli anma:** kararlı-hâl sıcaklıkların sınıra (sargı ~180 °C H sınıfı, mıknatıs 150 °C)
  ulaştığı tork-hız noktaları.
- Araç: Motor-CAD (termal), Ansys Mechanical/Fluent, veya yumuşak-bağ (lumped) ağ.

## 8. Yapısal plan
- **Rotor santrifüj gerilmesi @ 1.2× maks hız (21600 rpm):** dış köprüler ve merkez post'ta
  von Mises; lamine akmasına karşı **emniyet katsayısı ≥ 1.5**.
- **Mıknatıs tutma:** köprüler 18000 rpm santrifüj kuvvetine karşı mıknatısları tutmalı.
- **Rotordinamik:** şaft-rotor 1. eğilme kritik hızı, maks çalışma hızının üstünde (marj ile).

---

## 9. Kabul kriterleri (hedefler)

| Ölçüt | Hedef |
|---|---|
| Tepe tork | ≥ 426 Nm |
| Sürekli tork | ≥ 196 Nm |
| Tork dalgalanması | ≤ %5 |
| Cogging | nominal torkun ≤ %1'i |
| Demagnetizasyon | tepe akım + 150 °C'de **yok** |
| Mıknatıs sıcaklığı (sürekli) | ≤ 150 °C |
| Rotor emniyet katsayısı (1.2× maks hız) | ≥ 1.5 |
| Tepe verim | ≥ %95 |

---

## 10. Araç önerileri
- **2D EM:** Ansys Maxwell 2D · Ansys Motor-CAD (E-Magnetic) · **FEMM / pyFEMM** (ücretsiz)
- **Termal:** Motor-CAD Thermal · Ansys Mechanical/Fluent · lumped-parametre ağı
- **Yapısal:** Ansys Mechanical · herhangi bir rotor-gerilme FEA

---

## 11. İş akışı (özet)
1. `python -m motor_nx.cli fea` → `fea/` (spec + DXF + winding) üret.
2. DXF'i 2D EM çözücüye al; katmanlara malzeme ata (Bölüm 2).
3. 1-kutup anti-periyodik sektör + A=0 dış sınır kur (Bölüm 1).
4. Sargıyı `winding.csv`'ye göre uyarımla (Bölüm 3–4).
5. Analiz matrisini koştur (Bölüm 6); MTPA'yı bul.
6. EM kayıplarını termale, geometriyi yapısala besle (Bölüm 7–8).
7. Kabul kriterleriyle (Bölüm 9) karşılaştır; tasarımı `params.py` üzerinden iterasyon et,
   `cli validate` ile geometriyi doğrula, yeniden üret.

> Geometri/malzeme bir parametre değişince: `params.py`'yi düzenle → `cli validate` →
> `cli fea` → builder'ı yeniden koştur. Tüm zincir parametriktir.
