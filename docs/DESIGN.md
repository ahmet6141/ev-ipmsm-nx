# IPMSM tasarım gerekçesi

Bu üreteç, **2024-2026 yolcu-EV traksiyon sınıfı** radyal akı IPMSM'i temsil eder:
Tesla Model 3 / Model Y arka tahrik ünitesi (RDU) ölçeğinde, V dizilimli gömülü
NdFeB mıknatıslı, dağıtık **hairpin** sargılı, su/yağ soğutmalı bir IPM-SynRM.

Hedef performans bandı (referans): ~150–220 kW tepe güç, ~350–420 Nm tepe tork,
taban hız ~5–6k rpm, maksimum hız ~16–18k rpm, DC bara 400–800 V.

Değerler aşağıdaki kaynaklara dayanır: Tesla Model 3 teardown/FEA analizleri
(MotorXP), hairpin sargı tasarım literatürü ve elektrikli makine tasarım
referansları (bkz. dosya sonundaki kaynaklar).

## Varsayılan parametre seti

| Parametre | Değer | Birim | Not |
|---|---|---|---|
| stator OD | 225 | mm | Lamine dış çap (Model 3 RDU) |
| stator bore (iç çap) | 161 | mm | Split ratio (bore/OD) ≈ 0.715 (yüksek-hız EV optimumu 0.62–0.72) |
| rotor OD | 159.6 | mm | = bore − 2·hava aralığı |
| hava aralığı | 0.7 | mm | EV IPM traksiyon 0.5–0.8 mm |
| stack uzunluğu | 134 | mm | Aktif eksenel lamine boyu; L/D ≈ 0.6–0.8 |
| oluk / kutup | 54 / 6 | — | q = 54/(6·3) = 3, tam-oluklu dağıtık sargı |
| back-iron (boyunduruk) | 13 | mm | Boyunduruk akı yoğunluğu ~1.5–1.6 T |
| diş genişliği (min) | 5.6 | mm | Oluk adımı bore'da ≈ 9.37 mm; diş akısı ~1.6–1.7 T |
| oluk ağzı | 1.9 × 1.0 | mm | Yarı-kapalı; cogging/oluk-harmoniği düşürür |
| oluk gövde genişliği | ~3.9 | mm | `tooth_width`'ten türetilir (dikdörtgen hairpin oluğu) |
| iletken / oluk | 8 | — | 8 katmanlı hairpin; AC kaybı 2-kat'a göre ~%80 ↓ |
| paralel yol | 2 | — | Transpozisyon sonrası dengeli |
| mıknatıs G × K | 26 × 4.5 | mm | Bacak uzunluğu × manyetizasyon kalınlığı (V başına 2 adet) |
| V açısı | 145 | ° | Bacaklar arası açılım; geniş V mıknatıs torkunu, dar V relüktansı artırır |
| dış köprü | 1.0 | mm | Mıknatıs cebi ↔ rotor yüzeyi; santrifüj-kritik (mekanik FEA) |
| merkez nervür (yarı) | 1.0 | mm | d-ekseni postu (toplam ~2 mm); 16–18k rpm yükünü taşır |
| şaft çapı / boru | 45 / 14 | mm | İçi boş şaft (yağ-beslemeli rotor soğutma) |
| soğutma ceketi kalınlığı | 6 | mm | Su-glikol ceket, stator OD çevresi |
| lamine kalınlığı | 0.27 | mm | M250-35A sınıfı; ~900 Hz için demir kaybı/maliyet dengesi |

> 800V mimariler için: 0.20 mm (NO20) lamine + sargı tur/paralel-yol seçimi.

## Oluk/kutup seçimi

**Birincil: 54 oluk / 6 kutup (q = 3).** Tesla Model 3/Y arka tahrik yapılandırması;
hairpin V-mıknatıs EV traksiyon için en güvenli varsayılan: tam sayı q=3 → temel
sargı faktörü ~0.96, neredeyse sinüzoidal MMK, düşük tork dalgalanması, en düşük
5./7. harmonik içerik. 18.000 rpm'de elektriksel frekans yalnızca 900 Hz.

**Alternatif: 48 oluk / 8 kutup (q = 2).** Yaygın 800V seçeneği (sargı faktörü
~0.93); daha ince boyunduruk (hafif, küçük OD) ama daha yüksek frekans (16k'da
1067 Hz) ve AC kaybı. `configs/sweep_example.json` içinde bu varyant gösterilir.

**Geçerlilik kuralı:** `oluk % (kutup × faz) == 0`. Hairpin dağıtık sargı için
kesirli-oluk yoğun sargılar kapsam dışıdır. `em_design.validate()` bunu kontrol eder.

## V-mıknatıs geometrisi (parametrelendirme)

Üreteçteki V tek katmanlıdır (kutup başına bir V = 2 mıknatıs), Model 3 topolojisi.
[motor_nx/blueprint.py](../motor_nx/blueprint.py) içindeki yapı:

- **Eğim (tangenttan):** `tilt = (180 − v_angle)/2`. 180° V = düz tangansiyel bar
  (tilt 0); daha dar V daha diktir. `v_angle=145°` → tilt 17.5°.
- **Apeks (iç uçlar):** d-ekseni üzerinde, şaft yüzeyinden `vertex_gap` kadar dışta,
  her iki mıknatısın iç ucu d-ekseninden `center_post_halfwidth` kadar ayrık (nervür).
- **Cep = mıknatıs + akı bariyeri:** cep, mıknatısı uzunlamasına `end_barrier`
  (her uçta hava → **akı bariyeri**) ve enlemesine `pocket_clearance` kadar büyütür.
- **Mıknatıslar** ayrı katı gövdeler olarak ceplere yerleştirilir (NdFeB malzeme).

Doğrulama; mıknatısın rotor yüzeyini (`outer_bridge` payıyla) aşmamasını, şaft
üzerinde minimum web bırakmasını ve kutup yarım-adımını (inter-pole köprü payıyla)
aşmamasını garanti eder — böylece NX'te imkânsız bir kesim hiç denenmez.

## Malzemeler

- **Lamineler:** yönsüz silisli çelik 0.27 mm (M250-35A / 35JN sınıfı); rotor için
  yüksek-mukavemetli sınıf (≥450 MPa) ince köprü santrifüj gerilmesi için.
- **Mıknatıslar:** sinterlenmiş NdFeB, yüksek koersivite (N42UH / N45SH / N48SH),
  ~150–180 °C çalışma / demanyetizasyon güvenliği.
- **Sargı:** dikdörtgen bakır mıknatıs teli (C11000), yüksek-sıcaklık emaye; hairpin
  taçları lazer/TIG kaynaklı. Oluk doluluğu ~0.65–0.75 (yuvarlak telde ~0.4).
- **Şaft:** alaşımlı çelik (42CrMo4/4140). **Gövde:** döküm alüminyum + su ceketi.

## Türetilen büyüklükler

`em_design.derive()` şunları hesaplar: stator/rotor yarıçapları, oluk gövde iç/dış
yarıçapı, oluk derinliği/genişliği, kutup adımı, slots/pole/phase (q), dağıtım
faktörü kd, adım faktörü kp, sargı faktörü kw = kd·kp, split ratio, en-boy oranı.
`em_design.report()` bunları tek ekranda özetler.

## Kaynaklar (araştırma workflow'undan)

- MotorXP — Tesla Model 3 motor analizi: <https://motorxp.com/wp-content/uploads/mxp_analysis_TeslaModel3.pdf>
- Hairpin sargı tasarım kılavuzu (ResearchGate 358422319), MDPI Machines 10(11):1029
- Nottingham repo 7468083, IET Electric Power Applications, arXiv 2501.18200
- CleanTechnica / lesics — Model 3 motor genel bakış

> Bu değerler bir **başlangıç tasarımıdır**, optimize edilmiş nihai bir motor değil.
> Elektromanyetik (FEA) ve mekanik (santrifüj) doğrulama ayrı yapılmalıdır; özellikle
> dış köprü ve merkez nervür kalınlıkları maksimum aşırı-hızda mekanik FEA ister.
