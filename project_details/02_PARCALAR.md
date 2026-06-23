# 02 — Parça Parça Şartname

Her parça için: **işlevi · ana geometri · malzeme · üretim yöntemi · montaj/üretim
özellikleri (delikler vb.) · kritik toleranslar**. Sayılar varsayılan tasarımdan
(`MotorParams()`); `python -m motor_nx.cli bom | tolerances | hardware` ile canlı
üretilir. Montaj özelliklerinin tam kataloğu: [03_MONTAJ_OZELLIKLERI.md](03_MONTAJ_OZELLIKLERI.md).

NX gövde adları: `STATOR_STEEL`, `ROTOR_STEEL`, `MAGNET_nnn`, `COIL_nnn`, `SHAFT`,
`HOUSING`. Parça-başına STEP export: `run_journal nx_builder.py -args out.prt both parts`.

---

## 1. Stator lamine paketi  (`STATOR_STEEL`)

- **İşlev:** sargıyı taşır, manyetik akı yolu (diş + boyunduruk).
- **Geometri:** OD 225, bore 161, 54 yarı-kapalı paralel-duvarlı (hairpin) oluk,
  diş 5.6 mm (min, r1'de), boyunduruk (back-iron) 13 mm, oluk derinliği ~12.5 mm.
- **Malzeme:** 0.27 mm NO silisli çelik (M250-27 sınıfı), paketleme faktörü 0.96.
- **Üretim:** ilerlemeli pres kalıbı (lazer yalnız prototip); **backlack** bağlama;
  bore + OD bağlama sonrası finiş işlenir.
- **Montaj/üretim özellikleri:**
  - **Tie-rod / bağlama delikleri** — boyundurukta 6 × Ø6.5 eksenel delik
    (pitch R≈106 mm). Paketi sıkıştırır / handle eder / gövdeye konumlar. (M6)
  - **Dönmez OD kaması** — OD'de 2 × 6×2.5 mm eksenel kama-çentiği; gövdedeki
    kamayla eşleşir, shrink-fit üzerinde arıza torkunu karşılar.
- **Toleranslar:** bore IT7/H7 (+0.040/0), silindiriklik 0.015, runout 0.02–0.03 TIR
  (Datum A); OD h6/js6 shrink; paket boyu 134 ±0.30; oluk/diş ±0.02.

## 2. Rotor lamine paketi  (`ROTOR_STEEL`)

- **İşlev:** mıknatısları tutar, d/q-eksen relüktans yolu; şafta press/shrink oturur.
- **Geometri:** OD ~159.6, bore = şaft 45 mm, 6 V-kutup; dış köprü 1.0 mm, d-eksen
  merkez nervürü 2.0 mm, V-açı 145°, mıknatıs apex'i şaft yüzeyinden 38 mm yukarıda.
- **Malzeme:** stator ile aynı NO silisli çelik; backlack veya perçin/kilit; mıknatıs
  köprülerinde delici kaynak yapılmaz.
- **Üretim:** pres kalıbı; paketle-bağla; rotor OD bağlama sonrası finiş.
- **Montaj/üretim özellikleri:**
  - **Rivet / uç-plaka tutma delikleri** — hub çeliğinde 6 × Ø5 eksenel delik
    (pitch R≈39.6 mm, şaft ile V-cepleri arasında). Rotor uç-plakalarını tutar,
    paketleme sırasında hizalar.
  - **Bore kama yuvası** — opsiyonel (varsayılan KAPALI; tasarım press/shrink fit).
  - (Balans: montajda dengeleme bölgelerinde düzeltme — `lightening_holes` ile de
    isteğe bağlı hafifletme/soğutma deliği eklenebilir.)
- **Toleranslar:** bore H7 (şaft n6/m6 ile sıkılık 0.02–0.06); cep konumu ±0.1°
  kutuplar arası (pos 0.10 MMC, Datum B); dış köprü ±0.05; OD runout ≤0.02 (A);
  balans ISO 21940-11 G2.5 (hedef G1.0).

## 3. Rotor mıknatısları  (`MAGNET_nnn`)

- **İşlev:** uyarma akısı (PM); IPM saliency'si relüktans torku ekler.
- **Geometri:** kutup başına tek V = 2 blok; her blok 26 × 4.5 mm; eksende **4
  segment** + ~0.1 mm yalıtım/yapıştırıcı boşluğu (eddy kaybı). Toplam 6×2×4 = 48 segment.
- **Malzeme:** sinter NdFeB **N42SH** (Br ~1.28 T, ≤150 °C); kaplama epoksi/Ni-Cu-Ni.
- **Üretim:** tedarikçi (sinter + taşlama, ısmarlama). **Kritik:** mıknatıslanmamış
  yapıştır → montajlı rotorda yerinde mıknatısla (back-EMF/yüzey-akı ile doğrula).
- **Montaj/üretim özellikleri:** cep içinde 0.10–0.25 mm yapışkan/tolerans boşluğu;
  eksenel uç-halkalarla tutma (donanım listesi). Mıknatısta delik AÇILMAZ.
- **Toleranslar:** cep genişliği +0.05/0; cep konum saçılımı doğrudan cogging/ripple.

## 4. Stator hairpin sargısı  (`COIL_nnn`)

- **İşlev:** stator akımını taşır; döner alanı kurar.
- **Geometri:** oluk başına 8 dikdörtgen bar (radyal yığılı) × 54 oluk = 432 bar;
  bar köşe yarıçapı 0.8 mm, duvar boşluğu 0.45 mm; her uçta uç-sargı zarfı (22 mm).
- **Malzeme:** emaye bakır mıknatıs teli, H sınıfı (180 °C); 800 V için Kapton MT+ astar.
- **Üretim:** tel-form (düzleştir/kes/soy/U-büküm); oluk astarı → yerleştir → bük →
  lazer/TIG kaynak (çift-çift taç); emprenye (VPI, Sınıf H); IR/hi-pot/surge testi.
- **Montaj/üretim özellikleri:** oluk astarı (Nomex-Kapton-Nomex); faz/uç-tur kâğıdı;
  terminal/busbar seti (gövde terminal geçişinden çıkar). Delik özelliği yok (tel işleme).
- **Toleranslar:** oluk gövdesi/hairpin fit +0.05/0; bar başına boşluk 0.40–0.50 mm.

## 5. Şaft  (`SHAFT`)

- **İşlev:** torku iletir; rotoru taşır; rulmanlarda döner; içi boş → rotor soğutma yağı.
- **Geometri:** ana journal 45 mm, rulman yatakları 40 mm (k5), içi boş bore 14 mm,
  her uçta 35 mm overhang, rulman yatağı boyu 22 mm; **DE çıkış stub'ı Ø32 × 45 mm**
  (rulmanın ötesine uzanır, kamayı taşır).
- **Malzeme:** alaşımlı çelik 42CrMo4 / 4140; journal'lar sertleştirilir + taşlanır.
- **Üretim:** **CNC tornalama** (+ kama/segman frezeleme) → **NX CAM (turning)** uygun.
- **Montaj/üretim özellikleri:**
  - **Çıkış stub'ı (DE)** — rulman yatağından çapça aşağı basamaklı (Ø32) uzantı;
    kaplin/dişli buraya oturur. Kapatmak için `shaft.drive_stub_length = 0`.
  - **Tahrik-ucu kama yuvası** — 12 mm geniş, L≤45 mm (DIN 6885-A); stub varsa **stub'a**,
    yoksa DE rulman-yatağı journal'ına yerleşir (çap basamağını geçmeyecek şekilde clamp'li);
    çıkış kaplini/dişlisine tork.
  - **Segman (retaining-ring) kanalı** — DE rulman yatağının iç tarafında (DIN 471);
    rulman iç bileziğini eksende konumlar.
  - **Radyal yağ çapraz-delikleri** — paket ortasında 4 × Ø4 (içi boş bore → journal
    yüzeyi); rotor soğutma yağı.
- **Toleranslar:** yatak k5 (+0.013/+0.002), Ra ≤0.4 µm taşlama, silindiriklik
  0.004–0.006, iki-journal eş-eksenlilik 0.01 (Datum A-B); kama yuvası genişlik N9,
  simetri 0.02; segman kanalı dia/­genişlik DIN 471.

## 6. Gövde / su soğutma ceketi  (`HOUSING`)

- **İşlev:** statoru taşır (shrink fit, birincil ısı yolu); su ceketiyle soğutur;
  rulmanları/uç-kalkanları taşır; motoru şanzımana monte eder.
- **Geometri:** ceket iç R≈113, dış R≈119 (kalınlık 6 mm); 12 eksenel soğutma kanalı
  (Ø4); her uçta montaj flanşı (kalınlık 12 mm, OD ≈ Ø298 = ceket OD + 2×30 mm lip).
- **Malzeme:** döküm alüminyum (HPDC ADC12/A380), entegre su ceketi.
- **Üretim:** döküm + **CNC finiş frezeleme** (yatak yuvaları, yüzeyler, civata
  delikleri) → **NX CAM (milling)** uygun. Ceket %100 sızdırmazlık testi.
- **Montaj/üretim özellikleri:**
  - **Montaj flanşı (DE + NDE)** — dışa taşan disk lip; uç-kalkan/şanzıman arayüzü.
  - **Şanzıman montaj civata dairesi (DE)** — 8 × Ø11 geçiş deliği (M10), pitch R≈140.6.
  - **Uç-kalkan / rulman-kapağı civata dairesi (her iki uç)** — 8 × Ø7 (M6), pitch R≈128.
  - **Radyal soğutucu giriş + çıkış portları** — 2 × Ø12 (ceket bandına).
  - **Radyal terminal / kablo geçişi** — 1 × Ø28 (IP-keçeli busbar/kablo çıkışı) +
    çevresinde **kabartılmış döküm boss** (8 mm; terminal kutusu/glandı buraya cıvatalanır).
  - **Radyal kaldırma deliği (üst)** — 1 × Ø11 dişli (M10 eyebolt, DIN 580).
- **Toleranslar:** flanş register/pilot h7/H7 (şanzımana), register↔bore eş-merkezlilik
  0.05, flanş yüzü diklik 0.05 (A); civata dairesi konum 0.3 MMC; ceket basınç testi.

## 7. End-shield'ler / rulman kapakları (`ENDSHIELD`, DE + NDE)

- **İşlev:** gövde flanşlarına cıvatalanan döküm kapaklar; rulmanları stator bore'una
  **eş-merkez** taşır, keçeleri tutar, gövde bore'unu kapatır.
- **Geometri:** halka plaka (kalınlık 14 mm), dış = flanş OD (Ø298), merkezde **rulman-OD
  bore'u** Ø80 (40 mm-bore rulmanın dış bileziği); gövde uç-kalkan civata dairesiyle (8 × Ø7)
  **hizalı** delikler. Şaft çıkış stub'ı (Ø32) DE end-shield bore'undan (Ø80) geçer.
- **Malzeme:** döküm alüminyum (gövde ile aynı sınıf), CNC finiş (rulman yuvası).
- **Üretim:** döküm + CNC frezeleme (rulman yuvası h-toleransı, register).
- **Montaj/üretim özellikleri:** uç-kalkan civata deseni (gövdeyle ortak), merkezi rulman
  yuvası, keçe oturma yüzü, gövde register'ına pilot. (`assembly.endshield_enabled` ile kapatılır.)
- **Toleranslar:** rulman yuvası H6/J6 (sabit/serbest taraf), yuva↔gövde-register
  eş-eksenlilik 0.02 (Datum A); civata deseni gövdeyle ortak konum.

---

## Bu motorla mate olan tedarik parçaları (donanım listesi)

3D modelde **arayüzleri** modellenir (journal'lar, civata daireleri, register'lar);
end-shield'ler artık **ayrı gövde olarak modellenir** (§7) ama kapakların makine
parçaları (rulman/keçe) tedarik edilir — `python -m motor_nx.cli hardware`:

rulmanlar (yalıtımlı/hibrit-seramik) · keçeler (FKM/HNBR) · civatalar (ISO 4762) ·
şaft kaması (DIN 6885) · segman (DIN 471) · soğutucu rakorları · terminal/kablo glandı ·
kaldırma cıvatası (DIN 580) · sıcaklık sensörleri (PT100/NTC) · konum sensörü (resolver/encoder).

> Tüm imalat paketini tek komutla al: `python -m motor_nx.cli package -o manufacturing/`
> → BOM + donanım + tolerans CSV'leri, DFM Monte Carlo raporu, Markdown özet ve 5 çizim
> (montaj/stator/rotor/şaft/**patlatılmış montaj**). DFM yetkinliği: `cli dfm`.
