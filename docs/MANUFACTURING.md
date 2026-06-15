# İmalat hazırlığı (Adım 5)

EV traksiyon IPMSM'i (54 oluk / 6 kutup, OD 225 / paket 134 mm, tek-V N42SH, hairpin)
için imalat hand-off dokümanı: **Malzeme Listesi (BOM)**, **GD&T / kritik ölçü
toleransları** ve **imalat süreç notları**. Tolerans ve süreç değerleri EV traksiyon
üretim pratiğinden web ile temellendirildi ve düşmanca bir incelemeden geçirildi
(kaynaklar dosya sonunda).

BOM ve tolerans tabloları modelden türetilir — parametre değişince güncellenir:

```bash
python -m motor_nx.cli bom                 # kütle + adet
python -m motor_nx.cli bom --csv bom.csv
python -m motor_nx.cli tolerances --csv tol.csv
```

---

## 1. Malzeme Listesi (BOM)

Kütle/adet doğrudan model geometrisinden hesaplanır ([motor_nx/manufacturing.py](../motor_nx/manufacturing.py)):
her build step'in hacmi integre edilir (extrude=alan×boy, tube=halka, revolve=Pappus),
kesimler hedef gövdeden net'lenir, hacim×yoğunluk→kütle. Varsayılan tasarım için:

| Bileşen | Malzeme | Adet | ~Kütle |
|---|---|---|---|
| Rotor lamine paketi | NO silisli çelik 0.27 mm (M250-27) | ~496 yaprak | 16.5 kg |
| Stator lamine paketi | NO silisli çelik 0.27 mm (M250-27) | ~496 yaprak | 15.3 kg |
| Stator sargısı — uç turlar | Bakır mıknatıs teli (H sınıfı) | 2 | 4.0 kg¹ |
| Stator sargısı — oluk barları | Bakır mıknatıs teli (H sınıfı) | 432 | 2.4 kg |
| Şaft | Alaşımlı çelik (42CrMo4/4140), içi boş | 1 | 2.2 kg |
| Gövde / soğutma ceketi | Döküm Al (su ceketi) | 1 | 1.7 kg |
| Rotor mıknatısları | Sinter NdFeB N42SH | 48 segment | 1.4 kg |
| **Toplam (modellenen)** | | | **~43.5 kg** |

¹ Uç-tur kütlesi *katı zarf* hacminden (üst-sınır); gerçek uç-turlar kısmen havadır.
Aktif malzeme kütlesi ~39.6 kg; bakır 6.4 kg; NdFeB 1.4 kg. (Emprenye, bağlantı
elemanları, sensörler, konektörler hariç.)

### Tam BOM kalem listesi (üretim için)
Lamine, mıknatıs, hairpin bar, şaft, gövde, rulman dışında üretimde gereken kalemler:
backlack/bonding yapıştırıcı, mıknatıs yapıştırıcısı (150-180 °C epoksi), oluk astarı
(Nomex-Kapton-Nomex), faz/uç-tur yalıtım kağıdı, emprenye verniği (Sınıf H), terminal/
busbar seti, rotor uç/balans halkaları, dış ceket kovanı + kapaklar, rulmanlar
(yalıtımlı/hibrit-seramik), şaft+ceket keçeleri (FKM/HNBR), sıcaklık sensörleri
(PT100/NTC), konum sensörü (resolver/encoder), bağlantı elemanları.

---

## 2. Kritik ölçü / GD&T şeması

Datum **A** = rulman-yatağı ekseni (rotor); stator özellikleri gövde register'ına
referanslı. `python -m motor_nx.cli tolerances` ile tam tablo.

| Özellik | Nominal | Tolerans / fit | GD&T | Datum |
|---|---|---|---|---|
| Hava aralığı (radyal) | 0.70 mm | doğrudan ölçülmez; montaj eksantrikliği **≤0.07 mm** (RSS) | yığılımla yönetilir | A |
| Stator bore | 161.0 mm | IT7 / H7 (+0.040/0) | silindiriklik 0.015; runout 0.02-0.03 TIR | A |
| Stator OD (Al ceket) | 225.0 mm | h6/js6 shrink; sıkılık 0.05-0.10 mm | OD↔bore eşmerkezlilik 0.05; **shrink sonrası** bore yuvarlaklığı | A |
| Rotor OD | 159.6 mm | −0.02/−0.04 (aralık açık kalsın) | runout A'ya ≤0.02; yataklarda tek bağlamada finiş | A |
| Mıknatıs cebi genişliği | 4.80 mm | +0.05/0; ±0.05 taşlı mıknatısla gerçek boşluk **0.10-0.25** | duvar profili 0.05 | B |
| Mıknatıs cebi konumu | 60° kutup adımı | ±0.1° kutuplar arası; radyal ±0.05 | konum 0.10 MMC; V-simetri 0.05 | B |
| Dış köprü kalınlığı | 1.0 mm | ±0.05 (kalıp ±0.02) | web profili 0.05; köprüler arası 0.04 | B |
| Oluk ağzı genişliği | 1.9 mm | ±0.02; diş 5.6 ±0.03 | oluk deseni konum 0.05; ağız profili 0.04 | A |
| Oluk gövdesi / hairpin fit | 3.88 mm | +0.05/0; bar başına boşluk 0.40-0.50 | duvar paralelliği 0.03 | A |
| Rulman yatakları (DE/NDE) | 40.0 mm | k5 (+0.013/+0.002); gövde: serbest H6, sabit J6/K6; Ra≤0.4 µm | silindiriklik 0.004-0.006; eş-eksenlilik 0.01 | A-B |
| Rotor oturma yüzeyi (şaft) | 45.0 mm | şaft n6/m6 ↔ paket bore H7; sıkılık 0.02-0.06 | eş-eksenlilik A-B'ye 0.01 | A-B |
| Lamine paket uzunluğu | 134 mm | ±0.30 (√N×yaprak saçılımı) | uç yüz diklik 0.05; paralellik 0.05 | A |
| Rotor balansı | iki düzlem dinamik | ISO 21940-11 **G2.5 azami; hedef G1.0** (NVH) | balans bölgelerinde düzeltme, ~1 g-mm | A-B |

**Genel notlar:** Datum stratejisi (yatak ekseni birincil), eksantriklik yığılım
bütçesi (RSS ≤0.07 mm), lamine çapak <0.02 mm (IEC 60404; aksi halde demir kaybı
%15-20 ↑), yüzey finişleri, GD&T çerçevesi (ISO 1101 + ISO 286 + ISO 2768-mK),
hat-sonu testi — tamamı `manufacturing.general_notes()` içinde.

---

## 3. İmalat süreci

### Lamine
NO silisli çelik 0.25-0.27 mm (M250-35A / NO20 sınıfı) — 18 krpm / 6 kutupta f_e ~900 Hz
için düşük demir kaybı. **İlerlemeli kalıp** (progressive-die) sertmetal takımla seri
üretim; lazer kesim yalnız prototip (kesim kenarı manyetik özelliği bozar). Çapak <15-25 µm.
**Paketleme:** stator için **backlack** (öz-yapışkanlı, ısı+basınçla kürlenen epoksi
kaplama) — interlaminer kısa devre yok, ses düşük, kaynak-kaynaklı eddy yolu yok (premium
traksiyon tercihi). Rotor: backlack veya perçin/kilit; mıknatıs köprülerinde delici kaynak
yapma. Paketleme faktörü ≥0.96-0.97. Gerilme-giderme tavlaması yalnız bağlamadan **önce**
gevşek yapraklara. Bore (161) / OD (225) ve rotor OD (159.6) bağlama sonrası finiş işlenir.

### Mıknatıs (N42SH)
Sinter NdFeB **N42SH** (Br ~1.28-1.32 T, yüksek Hcj, ~150 °C sürekli); ağır-nadir-toprağı
azaltmak için **GBD (sınır-difüzyonlu)** varyant düşünülebilir. **Eksenel segmentasyon:**
kutup başına 4 segment + ~0.05-0.1 mm yalıtım/yapıştırıcı boşluğu — eksenel eddy döngüsünü
kırar, PWM/oluk harmoniği kayıplarını düşürür. Kaplama: epoksi veya Ni-Cu-Ni. **Kritik:**
mıknatısları **mıknatıslanmamış (yeşil)** olarak V ceplerine yapıştır, sonra montajlı
rotorda **yerinde mıknatısla** (pulse fikstür) — kırılgan mıknatıslı blokların çekim/
çatlama/FOD/güvenlik tehlikesini önler. Yüksek-sıcaklık yapısal epoksi (150-180 °C). Br
sıcaklık katsayısı ~−0.11..−0.12 %/°C (Kr derating). Doğrulama: back-EMF / yüzey-akı haritası.

### Sargı (hairpin)
Dikdörtgen emaye Cu bar, **oluk başına 8 bar × 54 oluk**, doluluk >%70. **Form:** düzleştir →
kes → uç soy → **U (hairpin)** büküm (katman-bazlı pin şekli). **Oluk astarı:** S-katlı
Nomex-Kapton-Nomex (800 V için Kapton MT+) — pin yerleşiminden önce. **Yerleştir → bük (twist)**
kaynak tarafında bitişik bacakları eşleştir → **lazer/TIG kaynak** çift-çift taç uçları;
%100 kaynak muayenesi. **Emprenye:** trickle/VPI epoksi-poliester vernik (Sınıf H, 180 °C)
→ kürle. **Test:** IR, hi-pot (faz-toprak/faz-faz), tur-tur surge, faz direnç dengesi.

### Gövde / soğutma
**HPDC alüminyum** (ADC12/A380) entegre su ceketiyle. İki yapı: (a) tek-parça döküm
spiral/eksenel kanal, veya (b) "makara + dış kovan" (spiral-kaburgalı stator taşıyıcı dış
ceket içine shrink — Tesla tarzı). Ceket %100 sızdırmazlık/basınç testi. **Stator→gövde:**
sıkı (shrink) geçme — gövdeyi ısıt/statoru soğut, bırak kilitlensin (birincil ısı yolu da).
Rulman yataklarını stator bore'a eş-merkez finiş işle (0.70 mm aralığı tutmak için).
**Rulmanlar:** 18 krpm; en az bir tarafta yalıtımlı/hibrit-seramik (PWM ortak-mod EDM
yatak akımını engelle). Keçeler FKM/HNBR; içi boş 45 mm şaft rotor soğutma yağı taşıyabilir.

---

## 4. Montaj sırası
1. Giriş muayenesi (lamine/mıknatıs/hairpin/döküm; çelik & mıknatıs sınıfı, emaye).
2. Stator paketini bas/bağla (backlack kür) 134 mm; bore 161 + OD 225 finiş işle.
3. Rotor paketini bas/bağla; rotor OD 159.6 finiş; içi boş şaft bore finiş.
4. Oluk astarı; hairpin form/yerleştir/bük/lazer-kaynak; kaynak muayene.
5. Emprenye/vernik + kür; **sarılı statorda IR + hi-pot + surge** (değer eklemeden önce ele).
6. Mıknatıslanmamış N42SH segmentlerini (4 eksenel) rotor V ceplerine yapıştır; kürle.
7. Rotor lamine + tutucuları içi boş 45 mm şafta bas/shrink; uç halkaları tak.
8. Montajlı rotoru pulse fikstürde **yerinde mıknatısla**; back-EMF / yüzey-akı doğrula.
9. Rotor-şaft setini **iki düzlem dinamik balans** (G2.5/G1.0); yeniden doğrula.
10. Gövde su ceketi sızdırmazlık testi; ceket alt-parçalarını keçelerle birleştir.
11. Sarılı statoru gövdeye **shrink** (gövdeyi ısıt, yerleştir, soğut).
12. Rulmanları (yalıtımlı/hibrit) gövde + uç kalkanına tak.
13. Rotoru stator bore'a **0.70 mm aralığı koruyarak** yerleştir; uç kalkan, ön-yük, keçe.
14. Sargı terminallerini busbar'a bağla/lehimle; sıcaklık sensörü + resolver yönlendir.
15. Soğutucu doldur/kontrol; nihai tork + emniyetleme.
16. **Hat-sonu** elektrik/performans/akustik (NVH) test; serileştir + etiketle.

---

## 5. Kaynaklar (araştırma workflow'undan)
- MDPI Energies 17(8):1913 — *Influence of Motor Manufacturing Tolerances on EOL Testing*
- Nature SR 41598-024-68632-z / PMC11283456 — PMSM EV hava aralığı eksantrikliği (<%10)
- ISO 1940-1 / ISO 21940-11 (balans G2.5); SKF/EngineersEdge (k5/m5 yatak fitleri, ISO 286)
- Hexagon / lamnow / Precision Micro / emobility-engineering — lamine metroloji & stamping ±0.02
- Arnold Magnetics / Stanford Magnets — NdFeB yüksek-sıcaklık & sıcaklık katsayıları
- Charged EVs / Laserax / IEEE 8863004 / electricmotorengineering — hairpin & oluk astarı (800 V)
- UTS / USPTO patentleri — mıknatıs segmentasyonu ile eddy kayıp azaltma
- empcasting — EV motor gövdesi (HPDC Al, su ceketi, sıfır-hata)

> Bu şema bir **başlangıç imalat paketidir**; nihai resimler, kalıp tasarımı ve süreç
> validasyonu (özellikle dış köprü mekanik FEA'sı ve shrink sonrası bore distorsiyonu)
> üretim mühendisliği + tedarikçi onayı gerektirir.
