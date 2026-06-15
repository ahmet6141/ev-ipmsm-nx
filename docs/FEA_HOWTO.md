# Gerçek FEA'yı nasıl, hangi programla yapacaksın

Bu projedeki `analysis.py` **birinci-mertebe** (±%20-30) analitik tahminler verir.
**Gerçek** doğrulama 2D/3D sonlu-elemanlar (FEA) ister. Bu rehber: hangi programı
seçeceğin, üreteceğimiz paketi (`fea/`) nasıl kullanacağın ve hangi analizleri
koşturup neye karşı kontrol edeceğin.

İhtiyacın olan girdileri `python -m motor_nx.cli fea -o fea/` üretir:
`fea_spec.json` (tam kurulum: malzeme/BH, sınır koşulları, sargı haritası, uyarım,
analiz matrisi, kabul hedefleri), `cross_section.dxf` (malzeme-katmanlı 2D geometri,
FEMM/Maxwell doğrudan içe aktarır), `winding.csv` (oluk→faz) ve `femm_labels.csv`
(FEMM blok-etiket reçetesi: her bölge için iç-nokta + malzeme + devre/tur/işaret +
mıknatıs manyetizasyon yönü).

---

## 1. Hangi program?

| Araç | Tip | Lisans | Bu proje için |
|---|---|---|---|
| **FEMM + pyFEMM** | 2D manyetostatik/harmonik | **Ücretsiz** (Windows) | **Önerilen başlangıç.** Python'dan sürülür; üreticimiz doğrudan betik yazıyor |
| Ansys Maxwell 2D/3D | 2D/3D EM | Ticari | Endüstri standardı; DXF + fea_spec ile kurulur |
| Siemens Simcenter / Motorsolve | motor EM | Ticari | NX ekosistemiyle entegre |
| **Ansys Motor-CAD** | EM + **termal** + mekanik | Ticari | Motor-özel; template + sweep'ler için en hızlı |
| JMAG / MotorXP | motor EM | Ticari / uygun | Doğrulama referansı |
| **Elmer FEM / Agros2D** | çok-fizik | Ücretsiz/açık | Termal + yapısal alternatif |
| Ansys Mechanical | yapısal | Ticari | Rotor santrifüj gerilmesi (köprü/nervür) |

**Pratik yol (ücretsiz):** EM doğrulaması için **FEMM**; termal için bizim
`analysis.thermal_rating` + (gerekirse) Elmer/Fluent; yapısal için Elmer/Mechanical.
**Pratik yol (ticari, en hızlı):** **Motor-CAD** (EM+termal tek araçta).

---

## 2. FEMM (ücretsiz) ile adım adım

1. **Kur:** [femm.info](https://www.femm.info) (FEMM 4.2) + `pip install pyfemm`.
2. **Geometriyi içe aktar:** FEMM → File → Import DXF → `fea/cross_section.dxf`
   (katmanlar: STATOR_STEEL / ROTOR_STEEL / MAGNET / COPPER / SLOT_AIR / POCKET_AIR …).
   Manyetostatik problem (Problem → freq 0, mm, planar, **depth = stack 134 mm**).
3. **Malzeme + blok-etiketleri ata:** `fea/femm_labels.csv` her bölge için tam
   koordinat + atama verir — her satır için o (x,y)'ye bir blok-etiket koy:
   - Çelik (`Steel`, BH eğrisi `fea_spec.materials.lamination`), Hava (`Air`),
   - **Mıknatıs** (`NdFeB`, Br/Hc `fea_spec.materials.magnet`, **magdir** sütunundaki
     açı = manyetizasyon yönü; kutuplar arası N/S alternasyonu reçetede hazır),
   - **İletken** (`Copper`, `circuit` sütunundaki faz A/B/C, `turns`=±1 işaret).
   Devreler: `mi_addcircprop('A'/'B'/'C', I, 1)`. Dış çapa A=0 (Dirichlet) sınır koşulu.
4. **Analizleri koştur** (`fea_spec.json` → `analyses` matrisi). FEMM'de rotoru
   adım adım döndürerek:
   - **cogging:** akımsız, 1 oluk-adımı boyunca ince adımlarla tork(θ).
   - **back-EMF:** açık-devre faz akı-bağı(θ) → türevle EMK; genlik vs Vdc, THD.
   - **Ld/Lq:** id,iq taraması → akı haritaları, çıkıntılık (saliency).
   - **tork-açı (MTPA):** anma & tepe akımda ilerleme açısı taraması.
   - **demag:** tepe q-zıt akım @150 °C; min mıknatıs B'si knee'nin üstünde mi?
   - **kayıplar:** demir (stator/rotor), mıknatıs eddy (segmentasyonla), AC bakır.
5. **Kabul kriterleri:** sonuçları `fea_spec.json` → `acceptance_targets` ile kıyasla
   (tepe tork ≥ %90 hedef, tork dalgalanması ≤ %5, cogging ≤ anma %1, demag yok @150 °C,
   rotor SF ≥ 1.5 @1.2×max hız, tepe verim ≥ %95).

> `analysis.py` çıktıları (J_cont≈9.8 termal-sınırlı, demag worst-case marjı, rotor SF)
> FEA'da **önce kontrol edilecek şüpheli noktalardır** — oradan başla.

---

## 3. Ansys Maxwell / Motor-CAD ile

1. **Geometri:** `cross_section.dxf`'i içe aktar (malzeme-katmanlı: STATOR_STEEL,
   ROTOR_STEEL, MAGNET, COPPER, …) **veya** `fea_spec.json` → `geometry_mm`'den
   parametrik kur (Maxwell RMxprt / Motor-CAD template). Maxwell'de **1 kutup
   anti-periyodik** dilim kullan (`symmetry_and_bc`: sektör 60°, radyal kesim
   yüzleri anti-periyodik master/slave, dış sınır A=0).
2. **Malzeme:** `fea_spec.json` → `materials`: lamine BH eğrisi + demir-kayıp
   katsayıları (datasheet ile değiştir), mıknatıs Br/Hcj/μ_recoil + sıcaklık
   katsayıları, bakır iletkenlik + dolum.
3. **Sargı:** `winding.csv` / `fea_spec.winding.slot_phase_map` ile oluk→faz/işaret;
   `series_turns_per_phase`, `parallel_paths`, `conductors_per_slot`.
4. **Uyarım & çalışma noktaları:** `excitation` (anma/tepe akım, ilerleme açısı
   taraması) + `operating_points` (Vdc, taban/max hız, tork).
5. **Mesh:** `mesh` (hava aralığı 3 radyal katman, köprü/post/diş-uç/mıknatıs-köşe
   incelt). Analiz matrisi + kabul hedefleri yukarıdaki gibi.

---

## 4. Termal & yapısal

- **Termal:** `analysis.thermal_rating` lumped bir başlangıç verir (kazanç→sıcaklık).
  Kesin sürekli anma için Motor-CAD Thermal veya CFD (Fluent) — ceket akış debisi,
  ΔT, h katsayısı + uç-sargı/mıknatıs sıcak nokta. Kayıp girdileri `analysis.loss_breakdown`'dan.
- **Yapısal:** `analysis.rotor_stress` köprü/nervür gerilmesinin birinci-mertebe
  tahminidir. Kesin için rotor lamine kesitinde 2D düzlem-gerilme FEA (Ansys Mechanical
  / Elmer): 1.2×max hızda von Mises < akma / SF, gerilme yoğunlaşması köprü filetolarında.

---

## 5. Akış özeti

```
params  ──cli fea──►  fea/{fea_spec.json, cross_section.dxf, winding.csv}
        ──cli femm─►  femm/run_femm.py  ──FEMM──►  EM sonuçları (tork, EMK, cogging, demag)
        ──cli analysis►  birinci-mertebe kayıp/termal/demag/gerilme (FEA öncesi tarama)
                                       │
                                       ▼
                         acceptance_targets ile kıyasla → tasarımı iterasyonla
```

> Bu paket FEA'yı **kurar ve girdiyi üretir**; çözümü yukarıdaki araçlardan biri yapar.
> `analysis.py` rakamları yön verir, FEA kesinleştirir.
