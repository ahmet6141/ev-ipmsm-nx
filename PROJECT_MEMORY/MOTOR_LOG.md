# MOTOR PROJESİ — yapılanların kaydı

**Proje:** Siemens NX'te parametrik **EV traction IPMSM** üreteci (Tesla Model 3 arka
tahrik sınıfı). 54 oluk / 6 kutup, OD 225 mm, aktif paket 134 mm. GitHub repo:
`github.com/ahmet6141/ev-ipmsm-nx` (private).

**Mimari (`motor_nx/` paketi):**
- `params.py` — `MotorParams` + alt gruplar (Stator/Winding/Rotor/Shaft/Cooling/**Material**).
- `em_design.py` — türetilmiş geometri, `validate()`, `report()`, **`performance_report()`**.
- `blueprint.py` — saf-matematik geometri → `BuildStep` listesi (tube/cylinder/extrude/revolve).
- `nx_builder.py` — NX içinde (NXOpen) çalışır, build step'leri uygular. **Tek NXOpen modülü.**
- `nx_smoketest.py` — build'den önce NX 2506 API'lerini sınayan smoke test.
- `fea.py` — FEA hand-off paketi üreteci (yeni).
- `cli.py`, `preview.py`, `batch_build.py`.

**Build yöntemi:** NX → "play journal" → `nx_builder.py` → arg = çıktı `.prt` → Run.
`_default_blueprint` her koşuda `motor_nx` modüllerini sys.modules'tan düşürür → düzenlemeler
NX yeniden başlatılmadan yansır. Builder varsayılan build'de tasarım+performans raporunu ve
FEA paketini de loglar/üretir.

---

## Adım 0 — Hata ayıklama (smoke test'i geçirmek)
- **Kök hata (int/double):** NX 2506 NXOpen, `Point3d`/`Vector3d` ve yay yarıçap/açısında
  **float** istiyor; scriptler int geçiyordu → tüm extrude/revolve patlıyordu.
  **Düzeltme:** `_p3`/`_v3` float-zorlayıcı yardımcılar; tüm çağrılar + yarıçaplar float'a alındı
  (`nx_smoketest.py` + `nx_builder.py`).
- **Slot fixture:** smoke test dikdörtgeni tüp yüzeyine teğet → "zero wall thickness" → [25,40]→[24,41].
- **Sahte "STEP empty" verdisi:** NX'in STEP/Parasolid çevirmeni journal bittikten SONRA yazıyor;
  smoke test dosyayı erken kontrol ediyordu → verdiyi çekirdek adımların hatasızlığına bağladım.
- Sonuç: **SMOKE TEST PASSED**; tam motor **448 katı gövde** kuruldu.

## Adım 1 — cooling_channels düzeltmesi
- Kanal Ø6 mm = jacket yarı-kalınlığı (3 mm) → iki yüzeye teğet → sıfır et. **Ø6→Ø4** (1 mm duvar).
- `validate()`'e cooling guard (radyal duvar + çevresel çakışma). Builder'a modül-tazeleme.
- Sonuç: tam motor **0 hata**.

## Adım 2 — Analitik performans
- `em_design.estimate_performance/performance_report` eklendi (dokümante varsayımlarla).
- **Varsayılan tasarım:** tork 218/473 Nm (sürekli/tepe), güç 99/213 kW, faz akımı 130/282 A,
  elektrik yükleme 56/120 kA/m, taban hız 4307 rpm, geri-EMK 81 V(LL)/krpm, bakır kaybı 1.84 kW.
  → Model 3 RDU ile çok uyumlu. CLI `report` + builder log'una bağlandı.

## Adım 3 — Geometri tamlığı (hepsi build ile doğrulandı)
- **3a Malzeme specs** (`MaterialParams`): mıknatıs **N42SH** (Br 1.28T, ≤150°C), lamine
  **0.27 mm M250-27 sınıfı** (paketleme 0.96), iletken **H sınıfı** yalıtım. `report()`'a eklendi.
- **3b Mıknatıs eksenel segmentasyon:** varsayılan **4 segment** (eddy kaybı ~16× ↓), 0.2 mm yalıtım
  boşluğu. `validate()` guard'ı. → **484 gövde**.
- **3c Profil filetolar:** `_round_corner`/`_round_polygon` (yerleşmeyen yarıçapta köşeyi keskin
  bırakır → güvenli). Slot dibi (`slot_bottom_fillet`) + mıknatıs cebi köşeleri (`magnet_pocket_fillet`).
- **3d Uç-sargılar:** `end_winding_style="envelope"` (varsayılan, toroidal halka) → **486 gövde**.
  - **Hairpin (B) deneyi (ayrı parça, "hairpin" adıyla):** builder'a *konumlu kısmi revolve*
    (`start_angle_deg`/`angle_deg`) eklendi. Evrim: düz yay → **U-şekli** (riser+crown) →
    **eğimli tepe (sloped apex)**, 7 facet'lik çatı. → **1348 gövde, 0 hata**.
- **Adım 3 denetimi + düzeltmeler** (ayrı inceleme ajanıyla):
  - 🐞 `validate()` iletken-sığma formülü `conductor_polygons` ile uyumsuzdu (sessiz 0-bar riski) → hizalandı.
  - 🔧 Kullanılmayan `bar_corner_radius` iletken barlarına uygulandı (yuvarlatılmış köşeler).
  - 🔧 `channel_type="spiral"` sessizce kanal üretmiyordu → `validate()` uyarıyor.
  - 📝 Segmentli mıknatısların `stack_length` ile sürülmediği dokümante edildi (fiziksel doğru).
  - ✓ Denetlendi-doğru: fileto matematiği, köşe indeksleri, segment z'leri, revolve round-trip, geriye-uyum.

## Adım 4 — FEA hazırlığı
- **`motor_nx/fea.py`:** `winding_layout` (slot→faz kuşakları A+/C−/B+/A−/C+/B−), `fea_spec`
  (BH eğrisi, Br/Hcj/sıcaklık katsayıları, 1-kutup anti-periyodik BC, uyarım, çalışma noktaları,
  mesh, analiz matrisi, kabul hedefleri), `to_dxf` (malzeme-katmanlı 2D kesit), `write_package`.
- **`docs/FEA_PREP.md`:** EM+termal+yapısal doğrulama planı + kabul kriterleri + araç önerileri.
- **`fea/` üretildi + doğrulandı:** `fea_spec.json`, `cross_section.dxf`, `winding.csv`.
- CLI `fea` komutu; builder varsayılan build'de `fea/`'yı otomatik üretir.

## Adım 5 — İmalat hazırlığı (2026-06-15)
- **`motor_nx/manufacturing.py`:** modelden türetilen **BOM** (build step hacimleri →
  extrude=alan×boy, tube=halka, revolve=Pappus; kesimler hedef gövdeden net'lenir;
  hacim×yoğunluk×paketleme faktörü → kütle). Varsayılan: **toplam ~43.5 kg** (aktif 39.6),
  48 NdFeB segment 1.4 kg, 432 bar+uç-tur 6.4 kg bakır, ~496 lamine yaprak.
- **GD&T / kritik ölçü tolerans şeması** (12 özellik + datum + gerekçe) + **genel notlar**.
  Değerler web ile temellendirildi + **düşmanca incelendi** (6 tutarsızlık düzeltildi:
  eksantriklik bütçesi 0.07 mm RSS, mıknatıs cebi boşluğu gerçekçi 0.10-0.25, balans
  G2.5→hedef G1.0, rulman seat-bazlı fit, shrink sonrası bore yuvarlaklığı, paket-boyu türetimi).
- CLI **`bom`** + **`tolerances`** komutları (+ `--csv`). `tests/test_manufacturing.py` **7/7**.
- **`docs/MANUFACTURING.md`:** BOM + tolerans tablosu + tam imalat süreci (lamine/mıknatıs/
  hairpin/gövde) + 16 adımlı montaj sırası + EOL test + kaynaklar.
- Commit'lendi + push edildi (`ev-ipmsm-nx`).

---

## Dosya/çıktı durumu
- Builder çıktısı: `<ad>.prt` + `<ad>_ap242.stp` (STEP) + `<ad>.x_t` (Parasolid).
- Üretilen test parçaları (TEMİZLİK BEKLİYOR): `motor_v2..v8.*`, `motor_hairpin*.*`, `nx_smoke*.*`,
  ve geçici journal'lar `_perf.py`, `_fea.py`.
- Kalıcı: `motor_nx/`, `docs/FEA_PREP.md` + `DESIGN.md` + `NX_AUTOMATION.md`, `fea/`, `PROJECT_MEMORY/`.

## Açık konular / dürüst sınırlar
- Performans **analitik** (±%20-30); kesin değerler **FEA**'dan (Adım 4 paketi hazır).
- Hairpin uç-sargı **per-slot zarf/U** seviyesinde; per-iletken (864 bar) solid + kesin sargı şeması
  endüstride **özel araç (Motor-CAD) + FEA** işi.
- `configs/` boş; `batch_build.py` `configs/default.json` bekliyor (sweep için eklenebilir).
