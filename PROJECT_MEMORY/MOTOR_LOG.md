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

## Derin denetim & iyileştirme (2026-06-15)
6-boyutlu çok-ajanlı denetim (53 ajan): **47 bulgu, 47'si doğrulandı** → 41 düzeltildi,
6 ertelendi/kabul. Detay+çözüm: `docs/AUDIT.md`. Commit'ler C1–C6:
- **C1 fizik/doğrulama** (em_design): taban-hız tavanı Vdc/√6 (SVPWM) düzeltildi
  (4307→3480 rpm); validate() boşlukları (d-ekseni nervür erozyonu, şaft borusu,
  sıfır iletken/yol, tangansiyel bar, negatif boşluk, ters ceket, yuvarlak-cep içi mıknatıs).
- **C2 tutarlılık**: Hcb Br/μ_recoil'den türetilir (915→970); slot_fill net 0.60;
  segment boşluğu 0.1; M250-27 grade; NX birimleri (count/mm/deg); BOM mass_by_material.
- **C3 kod-kalite**: `blueprint.iter_cross_section` (3 modüldeki kopya kesit projeksiyonu
  birleşti); gerçek `coil_span_slots`→kp; obfuske mıknatıs-anchor temizlendi.
- **C4 eksik analizler — `motor_nx/analysis.py`**: kayıp dökümü (demir/AC-bakır/mıknatıs-eddy),
  **termal sürekli-anma** (kayıp→sıcaklık, J_cont≈9.8 termal-sınırlı), demag marjı, rotor
  santrifüj gerilmesi (SF), cogging indeksi; BOM **maliyet** (mıknatıs payı %43), hesaplanan
  **eksantriklik yığılımı**; CLI `analysis`.
- **C5 testler**: test_analysis/em_design/fea → toplam **43/43**.
- **C6**: doküman uzlaştırma + `docs/AUDIT.md` + bu kayıt.

## Doğrulama sürücüleri & model rafine (2026-06-15)
PROJECT_PLAN'ın yürütme fazlarını fiilen koşturacak **çalıştırılabilir betikler** + analitik
modelin rafine edilmesi. (Kullanıcı seçimi: FEA aracı = **Ansys Motor-CAD**; ayrıca performans
rafine + NX Drafting + configs doğrula.)
- **`verification/motorcad_emag.py` — P1 EM-FEA sürücüsü (PyMotorCAD, ansys-motorcad-core).**
  `fea/fea_spec.json`'u Motor-CAD değişkenlerine eşler (geometri/sargı/malzeme), `fea_spec.analyses`
  matrisini koşar: cogging, geri-EMK (THD), tork-açı **MTPA** süpürmesi, ripple, demag (sıcak 150°C),
  + P2/P3 tohumları (termal steady-state, 1.2× aşırı-hız rotor gerilmesi). Sonuçları
  `fea_spec.acceptance_targets`'a karşı **geç/kal** olarak puanlar → `fea/motorcad_results.json`.
  API güncel sürümle doğrulandı (v0.8.6: `MotorCAD()`, `set_variable`, `show_magnetic_context`,
  `do_magnetic_calculation`, `get_magnetic_graph[_point/_harmonics]`, `set_winding_coil`,
  `do_mechanical_calculation`, `save_to_file`). Değişken-adı kayması için her set/get loglu +
  hedef-değer yazdırır (V-cep adları sürüme göre değişir → kullanıcı onaylar).
- **`verification/nx_drafting.py` — P6 resmi NX Drafting journal'ı (NXOpen, run_journal).**
  Drafting'e geçer, A3 sayfa + FRONT/TFR-ISO görünüm; **antet + GD&T/kritik-ölçü şeması +
  genel notlar + BOM** (hepsi canlı `manufacturing`/`em_design` verisinden) native NX notu olarak.
  `dxf` argümanı: `drawings.py`'nin tam-ölçülü DXF sayfalarını (montaj/stator/rotor) NX'e import eder.
  nx_builder tarzı: her adım guard'lı + Listing Window'a loglu.
- **`verification/README.md`** — hand-off zinciri (cli fea → fea/ → sürücüler), çalıştırma, uyarılar.
- **Performans modeli rafine (`em_design.py`):** `b_g1_peak_t` artık varsayılan **None → Br'den türetilir**.
  Yeni `airgap_flux_density()` (PM manyetik devresi: A_m/A_g, recoil, Carter `g_eff`, kaçak `k_leak`)
  + `carter_factor()`. **Etkin manyetik paket** `L_eff = k_stack·L` flux/torka uygulandı.
  Kalibrasyon (k_leak 1.05, pole-arc 0.85) referans tasarımda eski ~0.85 T'yi yeniden üretir,
  ama artık Br/mıknatıs-genişliği/kalınlık/hava-aralığı/kutup ile fiziksel ölçeklenir.
  Net etki (varsayılan): tepe tork **473→~440 Nm** (k_stack=0.96 yükü), taban hız ~3480→**~3621 rpm**.
  6 yeni test (test_em_design.py).
- **`fea.py` zenginleştirme:** `geometry_mm`'e V-cep tanımı eklendi (magnet_thickness/width,
  v_angle, magnets_per_pole, vertex_gap, end_barrier) → Motor-CAD/FEMM tam geometri alır.
- **`tests/test_configs.py` (yeni):** configs/*.json yükle → expand_variants → validate + blueprint
  üret (batch_build --dry-run yolunu kilitler). default temiz, sweep 9+1=10 varyant.
- **DÜRÜST SINIR:** Sandbox bu oturumda kapalıydı (HYPERVISOR_VIRT_DISABLED) → testler
  **statik incelendi** (base_speed/tork/sınırlar elle izlendi), **koşturulamadı**. Kullanıcı:
  `python tests/test_em_design.py && python tests/test_configs.py` (+ diğerleri) ile doğrulamalı.

---

## Dosya/çıktı durumu
- Builder çıktısı: `<ad>.prt` + `<ad>_ap242.stp` (STEP) + `<ad>.x_t` (Parasolid).
- Üretilen test parçaları (TEMİZLİK BEKLİYOR): `motor_v2..v8.*`, `motor_hairpin*.*`, `nx_smoke*.*`,
  ve geçici journal'lar `_perf.py`, `_fea.py`.
- Kalıcı: `motor_nx/`, `docs/FEA_PREP.md` + `DESIGN.md` + `NX_AUTOMATION.md`, `fea/`, `PROJECT_MEMORY/`.

## Adım 6 — Montaj/üretim özellikleri + klasör düzeni (2026-06-23)
- **`AssemblyParams` (params.py, yeni grup):** her parçaya profesyonel montaj/üretim
  özellikleri — stator tie-rod delikleri + dönmez OD kaması; rotor rivet/uç-plaka
  delikleri + opsiyonel bore kama; şaft DE kama yuvası (DIN 6885) + segman kanalı
  (DIN 471) + radyal yağ delikleri; gövde montaj flanşı (DE+NDE) + şanzıman & uç-kalkan
  civata daireleri + radyal soğutucu portları + terminal geçişi + kaldırma deliği.
  Hepsi parametrik, `assembly.enabled`/sayı/boyut=0 ile kapatılabilir. `expressions()`
  ve `_GROUP_TYPES`'a "assembly" eklendi.
- **`blueprint.py`:** `assembly_steps()` + yeni **`hole`** build-step kind'ı (keyfi
  eksende silindirik kesim → radyal delikler). Flanş = dolu disk-unite + bore-reopen.
- **`em_design.py`:** `_validate_assembly()` — her özellik build'den önce geometrik
  doğrulanır (duvar payı, oluk/mıknatıs/kanal çakışması, civata dairesi flanşa sığar,
  halka binmesi). Varsayılan + 8-kutup varyant TEMİZ.
- **`nx_builder.py`:** `hole` kind handler (`_extrude_on_axis`/`_circle_curve_on_axis`,
  keyfi eksen).
- **GERÇEK NX 2506'da BUILD DOĞRULANDI (2026-06-23):** varsayılan motor TÜM montaj
  özellikleriyle **486 katı gövde, 0 step hatası** kuruldu (tüm `hole`/flanş/civata/
  kama/segman/tie-rod/rivet adımları OK); STEP (_ap242.stp) export başarılı. İki düzeltme:
  - **Parasolid (.x_t) export'u patladı** ("Modeler error: please report fault",
    `_UF.Ps.ExportData`). Kök neden: `export_parasolid` `EntirePart` kapsamı kullanıyordu →
    section'ların dumb construction curve'lerini PK translator'a sürüyordu. **DÜZELTME:**
    STEP gibi yalnız katı gövdeleri seç (SelectedObjects + SelectionComp.Add).
  - **Şaft kama yuvası** çap basamağını geçip yüzey-altı slot oluşturuyordu. **DÜZELTME:**
    kama yuvası DE rulman-yatağı journal'ına clamp'lendi (uzunluk ≤ bearing_seat_length);
    varsayılan 30→18. Her ikisi de saf-Python'da 78 testle geçti, **NX'te re-confirm bekliyor.**
- **`manufacturing.py`:** `hole` hacmi; `hardware_schedule()`/`hardware_report()`
  (civata/kama/segman/rulman/keçe/sensör); montaj GD&T tolerans satırları; flanş BOM'da
  (tek döküm, qty=1). Toplam kütle ~42.7 kg (aralıkta).
- **`drawings.py`:** stator/rotor sheet'lerine montaj callout'ları; **4. sayfa = şaft**
  boyuna kesiti (kama/segman/yağ delikleri). `cli.py`'ye `hardware` komutu.
- **Testler:** `tests/test_assembly.py` (14 test); test_drawings 4 sayfa/8 dosyaya
  güncellendi. **78/78 test GEÇTİ** (.venv Python 3.10 ile koşuldu).
- **Klasör düzeni:** üretilmiş CAD çıktıları → `build/legacy/`; deneysel scratch
  (`_*.py`, `fem2.fem`, `sim1.sim`) → `archive/`. Kök temizlendi. `.gitignore`
  güncellendi (build/* ve /archive/* yoksay, README'ler izlenir). **Yeni klasör
  `project_details/`** — insan+AI için düz-metin proje anlatımı (genel bakış, mimari,
  parça-parça şartname, montaj özellikleri kataloğu).

## Adım 7 — montajı tamamla + imalat çıktılarını cilala (2026-06-23)
- **Şaft çıkış stub'ı:** `ShaftParams.drive_stub_diameter/length` (Ø32×45). shaft_profile
  rulmanın ötesine basamaklı uzantı ekler; kama yuvası artık **stub'a** yerleşir (yoksa DE seat'e).
- **End-shield'ler (DE+NDE) GERÇEK gövde:** `assembly.endshield_*`; rulman-OD bore'lu (Ø80)
  halka kapaklar, gövde uç-kalkan civata dairesiyle hizalı delikler. Yeni BOM grubu + NX
  component (`EndShield`) + body adı (`ENDSHIELD`). Toplam kütle ~47.8 kg.
- **Terminal boss:** terminal Ø28 etrafında kabartılmış döküm pad (radyal `hole` unite).
- **Patlatılmış montaj çizimi (5. sayfa):** bileşenler montaj sırasıyla yarı-kesit + sıra okları.
- **GD&T cilası:** `Drawing.fcf()` (feature-control frame) + `Drawing.datum()`; her sayfaya
  datum (A/B/A-B) + FCF; assembly'ye flanş OD/civata-dairesi/end-shield callout'ları.
- **DFM Monte Carlo:** `manufacturing.eccentricity_monte_carlo` (vektörel runout toplamı,
  random faz, half-normal; over-budget ppm + **Cpk**). Varsayılan Cpk ~3.8 (OK). CLI `dfm` (seeded → deterministik).
- **İmalat paketi:** `write_manufacturing_package` + CLI `package` → tek klasöre BOM+donanım+
  tolerans CSV'leri + DFM raporu + Markdown özet + 5 çizim (15 dosya).
- **Testler:** test_assembly 14→22, test_drawings 5 sayfa/10 dosya. **86/86 GEÇTİ.**
  10/10 sweep varyantı (8-kutup dâhil) temiz. `.gitignore`'a `/manufacturing/`.

## Açık konular / dürüst sınırlar
- Performans **analitik** (±%20-30); kesin değerler **FEA**'dan (Adım 4 paketi hazır).
- Hairpin uç-sargı **per-slot zarf/U** seviyesinde; per-iletken (864 bar) solid + kesin sargı şeması
  endüstride **özel araç (Motor-CAD) + FEA** işi.
- **Montaj özellikleri NX'te DOĞRULANMADI:** geometri+doğrulama+BOM saf-Python'da test edildi
  (78 test), ama yeni `hole` primitifi ve flanş unite/reopen **gerçek NX'te smoke edilmeli**
  (build-step sözdizimi doğru, NXOpen çağrı dizisi sıradaki in-NX koşusunda onaylanmalı).
- `build/legacy/` ve `archive/` gitignore'da (yeniden üretilebilir/scratch); silinebilir.
