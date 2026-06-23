# SIRADAKİLER — yapacaklarımız

> **AKTİF YOL HARİTASI:** uçtan uca yürütme planı `docs/PROJECT_PLAN.md`'de
> (12 faz P0–P10, FEA (Motor-CAD) → iterasyon → imalat). **P1 ve P6 sürücüleri artık
> yazıldı → `verification/`:** `motorcad_emag.py` (PyMotorCAD EM-FEA: cogging/geri-EMK/
> MTPA/ripple/demag + termal/mekanik tohum, kabul kapısına karşı puanlar) ve
> `nx_drafting.py` (resmi NX 2D resim). **Sıradaki = bu sürücüleri FİİLEN KOŞTUR**
> (Motor-CAD/NX lisansı sende) + FEMM çapraz-kontrol (femm_labels.csv).

## A) Analog çip projesi  ⟵ KULLANICININ ÖNCELİĞİ (beklemede)
- Kullanıcı, **analog çip projesine** devam etmek istiyor.
- Bu oturumda **henüz başlanmadı**; kullanıcı onayı bekleniyor.
- Başlarken: kapsam/hedef netleştir (hangi çip, hangi araç/akış — ör. şematik, SPICE/ngspice,
  layout/KLayout, vb.). Kullanıcının makinesinde ilgili araçlar kurulu görünüyor
  (ngspice, KLayout, Icarus Verilog, Anaconda...). Önce ne yapılacağını sor.

## B) Motor projesi — yapıldı (bu oturum) ve kalan
- ✅ **P1 EM-FEA sürücüsü** — `verification/motorcad_emag.py` (PyMotorCAD; API v0.8.6 doğrulandı).
- ✅ **P6 NX Drafting journal'ı** — `verification/nx_drafting.py` (antet+GD&T+notlar+BOM; `dxf` import).
- ✅ **Resmi NX Drafting çıktısı** (eski ⬜ madde) — yukarıdaki journal ile karşılandı.
- ✅ **Performans rafine** — `em_design.py`: Bg artık Br'den türetilir (`airgap_flux_density`+`carter_factor`),
  `L_eff=k_stack·L` flux/torka uygulandı; 6 yeni test. (tepe tork 473→~440 Nm, daha tutarlı.)
- ✅ **configs doğrulandı** — `tests/test_configs.py` (default temiz, sweep 10 varyant; dry-run yolu kilitli).

## C) Montaj/üretim özellikleri — yapıldı (Adım 6, 2026-06-23)
- ✅ **Her parçaya montaj delikleri + üretim detayları** (`AssemblyParams`): stator
  tie-rod + OD kama; rotor rivet + opsiyonel bore kama; şaft kama yuvası (DIN 6885) +
  segman kanalı (DIN 471) + yağ delikleri; gövde montaj flanşı + civata daireleri +
  soğutucu portları + terminal + kaldırma deliği. Parametrik + `validate` ile vetlenir.
- ✅ Yeni `hole` (radyal) build-step + `cli hardware` + 4. çizim (şaft) + 14 yeni test.
- ✅ Klasör düzeni: `build/legacy/` + `archive/` + `project_details/` (insan+AI anlatımı).
- ✅ **Montaj özellikleri NX'te BUILD DOĞRULANDI (2026-06-23):** varsayılan motor TÜM
  özelliklerle **486 gövde, 0 step hatası** (tüm `hole`/flanş/civata/kama/segman/rivet OK);
  STEP export OK. Bu, yeni `hole` primitifi + flanş yaklaşımının gerçek NX 2506'da çalıştığını kanıtlar.
- ⬜ **İki düzeltmeyi NX'te RE-CONFIRM ET:** (a) Parasolid `.x_t` export'u EntirePart→bodies-only'a
  alındı (construction-curve fault'u), (b) şaft kama yuvası DE seat'e clamp'lendi. İkisi de
  saf-Python'da geçti; sıradaki `run_journal nx_builder.py` koşusunda `.x_t` üretildiğini doğrula.

**Kalan (çoğu kullanıcı-tarafı):**
1. ⬜ **Sürücüleri FİİLEN KOŞTUR:** `pip install ansys-motorcad-core` → `python verification/motorcad_emag.py`;
   NX'te `run_journal.exe verification\nx_drafting.py -args <motor>.prt A3`. (Lisans/çözüm sende.)
2. ✅ **Testleri koştur:** .venv (Python 3.10) ile **78/78 test GEÇTİ** (test_em_design, test_configs,
   test_assembly + diğerleri). `python tests/test_*.py` ile tekrar doğrulanabilir.
3. ⬜ **FEMM çapraz-kontrol scripti** (ücretsiz; `femm_labels.csv` sürücülü pyFEMM) — sıradaki doğal ek.
4. ✅ **Temizlik:** çıktılar `build/legacy/`'e, scratch (`_*.py`/`fem2.fem`/`sim1.sim`) `archive/`'e
   taşındı (silinmedi). `.gitignore` güncellendi (README'ler izlenir).
5. ⬜ **Commit/push:** `ev-ipmsm-nx` (terminal sende). Yeni: `motor_nx/*` (montaj özellikleri),
   `tests/test_assembly.py`, `project_details/`, `build/README.md`, `archive/README.md`.
6. ⬜ Hairpin: kaynak tarafı + gerçek bağlantı şeması (ya da Motor-CAD'e bırak).

## Notlar
- Sandbox (Linux) bu oturumda kapalıydı (HYPERVISOR_VIRT_DISABLED) — Python'u doğrulamak için
  NX'in gömülü yorumlayıcısı (Play Journal) kullanıldı.
- NX erişimi oturumda zaman zaman sıfırlanıyor → tekrar `request_access` gerekti.
