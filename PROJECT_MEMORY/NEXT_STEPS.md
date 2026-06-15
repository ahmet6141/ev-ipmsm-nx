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

**Kalan (çoğu kullanıcı-tarafı):**
1. ⬜ **Sürücüleri FİİLEN KOŞTUR:** `pip install ansys-motorcad-core` → `python verification/motorcad_emag.py`;
   NX'te `run_journal.exe verification\nx_drafting.py -args <motor>.prt A3`. (Lisans/çözüm sende.)
2. ⬜ **Testleri koştur (sandbox bu oturumda kapalıydı):** `python tests/test_em_design.py`,
   `python tests/test_configs.py`, ve diğer `tests/*.py`. Beklenen: hepsi geçer (statik izlendi).
3. ⬜ **FEMM çapraz-kontrol scripti** (ücretsiz; `femm_labels.csv` sürücülü pyFEMM) — sıradaki doğal ek.
4. ⬜ **Temizlik:** `motor_v2..v8.*`, `motor_hairpin*.*`, `nx_smoke*.*`, `_perf.py`/`_fea.py` sil
   (kalıcı silme — onay gerek). + `.gitignore`.
5. ⬜ **Commit/push:** `ev-ipmsm-nx` (terminal sende). Yeni dosyalar: `verification/`, `tests/test_configs.py`.
6. ⬜ Hairpin: kaynak tarafı + gerçek bağlantı şeması (ya da Motor-CAD'e bırak).

## Notlar
- Sandbox (Linux) bu oturumda kapalıydı (HYPERVISOR_VIRT_DISABLED) — Python'u doğrulamak için
  NX'in gömülü yorumlayıcısı (Play Journal) kullanıldı.
- NX erişimi oturumda zaman zaman sıfırlanıyor → tekrar `request_access` gerekti.
