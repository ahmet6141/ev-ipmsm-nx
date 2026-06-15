# SIRADAKİLER — yapacaklarımız

## A) Analog çip projesi  ⟵ KULLANICININ ÖNCELİĞİ (beklemede)
- Kullanıcı, **analog çip projesine** devam etmek istiyor.
- Bu oturumda **henüz başlanmadı**; kullanıcı onayı bekleniyor.
- Başlarken: kapsam/hedef netleştir (hangi çip, hangi araç/akış — ör. şematik, SPICE/ngspice,
  layout/KLayout, vb.). Kullanıcının makinesinde ilgili araçlar kurulu görünüyor
  (ngspice, KLayout, Icarus Verilog, Anaconda...). Önce ne yapılacağını sor.

## B) Motor projesi — kalan adımlar (opsiyonel, onayla)
1. **Adım 5 — İmalat hazırlığı:**
   - ✅ Tolerans/GD&T şeması — `manufacturing.py` (12 özellik, web-temelli + denetlendi).
   - ✅ **BOM** — `manufacturing.py` (modelden kütle/adet; CLI `bom`/`tolerances` + CSV).
   - ✅ Lamine/mıknatıs/sargı/gövde süreç notları + montaj sırası — `docs/MANUFACTURING.md`.
   - ✅ **2D imalat resimleri** — `motor_nx/drawings.py` (ölçülü DXF+SVG; montaj/stator/rotor;
     CLI `drawings`). **Adım 5 TAMAM.**
   - ⬜ Opsiyonel: resmi NX Drafting çıktısı (antet/GD&T çerçevesi), kalıp/kesim resim seti.
2. **Temizlik:** test çıktılarını sil → `motor_v2..v8.*`, `motor_hairpin*.*`, `nx_smoke*.*`,
   geçici `_perf.py` / `_fea.py`. (Kullanıcı onayı gerek — kalıcı silme.)
3. **Commit/push:** değişiklikleri `git add/commit/push` ile `ev-ipmsm-nx` repoya gönder
   (terminal kullanıcı tarafında; ben terminale yazamıyorum).
4. **Opsiyonel iyileştirmeler:**
   - `configs/default.json` + sweep örnekleri (`batch_build.py` için).
   - Hairpin: kaynak tarafı + gerçek bağlantı şeması (ya da Motor-CAD'e bırak).
   - pyFEMM scripti (2D EM'i fiilen koşturmak için, ücretsiz).
   - Performans modelini malzeme Br'sinden Bg türetecek şekilde rafine et; stacking_factor'ı
     etkin manyetik boya uygula.

## Notlar
- Sandbox (Linux) bu oturumda kapalıydı (HYPERVISOR_VIRT_DISABLED) — Python'u doğrulamak için
  NX'in gömülü yorumlayıcısı (Play Journal) kullanıldı.
- NX erişimi oturumda zaman zaman sıfırlanıyor → tekrar `request_access` gerekti.
