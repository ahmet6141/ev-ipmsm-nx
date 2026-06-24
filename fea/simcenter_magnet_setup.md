# Simcenter MAGNET 3D — elle kurulum kartı (EV_IPMSM_traction)

Journal Simcenter MAGNET'in fiziğini scriptleyemediği için bu adımlar **GUI'de elle**
yapılır. Tüm değerler `fea/fea_spec.json`'dan türetilmiştir. Sıra: **Hava → Mesh →
Malzeme → Mıknatıslanma → Bobin → Sınır koşulu → Çöz → Post.**

Gövde adları (nx_builder): `STATOR_STEEL`, `ROTOR_STEEL`, `MAGNET_000..047` (12 konum × 4
eksenel segment), `COIL_000..433`, `SHAFT`, `HOUSING`, `AIR`.

---

## A) HAVA bölgesi  (journal ile)
Master `motor_named.prt` work part iken:
```
"%UGII_ROOT_DIR%\run_journal.exe" verification\simcenter_airbox.py
```
→ `AIR` silindiri (Ø≈405, boy≈200, motora ortalı), layer 20. Kaydeder. FEM'de **Update**.

## B) MESH  (FEM'de)
- Tüm gövdeleri 3D **Tetrahedral** mesh'le (AIR dahil).
- **Hava aralığında ince**: eleman ~0.2 mm, en az 3 radyal katman.
- Köprü/post/diş-ucu/mıknatıs-köşelerinde incelt.
- Non-manifold (konformal) — gövdeler ortak yüz paylaşıyor, mesh uyumlu olmalı.

## C) MALZEME  (mesh collector / fiziksel özellik)
| Gövde | Malzeme | Değerler |
|---|---|---|
| STATOR_STEEL + ROTOR_STEEL | Soft Magnet, **Nonlinear B-H** | B-H tablosu (aşağıda), ρ 7650, lam 0.27 mm, istif 0.96 |
| MAGNET_* | **Permanent Magnet** | Br **1.28 T**, Hcj **1592 kA/m** (Hcb 970), μ_recoil **1.05**, Br_tc −0.12 %/°C, Hcj_tc −0.55 %/°C, ρ 7500, σ 0.714 MS/m |
| COIL_* | Conductor / Copper | σ **59.6 MS/m**, μr 1.0  (veya library `Copper_C10100`) |
| AIR | Air | μr **1.0** |

**B-H eğrisi (13 nokta):**

| H (A/m) | 0 | 50 | 100 | 150 | 200 | 300 | 500 | 1000 | 2000 | 5000 | 10000 | 30000 | 80000 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B (T) | 0 | 0.55 | 0.95 | 1.18 | 1.33 | 1.49 | 1.60 | 1.71 | 1.80 | 1.90 | 1.96 | 2.05 | 2.20 |

## D) MIKNATISLANMA YÖNLERİ  (12 mıknatıs — her birinin 4 eksenel segmenti aynı yön)
Yön = global **+X eksenine göre CCW derece**; vektör = (cos θ, sin θ, 0). Mıknatısı
**konum açısına** göre ekranda bul (rotor yüzeyinde saat yönünde), o yöne ayarla.

| Kutup | Polarite | Mıknatıs konumu (açı) | Mıknatıslanma yönü θ |
|---|---|---|---|
| 0 | N | +13.7°  (+Y kol) | **342.5°** |
| 0 | N | −13.7°  (−Y kol) | **17.5°** |
| 1 | S | +73.7°  (+Y kol) | **222.5°** |
| 1 | S | +46.3°  (−Y kol) | **257.5°** |
| 2 | N | +133.7° (+Y kol) | **102.5°** |
| 2 | N | +106.3° (−Y kol) | **137.5°** |
| 3 | S | −166.3° (+Y kol) | **342.5°** |
| 3 | S | +166.3° (−Y kol) | **17.5°** |
| 4 | N | −106.3° (+Y kol) | **222.5°** |
| 4 | N | −133.7° (−Y kol) | **257.5°** |
| 5 | S | −73.7°  (+Y kol) | **137.5°** |
| 5 | S | −46.3°  (−Y kol) | **102.5°** |

> Kontrol: komşu kutuplar N/S dönüşümlü olmalı (6 kutup = 3N + 3S). Yanlışsa back-EMF/tork
> sıfıra yakın çıkar.

## E) BOBİN (Coil) — faz akımı
54 oluk, 3 faz. Her oluk tek faz belti (hairpin). **Cogging için akım = 0.**
Yüklü çalışmada faz akımı (peak) = **386 A** (= 273 A_rms × √2); etkin tur/oluk = **4**
(8 iletken / 2 paralel yol). Faz başına seri tur = 36, kw ≈ 0.96.

**Oluk → faz → işaret:**
- **Faz A** (18): `0+ 1+ 2+ 9- 10- 11- 18+ 19+ 20+ 27- 28- 29- 36+ 37+ 38+ 45- 46- 47-`
- **Faz B** (18): `6+ 7+ 8+ 15- 16- 17- 24+ 25+ 26+ 33- 34- 35- 42+ 43+ 44+ 51- 52- 53-`
- **Faz C** (18): `3- 4- 5- 12+ 13+ 14+ 21- 22- 23- 30+ 31+ 32+ 39- 40- 41- 48+ 49+ 50+`

(İşaret = sarım yönü; COIL_xxx gövdeleri oluk sırasına göre.)

## F) SINIR KOŞULU
- Dış **AIR yüzeyinde**: **Flux Tangent / A = 0** (manyetik vektör potansiyeli sıfır).
- (Tam motor 360° kullanıyorsan periyodiklik gerekmez; 1-kutup sektör kullansaydın
  radyal kesimlerde anti-periyodik gerekirdi.)

## G) ÇÖZ + POST
- Solution = **Static** (zaten kurulu). **Solve**.
- Post: **flux density B** dağılımını gör (çekirdek ~1.5–2 T, doymamalı aşırı).
- **Tork**: Post → Electromagnetic Torque (hava aralığı üzerinden Maxwell stres tensörü).
- Analiz matrisi için rotoru adım adım döndürüp tekrar çöz:
  - **cogging**: akım 0, 1 oluk-adımı (6.67° mek) boyunca ince adımlar.
  - **back-EMF**: açık devre, akı-bağı(θ) → türev.
  - **tork-açı (MTPA)**: 386 A peak, β = 0..45° tarama.
  - **ripple**: MTPA β'da bir elektriksel periyot.
  - **demag**: 150 °C mıknatıs + tepe ters akım (β=90°), min mıknatıs B'si knee'nin üstünde mi.

## H) Kabul kapısı (fea_spec)
peak tork ≥ 396 Nm · ripple ≤ 5 % · cogging ≤ 1 % (≤ ~2 Nm) · demag yok @150 °C · eff ≥ 95 %.
Sonuçları `simcenter_emag.py` ile (.sim hazır olunca) puanlatabilirsin.
