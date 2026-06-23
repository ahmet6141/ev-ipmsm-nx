# 03 — Montaj / Üretim Özellikleri Kataloğu

Bu, EM-aktif katının üzerine eklenen her üretim/montaj özelliğinin referansıdır.
Tümü [`motor_nx/params.py`](../motor_nx/params.py) içindeki `AssemblyParams`
dataclass'ında parametriktir, [`blueprint.py`](../motor_nx/blueprint.py)
`assembly_steps()` ile geometriye dökülür ve [`em_design.py`](../motor_nx/em_design.py)
`_validate_assembly()` ile **build'den önce** doğrulanır.

## Tasarım kuralları

1. **Ana anahtar:** `assembly.enabled` (varsayılan `True`). `False` → saf EM katı.
2. **Bağımsız kapatma:** bir özelliğin sayısını ya da boyutunu `0` yapmak yalnız o
   özelliği kaldırır; gerisi etkilenmez.
3. **AUTO yerleşim:** `*_pitch_radius = 0.0` → em_design/blueprint malzeme ortasına
   yerleştirir (boyunduruk ortası, hub ortası, flanş lip'i).
4. **Önce doğrula:** her özellik geometrik fizibilite kontrolünden geçer (duvardan
   çıkmaz, oluk/mıknatıs/kanalla çakışmaz, civata dairesi flanşa sığar, halkalar
   üst üste binmez). Bir hata o özelliği zerolayarak çözülür, gerisi etkilenmez.
5. **Standartlar:** ISO 286 (geçmeler), DIN 6885-A (kamalar), DIN 471/472 (segmanlar),
   ISO 4762 / metrik geçiş delikleri (civatalar), DIN 580 (kaldırma cıvatası).

---

## Stator lamine

| Özellik | Parametre(ler) | Varsayılan | Standart | Gerekçe / doğrulama |
|---|---|---|---|---|
| Tie-rod / bağlama deliği | `stator_tie_rod_count`, `_diameter`, `_pitch_radius` | 6 × Ø6.5, AUTO (R≈106) | M6 / ISO 4762 | Paketi sıkıştır/handle/konumla. Doğrulama: slot bottom'a ve OD'ye ≥1.5 mm duvar; halkalar binmesin |
| Dönmez OD kaması | `stator_key_count`, `_width`, `_depth` | 2 × 6×2.5 mm | paralel kama | Shrink-fit'te arıza torkunu karşıla. Doğrulama: derinlik < back-iron; OD'de çentikler binmesin |

## Rotor lamine

| Özellik | Parametre(ler) | Varsayılan | Standart | Gerekçe / doğrulama |
|---|---|---|---|---|
| Rivet / uç-plaka deliği | `rotor_rivet_count`, `_diameter`, `_pitch_radius` | 6 × Ø5, AUTO (R≈39.6) | perçin/dowel | Uç-plakaları tut, paketle-hizala. Doğrulama: şafta ≥2 mm web; mıknatıs cebine ≥1.5 mm; binme yok |
| Bore kama yuvası (ops.) | `rotor_keyway_width`, `_depth` | 0 (KAPALI) | DIN 6885 | Varsayılan press/shrink fit. Açılırsa: derinlik mıknatıs cebine ulaşmamalı |

## Şaft

| Özellik | Parametre(ler) | Varsayılan | Standart | Gerekçe / doğrulama |
|---|---|---|---|---|
| Çıkış stub'ı (DE) | `shaft.drive_stub_diameter`, `_length` | Ø32 × 45 | — | Rulmanın ötesinde çapça-aşağı çıkış uzantısı; kamayı taşır. Doğrulama: stub < bearing seat (basamak); bore'a ≥1.5 mm |
| Tahrik-ucu kama yuvası | `shaft_keyway_width`, `_depth`, `_length` | 12 mm, derinlik 5, L=18 | DIN 6885-A | Çıkış kaplini/dişlisine tork. **Stub varsa stub'a**, yoksa DE seat'e (clamp'li). Doğrulama: derinlik yüzeyden bore'a ≥1.5 mm; genişlik < yüzey |
| Segman (retaining) kanalı | `shaft_snap_ring_width`, `_depth` | 2×1.4 mm | DIN 471 | Rulman iç bileziğini eksende konumla. Doğrulama: kanal dibi bore'a ≥1.5 mm |
| Radyal yağ çapraz-deliği | `shaft_oil_hole_count`, `_diameter` | 4 × Ø4 | — | İçi boş şafttan rotor soğutma yağı. Doğrulama: içi boş şaft (bore>0) gerekir; journal'da binme yok |

## Gövde / su ceketi

| Özellik | Parametre(ler) | Varsayılan | Standart | Gerekçe / doğrulama |
|---|---|---|---|---|
| Montaj flanşı (DE + NDE) | `housing_flange_thickness`, `_od_margin` | 12 mm, +30 mm lip | — | Uç-kalkan/şanzıman arayüzü. Disk-unite + bore-reopen (`hole` değil, eksenel) |
| Şanzıman montaj civatası (DE) | `housing_mount_bolt_count`, `_diameter` | 8 × Ø11 (R≈140.6) | M10 / ISO 4762 | Motoru şanzımana cıvatala. Doğrulama: civata dairesi flanş lip'ine sığsın; halkalar binmesin |
| Uç-kalkan civatası (her iki uç) | `housing_endshield_bolt_count`, `_diameter` | 8 × Ø7 (R≈128) | M6 / ISO 4762 | Rulman kapaklarını/uç-kalkanları cıvatala (DE+NDE = 16) |
| Soğutucu giriş+çıkış portu | `housing_coolant_port_diameter` | 2 × Ø12 | BSP/NPT/O-ring boss | Ceket beslemesi (radyal `hole`) |
| Terminal / kablo geçişi (+ boss) | `housing_terminal_diameter`, `housing_terminal_boss` | Ø28 + 8 mm boss | kablo glandı | 3-faz çıkışı; IP-keçeli. Boss = terminal kutusunun cıvatalandığı kabartılmış döküm pad (radyal `hole` unite) |
| Kaldırma deliği (üst) | `housing_lifting_hole_diameter` | 1 × Ø11 dişli | M10 / DIN 580 | Handling/hoisting (radyal `hole`) |

## End-shield'ler / rulman kapakları (ayrı parça)

| Özellik | Parametre(ler) | Varsayılan | Standart | Gerekçe / doğrulama |
|---|---|---|---|---|
| End-shield (DE + NDE) | `endshield_enabled`, `endshield_thickness`, `endshield_bearing_bore` | Açık, 14 mm, Ø80 bore | ISO 15 rulman | Rulmanları stator bore'una eş-merkez taşır; flanşa cıvatalanır. Ayrı döküm gövde; civata deseni gövdeyle ortak. Doğrulama: bore > bearing seat ve < flanş OD |

---

## Geometrik notlar (uygulama)

- **Eksenel delikler** (tie-rod, rivet, civata daireleri) `cylinder`-subtract; lamine
  delikleri `drive_with_stack=True` → NX'te `stack_length` ile birlikte ölçeklenir.
- **Kama yuvaları / OD kama** `extrude`-subtract (eksenel cep/çentik).
- **Segman kanalı** `revolve`-subtract (tam 360° halka oluk).
- **Radyal delikler** (soğutucu/terminal/kaldırma/yağ) yeni `hole` primitifi: keyfi
  eksende silindirik kesim. Dairesel pattern hem taban noktasını hem ekseni Z
  etrafında döndürür.
- **Flanş** dolu disk-`unite` + bore-`reopen` (`cylinder`-subtract) olarak modellenir;
  böylece NX'te yalnız lip (ceket OD'sinden dışarı) eklenir, bore şaft için açık kalır.
  BOM: flanşlar tek dökümde birleşir (qty=1); lip ~0.3 kg yaklaşım payıyla sayılır.

## Doğrulama (özet)

`em_design.validate(p)` boş liste döndürürse tasarım — montaj özellikleri dâhil —
build edilebilir. Hızlı kontrol:

```bash
python -m motor_nx.cli validate            # exit kodu + sorun listesi
python -m motor_nx.cli hardware            # bağlantı-elemanı listesi (boyut/adet/standart)
python -m motor_nx.cli tolerances          # GD&T (montaj satırları dâhil)
python -m motor_nx.cli dfm                 # DFM Monte Carlo: hava-aralığı eksantriklik Cpk'sı
python -m motor_nx.cli package -o mfg/     # tüm imalat paketi (BOM+donanım+tolerans+DFM+5 çizim+özet)
python tests/test_assembly.py              # montaj geometri + doğrulama testleri (22 test)
```

Bir özellik bir varyantta geometrik olarak sığmıyorsa doğrulama net bir mesajla
söyler (ör. "rotor rivet holes ... would clip a pocket"); o özelliği zerolar veya
ilgili parametreyi ayarlarsın — gerisi etkilenmez.
