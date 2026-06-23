# 01 — Mimari ve Klasör Haritası

## Katman mimarisi

Tek yön akış: parametreler → türetilmiş geometri → blueprint → tüketiciler. Yalnız
**bir** modül (`nx_builder`) NX'e bağımlıdır; gerisi saf CPython'dur ve test edilir.

```
            MotorParams (params.py)            <- TÜM tasarım girdileri
                  |
                  v
       em_design.derive() / validate()         <- türetilmiş radyuslar + doğrulama
                  |
                  v
       blueprint.generate()  ───────────────►  build_steps[]  (saf-matematik)
                  |                                   |
   ┌──────────────┼───────────────┬──────────────────┼──────────────┐
   v              v               v                  v              v
nx_builder    manufacturing     drawings            fea          preview
(NX katı +    (BOM + GD&T +     (2D DXF/SVG:        (FEA spec    (SVG kesit)
 STEP export) donanım listesi)  4 sayfa)            + DXF + CSV)
```

| Katman | Dosya | NX'e bağımlı? | Görevi |
|---|---|:--:|---|
| Parametreler | [motor_nx/params.py](../motor_nx/params.py) | Hayır | Tüm girdiler: `stator/winding/rotor/shaft/cooling/`**`assembly`**`/material` dataclass'ları, JSON I/O, NX expression tablosu |
| Boyutlandırma | [motor_nx/em_design.py](../motor_nx/em_design.py) | Hayır | Türetilmiş geometri (`derive`), **doğrulama** (`validate`, montaj özellikleri dâhil), birinci-mertebe performans |
| Blueprint | [motor_nx/blueprint.py](../motor_nx/blueprint.py) | Hayır | Saf-matematik geometri → sıralı **build-step** listesi (tube/cylinder/extrude/revolve/**hole**) + JSON |
| Önizleme | [motor_nx/preview.py](../motor_nx/preview.py) | Hayır | NX'siz SVG kesit |
| İmalat | [motor_nx/manufacturing.py](../motor_nx/manufacturing.py) | Hayır | BOM + GD&T tolerans + **donanım listesi** + **DFM Monte Carlo (Cpk)** + **tek-klasör imalat paketi** + eksantriklik yığılımı |
| Analiz | [motor_nx/analysis.py](../motor_nx/analysis.py) | Hayır | Birinci-mertebe kayıp/termal/demag/rotor-gerilme/cogging |
| Resimler | [motor_nx/drawings.py](../motor_nx/drawings.py) | Hayır | Ölçülü 2D imalat resimleri (GD&T çerçeveleri + datum'lar): **montaj / stator / rotor / şaft / patlatılmış montaj** (DXF + SVG) |
| FEA | [motor_nx/fea.py](../motor_nx/fea.py) | Hayır | FEA hand-off (spec JSON + kesit DXF + sargı/FEMM haritaları) |
| NX builder | [motor_nx/nx_builder.py](../motor_nx/nx_builder.py) | **Evet** | NXOpen ile build-step'leri NX'te katıya çevirir + STEP/Parasolid/parça-başına export |
| Batch sürücü | [batch_build.py](../batch_build.py) | Hayır | run_journal.exe'yi sürer; parametre süpürme + manifest |
| CLI | [motor_nx/cli.py](../motor_nx/cli.py) | Hayır | report / validate / blueprint / preview / bom / tolerances / **hardware** / **dfm** / **package** / drawings / fea / analysis |

## Build-step kelime dağarcığı

Blueprint, NX'in bildiği küçük, sürümden-bağımsız komutlar listesidir. Her step bir
boolean op (`create` / `subtract` / `unite`), opsiyonel `target` gövde ve opsiyonel
dairesel `pattern` (sayı + açı) taşır.

| Kind | Geometri | Eksen | Kullanım |
|---|---|---|---|
| `tube` | İçi boş silindir (dış/iç yarıçap) | Z | stator/rotor lamine, ceket |
| `cylinder` | Dolu silindir, (cx,cy)'ye ötelenebilir | Z | mıknatıs/soğutma kesimleri, **tie-rod/rivet/civata delikleri**, flanş diski |
| `extrude` | Kapalı XY poligonunu +Z'de uzat | Z | oluk/cep kesimleri, barlar, **kama yuvaları, OD kama** |
| `revolve` | (r,z) profilini Z etrafında döndür | Z | şaft, **segman (retaining-ring) kanalı** |
| `hole` | Silindirik delik, **keyfi eksende** | herhangi | **radyal**: soğutucu portları, terminal, kaldırma deliği, içi boş şaft yağ delikleri |

> `hole` primitifi montaj özellikleri için eklendi; mevcut doğrulanmış extrude+boolean
> yolunu keyfi yön ile yeniden kullanır. **Sıradaki in-NX smoke koşusunda doğrulanmalı**
> (bkz. [../docs/NX_AUTOMATION.md](../docs/NX_AUTOMATION.md)).

## Klasör haritası (yeniden düzenlenmiş)

```
motor/
├── README.md                  # hızlı başlangıç + komutlar
├── batch_build.py             # NX batch sürücü (parametre süpürme)
├── motor_nx/                  # ÇEKIRDEK KÜTÜPHANE (yukarıdaki katmanlar)
│   ├── params.py  em_design.py  blueprint.py  preview.py
│   ├── manufacturing.py  analysis.py  drawings.py  fea.py
│   ├── nx_builder.py          # tek NX-bağımlı modül
│   ├── nx_smoketest.py        # NX API smoke testi
│   └── cli.py                 # komut satırı
├── configs/                   # parametre config'leri (default.json, sweep_example.json)
├── tests/                     # NX'siz birim testleri (78 test)
├── verification/              # dış-araç sürücüleri (Motor-CAD, FEMM, Simcenter, NX drafting)
├── docs/                      # tasarım/imalat/FEA/NX dokümantasyonu + örnek resimler
├── project_details/           # << BU KLASÖR: insan+AI için düz-metin proje anlatımı
├── PROJECT_MEMORY/            # oturum ilerleme günlüğü
├── fea/                       # cli fea çıktısı (spec + DXF + CSV haritaları)
├── manufacturing/             # cli package çıktısı (BOM+donanım+tolerans+DFM+çizimler; gitignore)
├── build/                     # üretilmiş CAD çıktısı (gitignore; build/legacy/ eski build'ler)
└── archive/                   # deneysel/scratch dosyalar (gitignore; _*.py, *.fem, *.sim)
```

Üretilen/yeniden-üretilebilir her şey (`build/`, `archive/`, `drawings/`,
`blueprint.json`, `preview.svg`, `*.prt/.stp/.x_t/.log`) gitignore'dadır; yalnız
`build/README.md` ve `archive/README.md` izlenir, böylece düzen kendini belgeler.
