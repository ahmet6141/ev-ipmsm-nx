# PROJE DETAYLARI — insan + yapay zekâ için tek kaynak

Bu klasör, **motor_nx** projesini (parametrik EV traksiyon IPMSM üreteci) bir
insanın ya da bir yapay zekâ ajanının kodu okumadan anlayabileceği **düz metin**
olarak anlatır. Her dosya tek bir konuya odaklanır; sayısal değerler ve parametre
adları koddaki gerçek değerlerle birebir tutulur (`motor_nx/params.py`).

> Bu klasör **anlatım/şartname** içindir — kaynak gerçeği koddur. Bir çelişki
> görürsen koda (ve `python -m motor_nx.cli report | bom | tolerances | hardware`
> çıktısına) güven; sonra burayı güncelle.

## Okuma sırası

| Dosya | İçerik |
|---|---|
| [00_GENEL_BAKIS.md](00_GENEL_BAKIS.md) | Proje nedir, hedef, kapsam (EM-aktif katı **+** montaj/üretim özellikleri), kim için |
| [01_MIMARI.md](01_MIMARI.md) | Katman mimarisi (params → em_design → blueprint → builder/mfg/drawings/fea) + **klasör haritası** |
| [02_PARCALAR.md](02_PARCALAR.md) | **Parça parça** şartname: her parçanın geometri + malzeme + üretim yöntemi + **montaj delikleri/özellikleri** + toleransları |
| [03_MONTAJ_OZELLIKLERI.md](03_MONTAJ_OZELLIKLERI.md) | Her montaj/üretim özelliğinin **kataloğu**: parametre, varsayılan, standart, gerekçe, doğrulama kuralı |
| [04_AKTARMA_ORGANI.md](04_AKTARMA_ORGANI.md) | Motora bağlanan **aktarma organı** (`driveline_nx`): diferansiyel (open/e-LSD/tork-vektörleme), CV mafsallı yarım akslar, **tekerlek göbeği + bijon bağlantı elemanları** |

## Diğer belgeler (kök ve docs/)

- [../README.md](../README.md) — hızlı başlangıç + komutlar
- [../docs/DESIGN.md](../docs/DESIGN.md) — elektromanyetik tasarım gerekçesi
- [../docs/MANUFACTURING.md](../docs/MANUFACTURING.md) — BOM + GD&T + imalat süreci + montaj sırası
- [../docs/NX_AUTOMATION.md](../docs/NX_AUTOMATION.md) — NXOpen otomasyon referansı
- [../docs/FEA_HOWTO.md](../docs/FEA_HOWTO.md) / [../docs/FEA_PREP.md](../docs/FEA_PREP.md) — FEA doğrulama
- [../docs/PROJECT_PLAN.md](../docs/PROJECT_PLAN.md) — uçtan uca yol haritası (P0–P10)
- [../PROJECT_MEMORY/](../PROJECT_MEMORY/) — oturum ilerleme günlüğü

## 30 saniyelik özet

54 oluk / 6 kutup, tek-V N42SH, hairpin sargılı, su-soğutmalı bir EV traksiyon
IPMSM'i **tek bir parametre setinden** (`MotorParams`) tam otomatik üretir:
saf-matematik bir **blueprint** (build-step listesi) çıkarır, bunu Siemens NX'te
katı modele çevirir ve STEP/Parasolid olarak dışa aktarır. Aynı blueprint'ten
**BOM, GD&T toleransları, bağlantı-elemanı listesi, 2D imalat resimleri ve FEA
hand-off paketi** türetilir. 3D model artık yalnız EM-aktif gövdeleri değil,
**profesyonel montaj/üretim özelliklerini** de içerir (bağlama delikleri, kama
yuvaları, montaj flanşı + civata daireleri, soğutucu portları, terminal, kaldırma
halkası) — bkz. [03_MONTAJ_OZELLIKLERI.md](03_MONTAJ_OZELLIKLERI.md).
