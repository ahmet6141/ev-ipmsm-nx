# NXOpen otomasyon referansı (headless `run_journal`)

[motor_nx/nx_builder.py](../motor_nx/nx_builder.py)'nin kullandığı NXOpen Python
API'sinin pratik referansı. **Hedef sürüm: NX 2506** (NX 2025 sürekli-sürüm serisi).
Çekirdek NXOpen sınıfları NX 1900 → 2506 boyunca kararlıdır.

## Önce smoke test (NX 2506'da tek seferlik)

Tam motoru build etmeden önce, builder'ın kullandığı tüm API'leri (extrude +
dairesel pattern + boolean + expression + STEP/Parasolid export) küçük bir parçada
sınayan smoke-test'i çalıştır — üye adlarının senin kurulumunda çözüldüğünü 30
saniyede teyit eder:

```bat
"C:\Program Files\Siemens\NX2506\NXBIN\run_journal.exe" motor_nx\nx_smoketest.py -args C:\temp\nx_smoke > smoke.log 2>&1
```

`smoke.log` içinde `SMOKE TEST PASSED` + boş olmayan `nx_smoke_ap242.stp` görürsen
builder hazır. `FAIL <adım>: <hata>` satırı düzeltilecek tam çağrıyı verir. Aşağıdaki çağrılar bir araştırma
workflow'unda web kaynaklarıyla doğrulanmış ve düşmanca (adversarial) bir incelemeden
geçirilerek **sürüm-kaymasına karşı sertleştirilmiştir** — `nx_builder.py` bu
düzeltilmiş hâlleri kullanır.

## `run_journal.exe` komut satırı

```bat
set UGII_BASE_DIR=C:\Program Files\Siemens\NX2306
set UGII_ROOT_DIR=%UGII_BASE_DIR%\NXBIN
set SPLM_LICENSE_SERVER=28000@lisans-sunucu
set PATH=%UGII_ROOT_DIR%;%PATH%

REM Argümanlar -args ile ve .py yolundan SONRA verilir; sys.argv[1:]'e düşer.
"%UGII_ROOT_DIR%\run_journal.exe" motor_nx\nx_builder.py -args blueprint.json out.prt both > run.log 2>&1
```

- NX 11+ : araç `…\NXBIN\run_journal.exe` (eski NX'te `…\UGII`).
- `sys.argv[0]` = script adı; gerçek argümanlar `sys.argv[1]`'den başlar.
- Başsız modda `ListingWindow.WriteLine(...)` **STDOUT**'a akar (Information
  penceresi yoktur) — `> run.log 2>&1` ile yakalanır. `nx_builder` ilerlemeyi böyle
  raporlar.

## Çekirdek çağrılar (sertleştirilmiş)

| Adım | API (doğrulanmış) | Tuzak / düzeltme |
|---|---|---|
| Yeni mm parça | `theSession.Parts.FileNew()` → `Units = Part.Units.Millimeters` → **`newPart = fileNew.Commit()`** | Saf batch'te (`MakeDisplayedPart=False`) `Parts.Work` güncellenmez; parçayı **`Commit()`'in dönüşünden** al. Şablon yoksa `Parts.NewBaseDisplay(path, Millimeters)`'e düş. |
| İsimli expression | `Expressions.CreateWithUnits("ad=formül", unit)` / `EditWithUnits(e, unit, rhs)` | Değer için `.RightHandSide` (expression birimi) kullan, `.Value` (taban birim) değil → 25.4× hatası. Edit RHS-only, create `"ad=rhs"`. |
| Profil (eğri zinciri) | `Curves.CreateLine` / `CreateArc(c, xDir, yDir, r, başAçı, bitAçı)` | Açılar **radyan**, CCW. Yay yönünü `min/max` ile sarma — normalize edilmiş başlangıç/bitişi doğrudan ver. |
| Section | `Sections.CreateSection(tol…)` → `ScRuleFactory.`**`CreateRuleCurveDumb`**`(curves)` → `AddToSection(...)` | `CreateRuleBaseCurveDumb` NX 7.5'ten beri **kullanımdan kalktı** — `CreateRuleCurveDumb` kullan. `AllowSelfIntersection(False)` + temiz kapalı döngü. |
| Extrude | `Features.CreateExtrudeBuilder` → `Limits.EndExtend.Value.RightHandSide = "stack_length"` | Limit RHS'i **expression adı** olarak ver → parametrik. `BooleanOperation.SetTargetBodies([body])`. `CommitFeature()`, sonra `Destroy()`. |
| Revolve | `Features.CreateRevolveBuilder` → `Axis` = Z ekseni, `EndExtend = "360"` | (r,z) profili XZ düzleminde. |
| Dairesel pattern (opsiyonel) | `CreatePatternFeatureBuilder` → `PatternService.PatternType = …Circular` → **`pfb.FeatureList.Add([feat])`** → `circ = PatternService.CircularDefinition` → `circ.AngularSpacing.SpaceType =` **`SpacingType.Offset`** → `circ.AngularSpacing.NCopies.RightHandSide` / `.PitchDistance.RightHandSide` | NX 2506 doğrulaması: `AddFeatureToPattern` **yok** (→ `FeatureList.Add`). Enum `PatternSpacing.`**`SpacingType`** (Enum-suffix yok), değer **`Offset`** (`CountAndPitch` **yok**). Builder bu yolu **varsayılan kullanmaz** (aşağıya bakın). |
| Boolean | `CreateBooleanBuilderUsingCollector` → `Operation` → `scc = bld.ToolBodyCollector` (get/set) → `scc.ReplaceRules([CreateRuleBodyDumb(tools, True)], False)` → `bld.CommitFeature()` | Builder bunu kullanmaz — kesim/birleştirme doğrudan Extrude'un `BooleanOperation.SetTargetBodies` ile yapılır. |
| **Update** | `theSession.UpdateManager.DoUpdate(undoMark)` | **Zorunlu.** Feature/expression, model güncellenene dek gerçekleşmez; atlanırsa export boş/eski çıkar. Her adımdan önce `SetUndoMark`, hata olursa `UndoToMark`. |
| STEP export | `DexManager.CreateStepCreator()` → `ExportAs = …Ap242/Ap214/Ap203` → `SelectionScope = SelectedObjects` + **`SelectionComp.Add(solid_bodies)`** → `InputFile = part.FullPath` → `Commit()` | `EntirePart` yerine katı gövdeleri seç → boş/kısmi STEP riskini önler. Önce `part.Save(...)` (translator diskten okur). |
| Parasolid export | `DexManager.`**`CreateParasolidExporter()`** (NX 2506; `CreateParasolidCreator` **yok**); yedeği UF: **`theUF.Ps.ExportData(bodyList, fileName)`** | Builder ikisini de dener, olmazsa UF'ye düşer. UF imzası (gövde listesi, dosya). Dosya varsa önce **sil**. |
| Yeni mm parça | **`Parts.NewBaseDisplay(path, NXOpen.BasePart.Units.Millimeters)`** (şablonsuz, birincil) | **Gerçek NX 2506'da doğrulandı:** birim enum'u **`BasePart.Units`** olmalı — `Part.Units.Millimeters` (`PartUnitsMemberType`) "Second parameter is invalid" verir. NewBaseDisplay var olan `.prt` üzerine yazmayı reddeder ("File already exists") → builder oluşturmadan önce eski `.prt`/`.stp`'yi siler. `FileNew` şablonu (`model-plain-1-mm-template.prt`) bu kurulumda yok; `NewDisplay` 2 argüman ister — ikisi de yalnız yedek. `UndoToMark` parça oluşturmadan sonra "Undo mark is missing" verebilir → savunmacı sarıldı. |

## Önemli tuzaklar

- **Lisans/başsızlık:** `run_journal` uygun bir NX yazarı lisansı ister. Stripped
  batch lisansında UI çağrıları (NXMessageBox, seçim diyalogları) asılır/hata verir.
- **Birim:** yeni parça inç şablonundan gelebilir → `Units = Millimeters` zorla.
- **PatternFeatureBuilder üye adları** sürümler arası en çok kayan kısımdır. Bir
  üye çözülmezse, hedef NX build'inizde elle bir dairesel pattern **kaydedip**
  üretilen journal'dan tam üye adlarını kopyalayın.
- **Bellek:** uzun süpürmelerde her builder'ı `Destroy()`, `PartLoadStatus`'u
  `Dispose()`, bitmiş parçaları kapatın (`Parts.CloseAll`).

## `nx_builder.py` akışı

1. `new_mm_part(out.prt)` — mm parça (Commit'ten yakalanır).
2. `push_expressions(...)` — tüm parametreleri isimli expression olarak yazar;
   `stack_length` aktif-stack feature'larının eksenel boyunu **sürer**
   (NX'te düzenle + Update → stack yeniden boyutlanır).
3. Her build step → bir/iki feature (tube = dış extrude + iç subtract; extrude;
   revolve; cylinder). Tekrarlı feature'lar (oluk, mıknatıs, iletken, kanal)
   **varsayılan olarak açık instance** ile üretilir: profil döndürülüp her kopya
   ayrı extrude edilir — yalnız onaylanmış-kararlı API (extrude + boolean), pattern
   üye-adı kayması riski yok. Daha hafif ağaç için 4. argüman `nxpatterns` ile NX
   dairesel pattern feature'ı açılır (smoke test onayladıktan sonra). Her adım
   undo-mark + `DoUpdate` ile sarılı; hata o adımı geri alır, batch devam eder.
4. `export_step` (AP242) ve `export_parasolid` (.x_t) — yalnız katı gövdeler.

> Düşmanca incelemenin **orta-yüksek** güveni: oturum/expression/extrude/revolve/
> STEP iskeleti iyi doğrulanmış; pattern enum'larının tam yazımı ve ParasolidCreator
> ayrıntıları sürüme göre kayabilir — ilk build'de bir kez doğrulayın
> (boş olmayan .stp ve tek bir pattern'ın çalıştığını teyit edin).
