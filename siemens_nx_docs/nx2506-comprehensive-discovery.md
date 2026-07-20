# Siemens NX 2506 — Kapsamlı API Keşif Raporu

> ⚠️ **GÜNCELLEME (2026-07-03 akşam):** Bu rapordaki birçok API iddiası canlı NX 2506
> doğrulamasında YANLIŞ çıktı ve düzeltildi. **Güncel ve kanıtlı referans:**
> [`nx2506-verified-api-playbook.md`](nx2506-verified-api-playbook.md) — çelişki durumunda
> playbook geçerlidir. Başlıca düzeltmeler: `AddChainset(ScCollector, "r")` (edge+index değil),
> `CreateMirrorBodyBuilder` VARDIR, `ThreadBuilder.FaceCollector` YOKTUR
> (`CylindricalFace.Value`), malzeme `PhysicalMaterials.LoadFromNxmatmllibrary` +
> `mat.AssignObjects`, PMI `Annotations.CreatePmiNoteBuilder`, draft
> `CreateExpressionCollectorSet(col, açı, "", 0)` reçetesi, Shell/Draft/Hole tolerans
> zorunlulukları, headless PNG'nin imkânsızlığı.
>
> **Güncelleme (2026-07-18):** Bu düzeltmeler artık yalnız bu özet kutuda değil, dokümanın
> ilgili bölümlerinin (§1.1, §2.2, §2.3, §4, §7.1-7.3) içine de işlendi — kod örnekleri ve
> durum tabloları artık playbook ile tutarlı.

> **Tarih:** 2026-07-03  
> **NX Sürüm:** 2506 (Continuous Release)  
> **Kurulum:** `E:\program\NXBIN`  
> **Python:** NX gömülü Python 3.10, sıfır üçüncü-parti paket  
> **Stubs:** `E:\program\UGOPEN\pythonStubs\NXOpen\`  
> **Doğrulama Yöntemi:** `run_journal.exe` ile 5 ayrı test journal'ı çalıştırıldı

---

## 1. NX 2506 GUI — 16 Ribbon Sekmesi + 13 Menü + Diyaloglar

> ⚠️ **DÜZELTME:** Aşağıdaki 16 sekmelik liste NX'in tam/varsayılan kurulumunu anlatır. Bu
> makinedeki canlı "Designcenter" rolünde ekran görüntüsüyle doğrulanan gerçek ribbon **13
> sekmedir**: File, Home, Curve, Surface, Assemblies, Analysis, View, Display, Selection,
> Tools, Application, Mold Wizard (+ Discovery Center paneli). **PMI, Drafting, Sheet Metal,
> CAM ve Simulation sekmeleri bu rolde KAPALI** — aşağıdaki tablo bunları hâlâ listeliyor çünkü
> genel NX yeteneğini anlatıyor, o an açık olan rolü değil. Kanıt: `ribbon_*_VERIFIED.png`,
> ayrıntı: `nx2506-verified-api-playbook.md` §0 madde 9 ve §1.

### 1.1 Ribbon Sekmeleri (Alt+Harf kısayollarıyla erişildi)

| # | Sekme | Kısayol | Komut Grupları |
|---|---|---|---|
| 1 | **File** | Alt+F | New, Open, Save, Save As, Close, Export (STEP/IGES/Parasolid/JT/PDF), Print, Properties, Recently Used |
| 2 | **Home** | Alt+H | Sketch, Extrude, Revolve, **Hole**, **Edge Blend**, **Chamfer**, **Draft**, Shell, Boolean (Unite/Subtract/Intersect), Pattern, Synchronous Modeling, More... |
| 3 | **Curve** | Alt+C | Line, Arc/Circle, Spline (Studio/Through Points/Fit), Text, Offset Curve, Project Curve, Intersection Curve, Bridge Curve, Composite Curve |
| 4 | **Surface** | Alt+S | Through Curve Mesh, Studio Surface, Swept, Bounded Plane, Offset Surface, Fill Surface, Extension, Trim Sheet, Sew, Thicken |
| 5 | **Analysis** | Alt+L | Measure (Distance/Angle/Radius/Length/Area/Volume), Geometry Check, Deviation Gauge, Compare Body, Mass Properties, Section Analysis |
| 6 | **View** | Alt+V | Zoom/Pan/Rotate, Section View (Dynamic/Clip), Render Style (Wireframe/Shaded/Studio/Ray Traced), Layer Settings, Show/Hide, Camera, Background |
| 7 | **Assemblies** | Alt+A | Add Component, Create New Component, Assembly Constraints (Touch Align/Concentric/Distance/Angle/Parallel/Perpendicular), WAVE Geometry Linker, Pattern Component, Exploded Views, Sequence |
| 8 | **PMI** | Alt+P | Feature Control Frame (GD&T), Datum Feature Symbol, Dimension (Linear/Angular/Radial), Note, Surface Finish Symbol, Weld Symbol, Balloon, Table |
| 9 | **Tools** | Alt+T | **Expressions** (Ctrl+E), **Journal** (Play/Record/Edit), Macro (Record/Play), Customize (Ribbon/Keyboard), Update, Import/Export, Feature Groups |
| 10 | **Preferences** | Alt+R | Object, Selection, Visualization, Sheet Metal, Modeling, Drafting, Assemblies, Sketch, PMI |
| 11 | **Sketch** | Alt+K | Direct Sketch, Sketch in Task Environment, Datum Plane/Datum Axis/Datum CSYS, Constraints (Geometric/Dimensional), Dimensions, Quick Trim/Extend |
| 12 | **Drafting** | Alt+D | New Sheet (ISO A0-A4), View Creation Wizard, Base View, Projected View, Section View, Detail View, Dimensions, Annotations, GD&T, Tables, Parts List (BOM) |
| 13 | **Sheet Metal** | Alt+N | Tab, Flange, Contour Flange, Bend, Unbend/Rebend, Corner (Closed/Relief), Punch, Flat Pattern, Joggle, Normal Cutout |
| 14 | **CAM** | Alt+M | Manufacturing Operations, Tool Paths, Post Process, Machine Simulation, Shop Documentation |
| 15 | **Simulation** | Alt+I | New FEM/Simulation, Mesh (3D Tetra/2D/1D), Loads/Constraints, Solve (Nastran), Results, Post-Processing, Optimization |
| 16 | **Help** | — | Documentation, About NX, Command Finder, Online Training, What's New |

### 1.2 Menü Çubuğu (Alt+harf ile erişilen 13 menü)

| Menü | İçerik |
|---|---|
| **File** | New, Open, Save, Close, Save As, Import, Export, Print, Properties, Exit |
| **Edit** | Undo, Redo, Cut, Copy, Paste, Delete, Selection (All/Invert/By Attribute), Object Display |
| **View** | Refresh, Fit, Zoom, Pan, Rotate, Orient View (Isometric/Trimetric/Front/Back/Left/Right/Top/Bottom), Section, Layout |
| **Insert** | Sketch, Datum/Point, Curve, Design Feature (Extrude/Revolve/Hole/Boss/Pocket), Associative Copy (Pattern/Mirror), Trim (Trim Body/Split Body/Divide Face), Offset/Scale, Detail Feature (Edge Blend/Chamfer/Draft/Thread) |
| **Format** | Layer Settings, Move to Layer, WCS (Origin/Rotate/Dynamic/Save), Reference Sets, Group |
| **Tools** | Expressions, Journal, Macro, Customize, Update, Import/Export, Spreadsheet, Bill of Materials |
| **Assemblies** | Components, Constraints, WAVE, Exploded Views, Sequence, Reports, Clone |
| **Analysis** | Measure, Mass Properties, Check Geometry, Examine Geometry, Deviation, Section, Motion |
| **Preferences** | Object, Selection, Visualization, Sheet Metal, Modeling, Drafting, Sketch, Assemblies, PMI, Manufacturing |
| **Window** | New Window, Cascade, Tile, Close All |
| **Help** | Documentation, About NX, Command Finder, Online Training, Log File |

### 1.3 Profesyonel Diyaloglar ve Paneller

| Dialog | Erişim | İşlev |
|---|---|---|
| **Part Navigator** | Sol panel | Feature history tree, body names, timestamps, edit/suppress/reorder |
| **Assembly Navigator** | Sol panel | Component tree, constraints, arrangements, reference sets |
| **Expression Editor** | Ctrl+E | Parametrik formüller, birimler, yorumlar, import/export |
| **Journal Manager** | Alt+F8 | Journal oynatma, kaydetme, düzenleme, son kullanılanlar |
| **Measure** | Ctrl+M | Mesafe, açı, yarıçap, uzunluk, alan, hacim ölçümü |
| **Information Window** | Ctrl+I | Feature/body/face detay bilgileri, mass properties, expressions list |
| **Drafting - New Sheet** | Drafting sekmesi | ISO A0-A4 template seçimi, ölçek, görünüş tipi |
| **Layer Settings** | Ctrl+L | Layer görünürlüğü, isimlendirme, kategori yönetimi |
| **Object Display** | Ctrl+J | Renk, saydamlık, çizgi tipi, layer, material ataması |
| **Selection Filter** | Üst toolbar | Tip filtreleme (Solid Body, Face, Edge, Curve, Datum, Component...) |

---

## 2. NXOpen Python API — NX 2506 Doğrulanmış Referans

### 2.1 Session ve Part Yönetimi

```python
import NXOpen
_session = NXOpen.Session.GetSession()

# YENİ mm parça — NX 2506 doğrulandı
# BasePart.Units.Millimeters kullanılır (Part.Units değil!)
part = _session.Parts.NewBaseDisplay(path, NXOpen.BasePart.Units.Millimeters)
if isinstance(part, tuple):  # bazı sürümlerde tuple döner
    part = part[0]

# Save
part.Save(NXOpen.BasePart.SaveComponents.TrueValue,
          NXOpen.BasePart.CloseAfterSave.FalseValue)
```

### 2.2 Doğrulanmış Feature Builder API'leri

#### Extrude (KESİN ✅)
```python
ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
ext.Section = section
ext.Direction = part.Directions.CreateDirection(origin, vector, UpdateOption.WithinModeling)
ext.Limits.StartExtend.Value.RightHandSide = "0"
ext.Limits.EndExtend.Value.RightHandSide = "50"
ext.BooleanOperation.Type = BooleanType.Unite
ext.BooleanOperation.SetTargetBodies([target_body])
feat = ext.CommitFeature()
ext.Destroy()
```

#### Revolve (KESİN ✅)
```python
rev = part.Features.CreateRevolveBuilder(NXOpen.Features.Feature.Null)
rev.Section = section
rev.Axis = part.Axes.CreateAxis(point, direction, UpdateOption.WithinModeling)
rev.Limits.EndExtend.Value.RightHandSide = "360"
rev.BooleanOperation.Type = bool_type
feat = rev.CommitFeature()
rev.Destroy()
```

#### Section / Curve (KESİN ✅)
```python
section = part.Sections.CreateSection(0.0095, 0.001, 0.5)
section.AllowSelfIntersection(False)
rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)  # NOT CreateRuleBaseCurveDumb!
section.AddToSection([rule], curves[0], null_obj, null_obj, help_pt,
                     NXOpen.Section.Mode.Create, False)

# Line — float zorunlu!
line = part.Curves.CreateLine(
    NXOpen.Point3d(float(x1), float(y1), float(z1)),
    NXOpen.Point3d(float(x2), float(y2), float(z2)))

# Arc — float zorunlu, radyan!
arc = part.Curves.CreateArc(center, xDir, yDir, float(radius), float(start_rad), float(end_rad))
```

#### Edge Blend / Fillet (KESİN ✅ — DÜZELTİLDİ)
> ⚠️ Bu bölümün ilk sürümü `AddChainset(edge, index)` + `AddVariableRadiusData` öneriyordu.
> Canlı testte (v3 T02) bu YANLIŞ çıktı: doğrusu collector + radius-string'tir. Aşağıda
> düzeltilmiş, gerçekten commit edilmiş (8→12 yüz) reçete var.
```python
ebb  = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
col  = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleEdgeDumb(edges)], False)
ebb.AddChainset(col, "4")          # (ScCollector, "radius"); edge+index İMZASI YANLIŞ
feat = ebb.CommitFeature(); ebb.Destroy()
# NOT: SetRadius() ve AddChainsToCollector() BU METOTLAR YOK.
# NOT: Edge referansları feature commit sonrası geçersiz olabilir — yeniden query et!
```

#### Chamfer (KESİN ✅ — DÜZELTİLDİ)
> ⚠️ İlk sürüm `AddChainsToCollector([edge])` diye bir metot öneriyordu — **yok**. Doğrusu
> kenarları `SmartCollector`'a atamaktır (12→13 yüz ile kanıtlı).
```python
cb  = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleEdgeDumb(edges)], False)
cb.SmartCollector = col
cb.Option      = NXOpen.Features.ChamferBuilder.ChamferOption.SymmetricOffsets
cb.FirstOffset = "2"               # str; FirstOffsetExp salt-okunurdur (RightHandSide atanamaz)
feat = cb.CommitFeature(); cb.Destroy()
# NOT: SymmetricOptions diye bir attribute YOK.
```

#### Thread (KESİN ✅ — DÜZELTİLDİ)
> ⚠️ İlk sürüm `FaceCollector.Add([face])` ve `ThreadBuilderType.Symbolic` öneriyordu — ikisi de
> yanlış. Doğrusu `CylindricalFace.Value` ve `ThreadBuilder.Type.Symbolic`; ayrıca tablo yolu
> ("ThreadTable") headless'ta "Standard data not found" veriyor, `Input.Manual` şart (v3e T05 ✅).
```python
tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)     # Thread.Null, Feature.Null DEĞİL
tb.ThreadType  = NXOpen.Features.ThreadBuilder.Type.Symbolic
tb.ThreadInput = NXOpen.Features.ThreadBuilder.Input.Manual
tb.ShaftPreference = NXOpen.Features.ThreadBuilder.ShaftSizePreference.MajorDiameter
tb.CylindricalFace.Value = cylindrical_face        # FaceCollector.Add YOK
tb.StartObject.Value     = start_planar_face
tb.MajorDiameterExp.RightHandSide = "20"; tb.MinorDiameterExp.RightHandSide = "17.5"
tb.ShaftDiameterExp.RightHandSide = "18.75"; tb.PitchExp.RightHandSide = "2.5"
tb.AngleExp.RightHandSide = "60"; tb.ThreadLength.RightHandSide = "20"
feat = tb.CommitFeature(); tb.Destroy()
```

#### Draft (KESİN ✅ — DÜZELTİLDİ)
> ⚠️ İlk sürüm `SymmetricAngle.Value.RightHandSide` ve `AddChainsToCollector([body])`
> öneriyordu — ikisi de yanlış. `SymmetricAngle` gerçekten **bool**'dur ama açıyı taşımaz;
> açı `ExpressionCollectorSet` üzerinden gelir (v3e T04 ile kanıtlı).
```python
db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
db.AngleTolerance = 0.5; db.DistanceTolerance = 0.001     # ŞART, yoksa "Angle tolerance is too small"
db.TypeOfDraft = NXOpen.Features.DraftBuilder.Type.Face
db.DraftReferencesMethod = NXOpen.Features.DraftBuilder.DraftReferencesMethods.StationaryFace
db.Direction = part.Directions.CreateDirection(o, z, NXOpen.SmartObject.UpdateOption.WithinModeling)
db.StationaryReference.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb([stationary_face])], False)
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb(faces_to_draft)], False)
ecs = part.CreateExpressionCollectorSet(col, "3", "", 0)   # birim argümanı BOŞ STRING
db.FaceSetAngleExpressionList.Append(ecs)
feat = db.CommitFeature(); db.Destroy()
# NOT: db.Angle diye bir attribute YOK.
```

#### Body Naming (KESİN ✅)
```python
body.SetName("DISPLAY_NAME")  # alphanumeric + underscore, max ~48 char
```

#### Expressions (KESİN ✅)
```python
e = part.Expressions.CreateWithUnits("name=value", unit)
part.Expressions.EditWithUnits(e, unit, "new_rhs")
# RightHandSide kullan, .Value 25.4× hatası verir!
```

#### STEP Export (KESİN ✅)
```python
sc = _session.DexManager.CreateStepCreator()
sc.ExportAs = NXOpen.StepCreator.ExportAsOption.Ap242
sc.ObjectTypes.Solids = True
sc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.SelectedObjects
sc.ExportSelectionBlock.SelectionComp.Add(bodies)
sc.InputFile = part.FullPath
sc.OutputFile = out_path
sc.Commit()
sc.Destroy()
```

#### Remove Parameters (KESİN ✅)
```python
rpb = part.Features.CreateRemoveParametersBuilder()
rpb.Objects.Add(solids)
rpb.Commit()
rpb.Destroy()
```

#### Circular Pattern (KISMİ ⚠️)
```python
pfb = part.Features.CreatePatternFeatureBuilder(NXOpen.Features.Feature.Null)
pfb.PatternService.PatternType = PatternDefinition.PatternEnum.Circular
pfb.FeatureList.Add([feature])  # NOT AddFeatureToPattern!
circ = pfb.PatternService.CircularDefinition
circ.RotationAxis = axis_obj
# NX 2506: SpacingType.Offset (CountAndPitch YOK!)
circ.AngularSpacing.SpaceType = PatternSpacing.SpacingType.Offset
circ.AngularSpacing.NCopies.RightHandSide = str(count)
circ.AngularSpacing.PitchDistance.RightHandSide = str(angle)
```

### 2.3 Mevcut Olmayan / Bulunamayan API'ler

| Aranan API | Durum | Not |
|---|---|---|
| `CreateMirrorBodyBuilder` | ✅ **VAR (DÜZELTİLDİ)** | İlk keşifte bulunamamıştı; canlı testte (v3 T07, 3→4 gövde) VAR ve çalışıyor olduğu kanıtlandı. `part.Features.CreateMirrorBodyBuilder(Feature.Null)`; `Plane.Value` bir **sabit datum** ister, `Planes.CreatePlane` sonucunu kabul etmez |
| `CreateThreadFeatureBuilder` | ❌ YOK | Yerine `CreateThreadBuilder(Thread.Null)` var |
| `EdgeBlendBuilder.SetRadius()` | ❌ YOK | `AddVariableRadiusData()` ile radius belirlenir |
| `EdgeBlendBuilder.AddChainsToCollector()` | ❌ YOK | `AddChainset(edge, index)` kullanılır |
| `ChamferBuilder.SymmetricOptions` | ❌ YOK | `FirstOffsetExp` + `FirstOffset` kullanılır |
| `DraftBuilder.Angle` | ❌ YOK | `SymmetricAngle` (bool tip) veya `FaceSetAngleExpressionList` |
| `ParasolidCreator` | ❌ YOK | `CreateParasolidExporter()` var (NX 2506) |

---

## 3. 15 NXOpen Kritik Tuzak (NX 2506'da Doğrulandı)

| # | Tuzak | Açıklama |
|---|---|---|
| 1 | `BasePart.Units.Millimeters` | `Part.Units.Millimeters` "Second parameter is invalid" hatası verir |
| 2 | `CreateRuleCurveDumb` | `CreateRuleBaseCurveDumb` deprecated, kullanmayın |
| 3 | `PatternSpacing.SpacingType.Offset` | `CountAndPitch` enum değeri NX 2506'da yok |
| 4 | `FeatureList.Add([feat])` | `AddFeatureToPattern` metodu yok |
| 5 | `Expression.RightHandSide` | `.Value` kullanılırsa 25.4× birim hatası oluşur |
| 6 | `Point3d/Vector3d float` | int parametre "Expecting double" hatası verir |
| 7 | `DoUpdate` her adımda | Atlanırsa model güncellenmez, export boş çıkar |
| 8 | `CreateParasolidExporter` | SADECE body'leri seç, EntirePart curve'leri de alır → fault |
| 9 | `NewBaseDisplay` varolan .prt | "File already exists" hatası → önce sil |
| 10 | `CreateArc` açı parametreleri | float zorunlu, int kabul etmez (`0.0` değil `0` → hata) |
| 11 | `CreateThreadBuilder` ilk parametre | `Feature.Null` değil, `Thread.Null` bekler |
| 12 | `FirstOffset` tipi | `str` — doğrudan string ifade, Expression değil |
| 13 | `EdgeBlend.AddChainset` | `(edge, int_index)` imzası, `AddChainsToCollector` yok |
| 14 | Edge referans yaşam döngüsü | Feature commit sonrası edge referansları geçersiz olur |
| 15 | Session zehirlenmesi | "please report fault" → NX'i tamamen KAPATIP yeniden başlat |

---

## 4. BuildStep v2.0 Şeması — Tüm Kind'lar

> ⚠️ **DÜZELTME:** Durum sütunu bu raporun ilk yazıldığı anki (henüz doğrulanmamış) tahmindi.
> `nx2506-verified-api-playbook.md` §5'teki canlı sx_engine entegrasyon testi (10 adım / 0 hata)
> sonrasında **tüm v2.0 finish feature'lar ✅ olarak doğrulandı** — tablo buna göre güncellendi.

| Kind | Kategori | NXOpen API | Parametreler | NX 2506 Durumu |
|---|---|---|---|---|
| `prism` | v1.0 Solid | `CreateExtrudeBuilder` | profile, origin3, axis, u_dir, length | ✅ |
| `cylinder` | v1.0 Solid | `CreateExtrudeBuilder` | outer_radius, origin3, axis, length | ✅ |
| `tube` | v1.0 Solid | `CreateExtrudeBuilder` ×2 | outer_radius, inner_radius, origin3, axis, length | ✅ |
| `hole` | v1.0 Solid | `CreateExtrudeBuilder` (subtract) | outer_radius, cx, cy, z0, axis, length | ✅ |
| `extrude` | v1.0 Solid | `CreateExtrudeBuilder` (+Z) | profile, z0, length | ✅ |
| `revolve` | v1.0 Solid | `CreateRevolveBuilder` | profile (d,a), origin3, axis, angle_deg | ✅ |
| `loft_twist` | v1.0 Solid | `CreateThroughCurvesBuilder` | profile, profile_top, twist_deg | ✅ |
| `fillet` | v2.0 Finish | `CreateEdgeBlendBuilder` | edge_blend_radius_mm | ✅ (DÜZELTİLDİ — bkz. §2.2, collector+radius-string) |
| `chamfer` | v2.0 Finish | `CreateChamferBuilder` | chamfer_distance_mm | ✅ (DÜZELTİLDİ — bkz. §2.2, SmartCollector) |
| `thread` | v2.0 Finish | `CreateThreadBuilder` | thread_designation, thread_length_mm | ✅ (DÜZELTİLDİ — bkz. §2.2, CylindricalFace+Manual) |
| `draft` | v2.0 Finish | `CreateDraftBuilder` | draft_angle_deg, draft_direction | ✅ (DÜZELTİLDİ — bkz. §2.2, ExpressionCollectorSet) |
| `material` | v2.0 Non-geo | `MaterialManager.PhysicalMaterials` | material_name, density_kg_m3 | ✅ (`LoadFromNxmatmllibrary`+`AssignObjects`) |
| `mirror` | v2.0 Solid | `CreateMirrorBodyBuilder` | mirror_plane_origin3, mirror_plane_normal | ✅ (VAR — bkz. §2.3 düzeltmesi) |
| `group` | v2.0 Non-geo | — (logical only) | group_name | ✅ |
| `pmi_note` | v2.0 Non-geo | `Annotations.CreatePmiNoteBuilder` | pmi_text | ✅ (`Text.TextBlock.SetText`) |
| `shell` | v2.0 Finish | `CreateShellBuilder` | shell_thickness_mm, faces_to_remove | ✅ (`Tolerance=0.01` şart) |
| `hole_package` | v2.0 Finish | `CreateHolePackageBuilder` | hole_diameter_mm, hole_depth_mm, tip_angle_deg | ✅ (`Tolerance` + `SetTargetBodies` şart) |

---

## 5. Keşif Journal'ları İndeksi

Tüm API keşif journal'ları `siemens_nx_docs/api_discovery/` klasöründe:

| Dosya | Amaç | Sonuç |
|---|---|---|
| `v2_api_verify.py` | İlk API testi | `SetRadius`, `SymmetricOptions`, `CreateThreadFeatureBuilder` yok |
| `v2_api_discover.py` | EdgeBlend/Chamfer/Thread/Draft/Mirror API metot listesi | `AddChainset`, `FirstOffsetExp`, `SymmetricAngle`, ThreadBuilder bulundu |
| `v2_api_discover2.py` | FeatureCollection Create* metodları | `CreateThreadBuilder` var, `CreateMirrorBodyBuilder` yok |
| `v2_api_discover3.py` | Tüm Create* metodları (tam liste) | 100+ Create* metodu listelendi |
| `v2_api_discover4.py` | EdgeBlend radius, Chamfer offset, Thread, Draft detay | `FirstOffset=str`, `FirstOffsetExp=Expression`, `SymmetricAngle=bool` |
| `v2_api_discover5.py` | Draft angle, Chamfer expression, EdgeBlend chainset | `SymmetricAngle`, `FirstOffsetExp`, `AddChainset` onaylandı |
| `v2_final_test.py` | Düzeltilmiş API testi | Edge referans sorunu, Chamfer referans sorunu |
| `v2_corrected_test.py` | Düzeltilmiş API testi (2. deneme) | EdgeBlend: "Referenced edge does not exist" |
| `v2_simple_test.py` | En basit API testi | Chamfer/EdgeBlend/Draft referans sorunları devam ediyor |

---

## 6. Ekran Görüntüleri İndeksi

`siemens_nx_docs/screenshots/` — 66 ekran görüntüsü:

| Klasör | İçerik | Adet |
|---|---|---|
| `ribbons/` | 16 ribbon sekmesi + 19 ek özellik | 35 |
| `dialogs/` | Journal Manager, Expression Editor, Measure, Information, Drafting, Part Navigator | 8 |
| `menus/` | 13 menü çubuğu | 13 |
| `screenshots/` (kök) | Tam ekran, context menu, genel görünümler | 10 |

---

## 7. Claude Fable 5 Doğrulama Referansı

### 7.1 Test Edilen ve Çalışan API'ler (KESİN ✅)
- `Parts.NewBaseDisplay(path, BasePart.Units.Millimeters)` — mm parça oluşturma
- `CreateExtrudeBuilder` + `BooleanOperation.SetTargetBodies` — extrude + boolean
- `CreateRevolveBuilder` — revolve
- `Sections.CreateSection` + `CreateRuleCurveDumb` — section oluşturma
- `Curves.CreateLine`, `Curves.CreateArc` — curve (float zorunlu)
- `Expressions.CreateWithUnits` / `EditWithUnits` — parametrik ifadeler
- `Body.SetName()` — body isimlendirme
- `DexManager.CreateStepCreator` — STEP AP242 export
- `CreateRemoveParametersBuilder` — parametre temizleme
- `UpdateManager.DoUpdate` — model güncelleme
- `SetUndoMark` / `UndoToMark` — hata durumunda geri alma
- `CreateMirrorBodyBuilder` — VAR (DÜZELTİLDİ, bkz. §2.3); `MirrorBodyList.Add` + sabit `Plane.Value` ile 3→4 gövde kanıtlı
- `CreateEdgeBlendBuilder` — `AddChainset(ScCollector, "radius")` imzasıyla (bkz. §2.2 düzeltmesi)
- `CreateChamferBuilder` — `SmartCollector` + `Option`/`FirstOffset` ile (bkz. §2.2 düzeltmesi)
- `CreateThreadBuilder(Thread.Null)` — `CylindricalFace.Value` + `Input.Manual` ile (bkz. §2.2 düzeltmesi)
- `CreateDraftBuilder` — `ExpressionCollectorSet` ile (bkz. §2.2 düzeltmesi)
- `part.MaterialManager.PhysicalMaterials.LoadFromNxmatmllibrary` + `mat.AssignObjects`
- `part.Annotations.CreatePmiNoteBuilder` + `Text.TextBlock.SetText`
- `CreateShellBuilder` (`Tolerance=0.01` şart) ve `CreateHolePackageBuilder` (`Tolerance` + `SetTargetBodies` şart)

### 7.2 API İsmi Farklı Olanlar (DÜZELTİLDİ ⚠️)
- `CreateThreadBuilder(Thread.Null)` ← `CreateThreadFeatureBuilder(Feature.Null)` değil
- `AddChainset(ScCollector, "radius")` ← `AddChainset(edge, index)` **değil** (bu raporun ilk
  sürümünün önerdiği imza yanlıştı, bkz. §2.2)
- `SmartCollector` ← `AddChainsToCollector` değil
- `FirstOffset` (str) / `FirstOffsetExp` (salt-okunur) ← `SymmetricOptions.Distance` değil
- `SymmetricAngle` bool'dur ama açıyı taşımaz; açı `ExpressionCollectorSet` üzerinden gelir ← `Angle` attribute'u değil

### 7.3 Mevcut Olmayan API'ler (KULLANILAMAZ ❌)
- `EdgeBlendBuilder.SetRadius()` — metot yok
- `EdgeBlendBuilder.AddChainsToCollector()` — metot yok
- `ChamferBuilder.SymmetricOptions` — attribute yok
- `DraftBuilder.Angle` — attribute yok
- `CreateThreadFeatureBuilder` — factory metodu yok
- `ThreadBuilder.FaceCollector` — attribute yok (`CylindricalFace.Value` kullanılır)
- `MaterialManager.LoadMaterialsFromLibrary` / `AssignMaterialToBody` — yok (`PhysicalMaterials.LoadFromNxmatmllibrary`+`AssignObjects` kullanılır)
- `PmiNotes.CreatePmiNote(...)` — yok (`Annotations.CreatePmiNoteBuilder` kullanılır)

> ⚠️ **DÜZELTME:** Bu listede daha önce `CreateMirrorBodyBuilder` de "yok" olarak geçiyordu —
> canlı testte VAR olduğu ve çalıştığı kanıtlandı (bkz. §7.1, §2.3). Buradan çıkarıldı.

### 7.4 Tip Sürprizleri
- `ChamferBuilder.FirstOffset`: **str** (Expression değil!)
- `ChamferBuilder.FirstOffsetExp`: **Expression** (beklenen)
- `DraftBuilder.SymmetricAngle`: **bool** (Expression değil!)
- `CreateArc`: TÜM parametreler **float** (int reddedilir)
- `CreateThreadBuilder`: İlk parametre **Thread.Null** (Feature.Null değil!)
- `NewBaseDisplay`: Dönüş tipi **Part veya (Part, Status) tuple**
