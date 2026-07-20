# NX 2506 — DOĞRULANMIŞ API EL KİTABI (Claude Fable 5, Canlı Kanıtlı)

> **Tarih:** 2026-07-03 · **NX:** 2506, `E:\program\NXBIN` · **Rol:** "Designcenter" arayüzü
> **Yöntem:** 3 paralel stub-madenciliği (`E:\program\UGOPEN\pythonStubs`) + 5 canlı headless
> doğrulama journal'ı (`api_discovery/v3_master_verify.py` → `v3e_draft_thread.py`) +
> canlı GUI ekran görüntüleri (`screenshots/live_20260703/`)
> **Sonuç:** 12/12 hedef özellik NX'te ÇALIŞIR reçeteyle kanıtlandı; sx_engine builder'a işlendi
> ve entegrasyon testi **10 adım / 0 hata** ile geçti (`test_v2_finish.json`).
>
> Bu doküman `nx2506-comprehensive-discovery.md`'nin (DeepSeek v4 Pro keşfi) **düzeltilmiş ve
> kanıtlı** halefidir. Çelişki durumunda BU doküman geçerlidir.
>
> **Güncelleme (2026-07-20):** §8.7 eklendi + §8.9-8.18 (Desktop'taki 4 UCAV journal'ının —
> `aircraft_parametric_nx.cs` v2, `ucav_delta_nx.cs`, `ucav_pro_suite_nx.cs`, `ucav_ttail_nx.cs` —
> karşılaştırmalı harmanlanmasından, sadece YENİ/ortak-olmayan kalıplar) + yeni §9 (Boolean
> Intersect hacim doğrulama, önceden yalnız `askeri/*` fork'larında yaşıyordu).
>
> **Güncelleme 2 (2026-07-20):** Bu klasör artık dokümanların **TEK gerçek/yaşayan kopyası**
> ve bu repo (`ev-ipmsm-nx`, private) üzerinden git ile versiyonlanıyor. Daha önce `motor/`,
> `askeri/sentinel_ugv/`, `askeri/ucav_nx/`, `tarımProjeleri/` olmak üzere 4 ayrı tam-kopya
> senkronize tutulmaya çalışılıyordu — bu, "tek yer" değildi ve tekrar ayrışma riski taşıyordu.
> O 3 konumdaki tam kopyalar artık SİLİNDİ; yerlerine buraya işaret eden birer kısa yönlendirme
> notu bırakıldı. **Tüm düzenlemeler SADECE burada yapılır.**

---

## 0. DeepSeek Dokümanına Kritik Düzeltmeler (kanıtla çürütüldü/düzeltildi)

| # | DeepSeek iddiası | GERÇEK (NX 2506'da canlı doğrulandı) | Kanıt |
|---|---|---|---|
| 1 | `AddChainset(edge, index)` | **`AddChainset(ScCollector, "radius_str")`** — edge değil collector, index değil yarıçap string'i! DeepSeek'in tüm EdgeBlend başarısızlıklarının kökü buydu | v3 T02: BLEND, 8→12 yüz |
| 2 | `AddVariableRadiusData(idx, radius)` ile sabit yarıçap | O metot değişken-yarıçap içindir ve `(Edge, float, str, Point, bool)` imzalıdır; sabit yarıçap chainset string'inden gelir | stub 19653 |
| 3 | `CreateMirrorBodyBuilder` ❌ YOK | **VAR** (`FeatureCollection`, stub 26463) ve çalışıyor: `MirrorBodyList.Add(body)` + `Plane.Value = sabit DATUM düzlemi` | v3 T07: MIRROR, 3→4 gövde |
| 4 | `ChamferBuilder.AddChainsToCollector([edge])` | O metot **yok**; kenarlar `SmartCollector` property'sine ScCollector olarak atanır; stil `Option = ChamferOption.SymmetricOffsets`; mesafe `FirstOffset = "2"` (str) | v3 T03: CHAMFER, 12→13 yüz |
| 5 | `ThreadBuilder.FaceCollector.Add([face])` | `FaceCollector` **yok**; yüz `CylindricalFace.Value = face` (SelectDisplayableObject); tip enum'u `ThreadBuilder.Type.Symbolic` (`ThreadBuilderType` diye sınıf yok) | v3e T05: SYMBOLIC_THREAD |
| 6 | `DraftBuilder.SymmetricAngle.Value.RightHandSide` ile draft | `SymmetricAngle` **bool**'dur; açı `ExpressionCollectorSet` üzerinden verilir (aşağıda tam reçete) | v3e T04: DRAFT |
| 7 | `MaterialManager.LoadMaterialsFromLibrary` / `AssignMaterialToBody` | İkisi de **yok**. Doğrusu: `part.MaterialManager.PhysicalMaterials.LoadFromNxmatmllibrary("Steel")` → `mat.AssignObjects([body])` | v3 T08 |
| 8 | `PmiNotes.CreatePmiNote(...)` | **Yok**. Doğrusu: `part.Annotations.CreatePmiNoteBuilder(None)` → `b.Text.TextBlock.SetText([satırlar])` → `b.Origin.Origin.SetValue(...)` → `Commit()` | v3b T10: PmiNote |
| 9 | Ribbon = 16 sekme (PMI, Drafting, Sheet Metal, CAM, Simulation…) | Bu kurulumdaki canlı arayüz ("Designcenter" rolü) **13 sekme**: File, Home, Curve, Surface, Assemblies, Analysis, View, Display, Selection, Tools, Application, **Mold Wizard** (+İnternal Discovery Center paneli). PMI/Drafting/CAM sekmeleri bu rolde kapalı | `ribbon_*_VERIFIED.png` |
| 10 | — (hiç bahsedilmemiş) | `Tools` sekmesindeki "Record" **Movie kaydıdır** (ekran videosu), journal kaydı değil; bu rolde Journal düğmeleri ribbon'da yok → journal'lar `run_journal.exe` (headless) veya Alt+F8 (GUI oynatma) ile | `ribbon_Tools_VERIFIED.png` |

---

## 1. Canlı GUI Envanteri (2026-07-03, ekran görüntülü)

**Pencere:** "Designcenter Modeling", boş `model1.prt`, Part Navigator + Discovery Center.
**Sekmeler (soldan sağa):** File · Home · Curve · Surface · Assemblies · Analysis · View ·
Display · Selection · Tools · Application · Mold Wizard

| Sekme | Canlı içerik (görüntüden okundu) | Dosya |
|---|---|---|
| Home | Datum Plane, Sketch · Extrude, Revolve, Hole, Unite, Subtract, Edge Blend, Chamfer, Draft, Shell · Pattern Feature, **Mirror Feature** · Synchronous: Move, Delete, Replace, Offset, Resize Blend | `ribbon_Home_VERIFIED.png` |
| Surface | Extrude, Swept, Through Curves, Through Curve Mesh, Edge Blend, Face Blend, Offset Surface, Thicken · Combine, Trim Sheet, Extend Sheet, Sew, Trim and Extend | `ribbon_Surface_VERIFIED.png` |
| Analysis | Measure (tek büyük komut — Ctrl+M ölçüm kabuğu) | `ribbon_Analysis_VERIFIED.png` |
| Display | Style, Face Edges, Hidden Edges · Edit Object Display · **Assign Visual Materials** · Background | `ribbon_Display_VERIFIED.png` |
| Tools | **Assign Materials**, Expressions, Raster Image · Movie: **Record/Pause/Stop**, Settings · Reuse Library: Fastener Assembly | `ribbon_Tools_VERIFIED.png` |
| Mold Wizard | Initialize Project, Mold CSYS, Mold Base Library, Design Fill, Pocket, BOM · Parting: Check Regions, Patch Surface, Define Cavity and Core · Cooling · Mold Tools: Bounding Body, Stock Size, Split Body, Extend Sheet · Tooling Validation: Preprocess Motion, Run Simulation · Mold Drawing | `ribbon_MoldWizard_VERIFIED.png` |

**Journal çalıştırma yolları (bu kurulumda doğrulandı):**
1. **Headless:** `E:\program\NXBIN\run_journal.exe <journal.py> -args ...` — GUI açıkken bile
   İKİNCİ oturum sorunsuz açılıyor (lisans çakışması YOK; bu oturumda 8+ kez koşuldu).
2. **GUI'de oynatma:** Alt+F8 (Journal Play) — bu rolde ribbon'da Journal grubu görünmüyor.
3. GUI Journal Record bu rolün ribbon'unda yok (Tools'taki Record = Movie/video). Kayıt için
   rol değiştirme/Customize gerekir; pratik yol zaten stub + headless doğrulamadır.

**UI otomasyonu dersi (delil toplarken yaşandı):** NX ribbon'u UIA'ya TabItem olarak açılmıyor;
%125 DPI sanallaştırması SetCursorPos/GetWindowRect eşleşmesini bozuyor → piksel tıklama
güvenilmez. Pencere görüntüsü almak güvenli (CopyFromScreen/PrintWindow), tıklamadan kaçın.

---

## 2. DOĞRULANMIŞ REÇETELER (kopyala-kullan; hepsi canlı NX 2506'da koştu)

Ortak önkoşullar:
```python
import NXOpen, NXOpen.Features, NXOpen.GeometricUtilities, NXOpen.Annotations
ses  = NXOpen.Session.GetSession()
part = ses.Parts.NewBaseDisplay(path, NXOpen.BasePart.Units.Millimeters)  # tuple dönebilir
```

### 2.1 Edge Blend (fillet) — kanıt: BLEND, 8→12 yüz
```python
ebb  = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
col  = part.ScCollectors.CreateCollector()
rule = part.ScRuleFactory.CreateRuleEdgeDumb(edges)      # List[Edge]
col.ReplaceRules([rule], False)
ebb.AddChainset(col, "4")                                # yarıçap STRING ifade!
feat = ebb.CommitFeature(); ebb.Destroy()
```

### 2.2 Chamfer — kanıt: CHAMFER, 12→13 yüz
```python
cb   = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
col  = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleEdgeDumb(edges)], False)
cb.SmartCollector = col
cb.Option      = NXOpen.Features.ChamferBuilder.ChamferOption.SymmetricOffsets
cb.FirstOffset = "2"                                     # str; FirstOffsetExp okunur-yalnız
feat = cb.CommitFeature(); cb.Destroy()
```

### 2.3 Draft (Face) — kanıt: DRAFT feature'ı
```python
db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
db.AngleTolerance    = 0.5          # ŞART: 0 kalırsa "Angle tolerance is too small"
db.DistanceTolerance = 0.001
db.TypeOfDraft = NXOpen.Features.DraftBuilder.Type.Face
db.DraftReferencesMethod = NXOpen.Features.DraftBuilder.DraftReferencesMethods.StationaryFace
db.Direction = part.Directions.CreateDirection(o, z, NXOpen.SmartObject.UpdateOption.WithinModeling)
db.StationaryReference.ReplaceRules(
    [part.ScRuleFactory.CreateRuleFaceDumb([sabit_yüz])], False)   # StationaryEntity DEĞİL
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb(eğilecek_yüzler)], False)
ecs = part.CreateExpressionCollectorSet(col, "3", "", 0)  # birim argümanı BOŞ STRING olmalı
                                                          # ("Degrees"/"deg" → invalid unit!)
db.FaceSetAngleExpressionList.Append(ecs)                 # Append yalnız ECS kabul eder
feat = db.CommitFeature(); db.Destroy()
```

### 2.4 Symbolic Thread — kanıt: SYMBOLIC_THREAD
```python
tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)  # Thread.Null!
tb.ThreadType  = NXOpen.Features.ThreadBuilder.Type.Symbolic
tb.ThreadInput = NXOpen.Features.ThreadBuilder.Input.Manual          # tablo yolu kırılgan
tb.ShaftPreference = NXOpen.Features.ThreadBuilder.ShaftSizePreference.MajorDiameter
tb.CylindricalFace.Value = silindirik_yüz
tb.StartObject.Value     = başlangıç_düzlem_yüzü   # gerekli; geçersizse başka aday dene
tb.MajorDiameterExp.RightHandSide = "20"
tb.MinorDiameterExp.RightHandSide = "17.5"          # metrik: major − 1.0825×pitch
tb.ShaftDiameterExp.RightHandSide = "18.75"         # minor < shaft < major OLMALI
tb.PitchExp.RightHandSide = "2.5"; tb.AngleExp.RightHandSide = "60"
tb.ThreadLength.RightHandSide = "20"
feat = tb.CommitFeature(); tb.Destroy()
```
Notlar: (a) `NX_Thread_Standard.xml` mevcut ama ThreadTable yolu headless'ta "Standard data
not found" veriyor → **Manual** kullan. (b) Pah (chamfer) silindir-üst kenarını yerse komşu
başlangıç yüzü geçersizleşir → **diş adımını pahtan ÖNCE** uygula ya da builder'daki çok-adaylı
StartObject denemesini kullan (sx_engine bunu yapar). (c) Başarısız Commit builder'ı kirletir —
her denemede builder'ı YENİDEN yarat.

### 2.5 Shell — kanıt: SHELL, hacim 64000→~14000 mm³
```python
sb = part.Features.CreateShellBuilder(NXOpen.Features.Feature.Null)
sb.Tolerance = 0.01                    # ŞART: 0 kalırsa "Tolerance error"
sb.Body = body
sb.SetDefaultThickness("2")
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb([kaldırılacak_yüz])], False)
sb.RemovedFacesCollector = col
feat = sb.CommitFeature(); sb.Destroy()
```

### 2.6 Mirror Body — kanıt: MIRROR, 3→4 gövde
```python
m = NXOpen.Matrix3x3()                 # satırlar: X, Y, Z ekseni; Z = düzlem NORMALİ
m.Xx,m.Xy,m.Xz = 0.0,1.0,0.0
m.Yx,m.Yy,m.Yz = 0.0,0.0,1.0
m.Zx,m.Zy,m.Zz = 1.0,0.0,0.0
dp = part.Datums.CreateFixedDatumPlane(NXOpen.Point3d(-30.0,0.0,0.0), m)  # SABİT DATUM şart
mb = part.Features.CreateMirrorBodyBuilder(NXOpen.Features.Feature.Null)
mb.MirrorBodyList.Add(body)
mb.Plane.Value = dp                    # SelectDatumPlane → Planes.CreatePlane KABUL ETMEZ
feat = mb.CommitFeature(); mb.Destroy()
yeni_gövde = feat.GetBodies()[0]
```

### 2.7 Hole Package — kanıt: HOLE PACKAGE, 8→9 yüz
```python
hp = part.Features.CreateHolePackageBuilder(NXOpen.Features.HolePackage.Null)
hp.HoleType = NXOpen.Features.HolePackageBuilder.Holetype.Simple
hp.GeneralSimpleHoleDiameter.SetFormula("8")
hp.HoleDepthLimitOption = NXOpen.Features.HolePackageBuilder.HoleDepthLimitOptions.Value
hp.GeneralSimpleHoleDepth.SetFormula("40")      # ThroughBody yolu "Tolerance Specification
hp.GeneralTipAngle.SetFormula("118")            #  requires three numbers" hatası verdi →
hp.Tolerance = 0.01                             #  Value + derinlik + uç açısı + Tolerance
sec = hp.HolePosition
sec.SetAllowedEntityTypes(NXOpen.Section.AllowTypes.OnlyPoints)
pt   = part.Points.CreatePoint(NXOpen.Point3d(175.0,10.0,30.0))
rule = part.ScRuleFactory.CreateRuleCurveDumbFromPoints([pt])
sec.AddToSection([rule], pt, NXOpen.NXObject.Null, NXOpen.NXObject.Null,
                 NXOpen.Point3d(175.0,10.0,30.0), NXOpen.Section.Mode.Create, False)
hp.BooleanOperation.SetTargetBodies([body])     # UNUTMA: yoksa "Missing target body"
feat = hp.CommitFeature(); hp.Destroy()
```

### 2.8 Malzeme atama — kanıt: Steel yüklendi, kütle 1.1847 kg (ρ=7.83 doğru)
```python
pm  = part.MaterialManager.PhysicalMaterials
mat = pm.LoadFromNxmatmllibrary("Steel")        # NX MatML kütüphanesinden
mat.AssignObjects([body])                       # atama MATERYAL nesnesinde, koleksiyonda değil
# aynı malzemeyi ikinci kez yükleme: pm.GetLoadedLibraryMaterial("physicalmateriallibrary.xml","Steel")
```

### 2.9 Kütle özellikleri ölçümü — kanıt: V=151324.66 mm³, A=19258.49 mm², m=1.1847 kg
```python
uc = part.UnitCollection
units = [uc.FindObject(n) for n in
         ("SquareMilliMeter","CubicMilliMeter","Kilogram","MilliMeter","Newton")]  # 5'li liste
mb = part.MeasureManager.NewMassProperties(units, 0.99, [body])   # accuracy 0.99
hacim, alan, kütle, merkez = mb.Volume, mb.Area, mb.Mass, mb.Centroid
```

### 2.10 PMI Not — kanıt: PmiNote nesnesi oluştu
```python
nb = part.Annotations.CreatePmiNoteBuilder(NXOpen.Annotations.SimpleDraftingAid.Null)
nb.Text.TextBlock.SetText(["SATIR 1", "SATIR 2"])   # Text'in kendisinde SetText YOK;
                                                    # TextBlock'unda var (SetEditorText de var)
nb.Origin.Origin.SetValue(NXOpen.TaggedObject.Null, part.Views.WorkView,
                          NXOpen.Point3d(0.0,0.0,80.0))
note = nb.Commit(); nb.Destroy()
```

### 2.11 Headless PNG görüntü — SONUÇ: **MÜMKÜN DEĞİL** (kesinleşti)
- `part.Views.CreateImageExportBuilder()` → `Commit` = "Invalid object state"
- `uf.Disp.CreateImage(png, DispImageFormat.PNG, DispBackgroundColor.WHITE)` = "The image
  file could not be created"
→ run_journal.exe altında grafik penceresi yok. PNG almak İNTERAKTİF NX ister
(nx_journal.py'daki `views` opsiyonunun GUI'de çalışması bundandır). Delil görüntüsü işi
Windows tarafında pencere yakalama ile çözülür.

---

## 3. Stub Madenciliği — Profesyonel Yetenek Envanteri (imza-doğrulanmış)

### 3.1 FeatureCollection'da işe yarayacak diğer fabrikalar (tam liste stub'da)
`CreateBlockFeatureBuilder, CreateCylinderBuilder, CreateConeBuilder, CreateSphereBuilder,
CreateTubeBuilder, CreateHelixBuilder, CreateSweptBuilder, CreateRuledBuilder,
CreateThroughCurvesBuilder, CreateThroughCurveMeshBuilder, CreateStudioSplineBuilderEx,
CreateFitCurveBuilder, CreateTextBuilder, CreateThickenBuilder, CreateSewBuilder,
CreateTrimBody2Builder, CreateSplitBodyBuilderUsingCollector, CreateDeleteFaceBuilder,
CreateMoveObjectBuilder, CreateScaleBuilder, CreatePatternGeometryBuilder,
CreateExtractFaceBuilder (gövde kopyalama), CreateWaveLinkBuilder (WAVE link),
CreateBooleanBuilderUsingCollector, CreateDatumPlaneBuilder, CreateDatumCsysBuilder,
CreateHoleFeatureBuilder (legacy delik), CreateEmbossBuilder, CreateOffsetSurfaceBuilder ...`

### 3.2 İki gövdeyi ayrı feature ile birleştir/çıkar (BooleanBuilder)
```python
bb = part.Features.CreateBooleanBuilderUsingCollector(NXOpen.Features.BooleanFeature.Null)
bb.Operation = NXOpen.Features.Feature.BooleanType.Subtract
bb.Target = hedef; bb.Tool = takım          # veya Target/ToolBodyCollector
feat = bb.CommitFeature()
```
Kısa yol (builder'sız): `part.Features.CreateUniteFeature(target, keep_t, [tools], keep_tools, allow_nonassoc)`.

### 3.3 Gövdeyi parametrik taşı/döndür (MoveObjectBuilder + ModlMotion)
```python
mo = part.Features.CreateMoveObjectBuilder(NXOpen.Features.MoveObject.Null)
mo.ObjectToMoveObject.Add(body)
mo.TransformMotion.Option = NXOpen.GeometricUtilities.ModlMotion.Options.DeltaXyz
mo.TransformMotion.DeltaX = 25.0; mo.TransformMotion.DeltaY = 0.0; mo.TransformMotion.DeltaZ = 10.0
mo.MoveObjectResult = NXOpen.Features.MoveObjectBuilder.MoveObjectResultOptions.CopyOriginal  # kopya
mo.Commit(); mo.Destroy()
# Döndürme: Option=Angle + AngularAxis (Axis) + Angle (Expression)
```

### 3.4 Montaj: bileşen ekleme + kısıt
```python
comp, status = part.ComponentAssembly.AddComponent(
    "C:/parts/wheel.prt", "MODEL", "WHEEL_FL", NXOpen.Point3d(0,0,0), orient_m3x3, 1)
pos = part.ComponentAssembly.Positioner
pos.BeginAssemblyConstraints()
c = pos.CreateConstraint(True)
c.ConstraintType = NXOpen.Positioning.Constraint.Type.Touch    # "Align" yok:
c.ConstraintAlignment = NXOpen.Positioning.Constraint.Alignment.CoAlign  # Touch+CoAlign=Align
c.CreateConstraintReference(comp,  geo1, False, False)
c.CreateConstraintReference(comp2, geo2, False, False)
c.SetExpression("10")                                          # Distance kısıtında
pos.EndAssemblyConstraints()
# Bileşen taşıma: ComponentAssembly.MoveComponent(comp, Vector3d, Matrix3x3)
```

### 3.5 Export envanteri (DexManager.Create*)
STEP (`CreateStepCreator`: Ap203/Ap214/**Ap242**/Ap242ED2), Parasolid, IGES, ACIS, CATIA v4/v5,
DXF/DWG, STL, 3MF, OBJ, IFC, USDZ, NXTo2d... **JT farklı yerde:**
`ses.PvtransManager.CreateJtCreator()` (DexManager'da CreateJtCreator YOK).

### 3.6 Oturum / parça yaşam döngüsü (çok-parça build'lerde)
```python
part.Close(NXOpen.BasePart.CloseWholeTree.TrueValue,
           NXOpen.BasePart.CloseModified.CloseModified, None)
ses.Parts.CloseAll(NXOpen.BasePart.CloseModified.CloseModified, None)
mark = ses.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "adım")
ses.UpdateManager.DoUpdate(mark)          # her feature sonrası ŞART
ses.UpdateManager.AddToDeleteList(objs)   # + DoUpdate = güvenli silme
```

### 3.7 Spiral/yay ve serbest eğri
- `CreateHelixBuilder`: SizeOption(Diameter/Radius), PitchLaw/SizeLaw (LawBuilder),
  NumberOfTurns (str), TurnDirection(RightHand/LeftHand), CoordinateSystem.
- `CreateStudioSplineBuilderEx(None)`: `Type=ThroughPoints/ByPoles`, `Degree`, noktalar
  `ConstraintManager.CreateGeometricConstraintData()` → `.Point = part.Points.CreatePoint(...)`
  → `Append`. (CurveCollection'da CreateSpline YOK.)

### 3.8 Gövde rengi (görsel ayrım)
```python
dm = ses.DisplayManager.NewDisplayModification()
dm.NewColor = 186; dm.ApplyToAllFaces = True
dm.Apply([body]); dm.Dispose()
```

### 3.9 Nitelik etiketleme (üretim meta-verisi)
```python
body.SetUserAttribute("PART_NO", -1, "SNT-4x4-0042", NXOpen.Update.Option.Now)
```

---

## 4. GÜNCEL TUZAK LİSTESİ (DeepSeek'in 15'i + canlı keşfedilen 10 yeni)

Eski 15 tuzaktan hâlâ geçerli olanlar: `BasePart.Units.Millimeters`, `CreateRuleCurveDumb`,
`PatternSpacing Offset`, `FeatureList.Add`, `RightHandSide` (.Value 25.4×), float zorunluluğu,
`DoUpdate` şartı, Parasolid gövde-seçimi, varolan .prt silme, `CreateArc` float, `Thread.Null`,
`FirstOffset=str`, oturum zehirlenmesi (→ NX restart). **Düzeltilenler:** #13 AddChainset
imzası yanlıştı (bkz. §0.1), #14 "edge referansı geçersiz olur" aslında yanlış imza hatasıydı.

Yeni doğrulanan tuzaklar:

| # | Tuzak | Belirti → Çözüm |
|---|---|---|
| 16 | `DraftBuilder.AngleTolerance` default'u geçersiz | "Angle tolerance is too small" → `AngleTolerance=0.5` ata |
| 17 | `ShellBuilder.Tolerance` default'u geçersiz | "Tolerance error" → `Tolerance=0.01` ata |
| 18 | `HolePackage` ThroughBody yolu | "Tolerance Specification requires three numbers" → `Value`+derinlik+uç açısı+`Tolerance=0.01` |
| 19 | `HolePackage` hedef gövde | "Missing target body" → `BooleanOperation.SetTargetBodies` çağır |
| 20 | `CreateExpressionCollectorSet` birim argümanı | "invalid unit measure" → birim **boş string** `""` ver |
| 21 | Thread tablo yolu (headless) | "Standard data not found" → `Input.Manual` + Exp'ler |
| 22 | Thread StartObject | "missing target face"/"Invalid thread start face" → komşu düzlemsel yüz; pahtan ÖNCE diş aç; adayları sırayla dene |
| 23 | Başarısız Commit builder'ı kirletir | Aynı builder'la ikinci Commit denemesi yapma → yeni builder yarat |
| 24 | Fonksiyon içi `import NXOpen.X` | Python scoping: `NXOpen` TÜM fonksiyonda yerel olur → "cannot access local variable 'NXOpen'". Alt-modül importlarını modül başına koy |
| 25 | Headless görüntü | ImageExportBuilder + UF.Disp.CreateImage ikisi de headless'ta çalışmaz → PNG yalnız GUI |

---

## 5. sx_engine Builder Entegrasyonu (bu oturumda işlendi ve test edildi)

`sx_engine/nx_builder.py` v2 kolları doğrulanmış reçetelerle yeniden yazıldı:

| Kind | Parametreler | Durum |
|---|---|---|
| `fillet` | `edge_blend_radius_mm`, opsiyonel `edge_min_len_mm`/`edge_max_len_mm` filtresi | ✅ test geçti |
| `chamfer` | `chamfer_distance_mm`, aynı kenar filtresi | ✅ |
| `thread` | `thread_designation` ("M20x2.5"), `thread_length_mm`; StartObject çok-adaylı | ✅ |
| `draft` | `draft_angle_deg`, opsiyonel `stationary_z_mm` | ✅ |
| `mirror` | `mirror_plane_origin3`, `mirror_plane_normal`; yeni gövde register edilir | ✅ |
| `material` | `material_name` (NX MatML adı, örn. "Steel") | ✅ |
| `pmi_note` | `pmi_text` (çok satır `\n`), `pmi_origin3` | ✅ |

Entegrasyon kanıtı: `scratchpad/test_v2_finish.json` → **10 adım, 0 hata**, STEP AP242 çıktı.
Ek düzeltmeler: hata listesi artık build log'una dökülüyor (`ERR ...` satırları); material
adımı hedef gövdeyi yeniden ADLANDIRMAZ; mirror'da `create+target` sahte uyarısı kaldırıldı.

**Tasarım kuralı (yeni):** dişli boss'larda adım sırası *thread → chamfer* (pah, diş başlangıç
yüzünü tüketmesin).

---

## 6. Kanıt Dosyaları İndeksi

| Dosya | İçerik |
|---|---|
| `api_discovery/v3_master_verify.py` + `v3_verify_report.json` | 12 testlik ana doğrulama (6 PASS ilk turda) + kütle/yüz sayıları |
| `api_discovery/v3b_fix_verify.py` + `v3b_verify_report.json` | Shell ✅, PMI ✅ + draft/thread/hole introspeksiyon dökümleri |
| `api_discovery/v3c_final_verify.py` + `v3c_verify_report.json` | Hole ilerleme, UF görüntü enum üyeleri (PNG/WHITE) |
| `api_discovery/v3d_last_verify.py` + `v3d_verify_report.json` | HOLE PACKAGE ✅ |
| `api_discovery/v3e_draft_thread.py` + `v3e_verify_report.json` | DRAFT ✅ + SYMBOLIC_THREAD ✅ |
| `api_discovery/v3*_verify_part.prt` | Feature'ların gerçekten oluştuğu NX parçaları |
| `screenshots/live_20260703/ribbon_*_VERIFIED.png` | Canlı GUI (içerik-doğrulanmış 6 ribbon) |
| `screenshots/live_20260703/_raw_unverified/` | Etiketi doğrulanamayan ham yakalamalar |
| scratchpad `test_v2_finish.json` + `test_v2_out.prt/.stp` | Builder entegrasyon testi (10/10) |

**Doğrulanmış fizik kanıtı:** 80×60×30 blok + Ø20×25 boss: V=151 853.98 mm³ (analitik:
144 000+7 853.98=151 853.98 ✓ birebir); blend+chamfer sonrası V=151 324.66 mm³ (malzeme
kaybı mantıklı); Steel atandıktan sonra m=1.1847 kg → ρ=7.829 g/cm³ = NX Steel ✓.

---

## 7. SKETCH → SOLID ÜRETİM İŞ AKIŞI (GUI journal'larından harvest)

> **Kaynak:** `journal1/2/3.cs` — NX 2506'da interaktif GUI oturumu kaydı (Tools→Journal→Record).
> **Provenance etiketi:** her reçeteye durum eklendi:
> **[C]** = journal'da GUI'de gerçekten **Commit** edildi (imza + davranış kanıtlı);
> **[S]** = imza doğru, GUI'de kuruldu ama **undo/Destroy** edildi (sonuç doğrulanmadı, dikkatli kullan).
> Journal'lar C#; API adları Python ile **birebir** aynı, yalnız sözdizimi farklı. Aşağısı Python idiomunda.
> Journal gürültüsü (zoom/rotate/FindIssues/FindMovableObjects/undo-mark dansı) **atılmalı** — bunlar
> GUI-only, build'e girmez. Section 1-6 solid-feature'a odaklıydı; bu bölüm onun bir katman altını
> (profili nasıl çizip feature'a beslersin) kapatır.

### 7.0 Kritik sketch önkoşulu (yoksa geometri kayar)
```python
p = ses.Preferences.Sketch
p.CreateInferredConstraints  = False      # ŞART: scriptli sketch için
p.ContinuousAutoDimensioning = False      # yoksa NX otomatik dim/constraint atıp koordinatı bozar
p.DimensionLabel = NXOpen.Preferences.SketchPreferences.DimensionLabelType.Expression
```
Sketch içi referanslar: `ActiveSketch.FindObject("XAxis"|"YAxis")` → `InfiniteLine`, `"Origin"` → `Point`.

### 7.1 Parametrik OFFSET datum düzlemi — [C]
```python
db  = part.Features.CreateDatumPlaneBuilder(NXOpen.Features.Feature.Null)
pl  = db.GetPlane()
pl.SetMethod(NXOpen.PlaneTypes.MethodType.Distance)
yz  = part.Datums.FindObject("DATUM_CSYS(0) YZ plane")   # ((DatumPlane))
pl.SetGeometry([yz])
pl.Expression.RightHandSide = "5"          # offset (ifade → parametrik)
pl.SetAlternate(NXOpen.PlaneTypes.AlternateType.One); pl.Evaluate()
db.SetCornerPoints(c1,c2,c3,c4)            # try/except AssertErrorCode(670309) "undefinable"
db.ResizeDuringUpdate = True
feat = db.CommitFeature()
datumPlane = ((NXOpen.Features.DatumPlaneFeature)feat).DatumPlane
# İkinci offset kopyası: yeni builder + db2.OffsetInstance = True; pl.RemoveOffsetData() ile sıfırla
```

### 7.2 Yüz/kenar üzerine IN-PLACE sketch (CSYS'i geometriden türet) — [C]
```python
# 1) Xform: face/datumPlane + X-yön + nokta → CSYS
pt   = part.Points.CreatePoint(edge, upd)                 # kenar/nokta üzerinden
dirx = part.Directions.CreateDirection(o, (1,0,0), upd)   # veya datumAxis'ten
xf   = part.Xforms.CreateXformByPlaneXDirPoint(face, dirx, pt, upd, 0.625, False, False)
csys = part.CoordinateSystems.CreateCoordinateSystem(xf, upd)
# 2) (opsiyonel görünür datum) DatumCsysBuilder.Csys = csys; DisplayScaleFactor=1.25; CommitFeature()
# 3) In-place sketch
sb = part.Sketches.CreateSimpleSketchInPlaceBuilder()
sb.UseWorkPartOrigin = False
sb.CoordinateSystem  = csys
sb.HorizontalReference.Value = datumAxis        # yatay referans
# 7.0'daki Preferences.Sketch ayarlarını burada set et
sketch = ((NXOpen.Sketch)sb.Commit())
sketchFeat = sketch.Feature
sketch.Activate(NXOpen.Sketch.ViewReorient.True)
#  ... eğrileri çiz ...
ActiveSketch.CalculateStatus()
ActiveSketch.Deactivate(ViewReorient.True, UpdateLevel.Model)   # Finish Sketch
ses.EndTaskEnvironment()      # BeginTaskEnvironment() ile açılmıştı
```
> `Sketches.CreateSketchInPlaceBuilder2(null)` + `.PlaneOption=ExistingPlane/Inferred` alternatif;
> `SimpleSketchInPlaceBuilder` en sağlamı. `CreateSketchAlongPathBuilder` de var (path sketch).

### 7.3 Sketch eğri builder'ları
- **Dikdörtgen** [C]: `CreateRectangleBuilder`; `Width/Height.SetFormula`; `SnapBasePoint(p)`,
  `SnapDiagonalPoint(p)`; hizalama için `SetSnapTarget(infiniteLine|line)`, `SetDirectionAtTarget(0, vec)`.
- **Yay** [C]: `CreateArcBuilder`; `Radius.SetFormula`; `SetChainPoint(arc, p)` (varolan eğriye zincirle),
  `SnapFirst/Second/ThirdPoint`, `SetDirectionAtTarget(idx, vec)` (teğet), `ThroughPointIndex`, `SetThirdPoint`.
- **Çizgi** [C]: `CreateLineBuilder`; `Length/Angle/RelativeAngle.SetFormula`; `SetChainPoint`,
  `Snap Start/EndPoint`, `SetSnapTarget`, `SetSnapPointTarget(target, helpPt)`.
- **Tek-çağrı fillet** [C] — İKİ imza:
  ```python
  # 2 eğri:
  arcs = ActiveSketch.Fillet(c1, c2, help1, help2, ptOnArc, 30.0,
           Sketch.TrimInputOption.True, Sketch.CreateDimensionOption.False,
           Sketch.AlternateSolutionOption.False, out constraints)
  # 3 eğri (+DeleteThirdCurveOption):
  arcs = ActiveSketch.Fillet(c1, c2, infiniteLine, help1, help2, help3, ptOnArc, 29.0,
           Sketch.TrimInputOption.True, Sketch.DeleteThirdCurveOption.False,
           Sketch.CreateDimensionOption.False, Sketch.AlternateSolutionOption.False, out cons)
  ```
- **Sketch chamfer** [C]: `CreateSketchChamferBuilder`; `Distance1/Distance2/Angle.SetFormula`;
  `CurvesToChamfer.Add(line)` (ya da `.Add(pt, WorkView, pt)`); `HelpPoint = pt` (RemoveViewDependency).
- **QuickTrim** [C]: `CreateQuickTrimBuilder`; `TrimmedCurves.Add(curve, WorkView, pt)`;
  **`BoundaryObjects.Add(line)`** ile sınır ver (journal3); `ExtendBound=False`.

### 7.4 Sketch dimensional constraint = parametrik nokta yerleşimi — [C]
Bir sketch noktasını X/Y eksenine göre ölçülendirir (delik konumunun gerçek yolu):
```python
ActiveSketch.AddGeometry(point)
d1 = NXOpen.Sketch.DimensionGeometry(); d1.Geometry=point;      d1.AssocType=AssocType.StartPoint
d2 = NXOpen.Sketch.DimensionGeometry(); d2.Geometry=yAxisInfLn; d2.AssocType=AssocType.StartPoint
ActiveSketch.CreateDimension(NXOpen.Sketch.ConstraintType.PerpendicularDim, d1, d2, origin3d, None)
ActiveSketch.UpdateDimensionDisplay(); ActiveSketch.ShowDimensions(); ActiveSketch.Update()
```

### 7.5 Sketch mirror / offset — [C]
```python
# Mirror curve:
mb = part.Sketches.CreateSketchMirrorPatternBuilder(NXOpen.SketchPattern.Null)
mb.Section.SetAllowedEntityTypes(Section.AllowTypes.CurvesAndPoints)
#   Section'a CreateRuleCurveFeatureChain([sketchFeature], seedLine, ...) ile eğri ekle
mb.DirectionObject.Value = ActiveSketch.FindObject("YAxis")   # ayna ekseni (InfiniteLine)
mb.UpdateDirectionObject(); mb.UpdateInputSection(); mb.Commit(); mb.GetCommittedObjects()
# Offset curve:
ob = part.Sketches.CreateSketchOffsetBuilder(NXOpen.SketchOffset.Null)
ob.Tolerance = 0.01; sec = ob.CreateSection(); ob.Distance.SetFormula("5")
#   sec'e CreateRuleCurveFeatureChain ile eğri ekle
ob.UpdateLoopsAndCopies(); ob.EvaluateOffset(); # Commit
```
> Pattern: `CreateSketchPatternBuilder(null)` — `PatternService.PatternType=Linear/Circular/General`.

### 7.6 Section kurma — profili feature'a besleme (3 kural)
`part.Sections.CreateSection(0.0095, 0.01, 0.5)` → `sec.SetAllowedEntityTypes(AllowTypes.OnlyCurves)` →
`sec.AddToSection([rules], ...)`. Kural seçimi:
- **RegionBoundary** [C] — kapalı bölge, seed noktalı (extrude/revolve profili):
  `ScRuleFactory.CreateRuleRegionBoundary(sketch, ICurve[], seedPt, 0.01, opts)`.
  Tek section'a birden çok RegionBoundaryRule konabilir (journal2 extrude: 2 dikdörtgen).
- **CurveFeature** [C] — bir sketch feature'ının TÜM eğrileri:
  `CreateRuleCurveFeature([sketchFeature], null, opts)`.
- **CurveFeatureChain** [C] — zincir (mirror/offset section'ları): `CreateRuleCurveFeatureChain(...)`.
- **FeaturePoints** [C] — delik konumları: `CreateRuleFeaturePoints([sketchFeature], null)`.

### 7.7 RevolveBuilder — [C] (journal2/3'te gerçekten gövde üretti)
```python
rb = part.Features.CreateRevolveBuilder(NXOpen.Features.Feature.Null)
rb.Limits.StartExtend.Value.SetFormula("0"); rb.Limits.EndExtend.Value.SetFormula("360")
rb.Tolerance = 0.01
rb.Section = part.Sections.CreateSection(0.0095, 0.01, 0.5)      # sonra section'ı doldur (§7.6)
rb.SmartVolumeProfile.OpenProfileSmartVolumeOption = False
rb.SmartVolumeProfile.CloseProfileRule = SmartVolumeProfileBuilder.CloseProfileRuleType.Fci
rb.SetStartLimitHelperPoint([0,0,0]); rb.SetEndLimitHelperPoint([0,0,0])
axis = part.Axes.CreateAxis(NXOpen.Point.Null, direction, upd)  # sonra axis.Point / axis.Direction set edilir
rb.Axis = axis
rb.ParentFeatureInternal = False
feat = rb.CommitFeature()
```

### 7.8 ExtrudeBuilder — [C] (journal2: revolve gövdesinden Subtract)
```python
eb = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
eb.Section = part.Sections.CreateSection(0.0095, 0.01, 0.5)
eb.AllowSelfIntersectingSection(True)
eb.BooleanOperation.SetTargetBodies([body]); eb.BooleanOperation.Type = BooleanType.Subtract  # veya Create
eb.Limits.StartExtend.Value.SetFormula("-8"); eb.Limits.EndExtend.Value.SetFormula("2")
eb.Draft.FrontDraftAngle.SetFormula("2");  eb.Offset.EndOffset.SetFormula("5")   # opsiyonel
eb.SmartVolumeProfile.CloseProfileRule = SmartVolumeProfileBuilder.CloseProfileRuleType.Fci
eb.Direction = part.Directions.CreateDirection(sketch, NXOpen.Sense.Forward, upd)
feat = eb.CommitFeature()
```

### 7.9 HolePackageBuilder — ÜRETİM sınıfı — [C] (§2.7'nin tam hali)
Delik konumu **sketch noktalarından** (§7.2+§7.4 ile önce nokta yerleştir), hedef gövde SmartCollector:
```python
hp = part.Features.CreateHolePackageBuilder(NXOpen.Features.HolePackage.Null)
hp.Tolerance = 0.01
hp.TypeOfHole = HolePackageBuilder.TypesOfHole.Counterbored   # Simple/Counterbored/Countersink...
hp.SizeOfHole = HolePackageBuilder.SizesOfHole.DrillSize      # DrillSize/Custom/Screw...
hp.DrillSizeStandard = "ISO"; hp.DrillSize = "8.2"; hp.DrillSizeFitOption = "Exact"
# veya delikli/vidalı: hp.StartHoleData.ScrewType="General Screw Clearance"; .ScrewSize="M10"; .FitOption="Normal (H13)"
# tüm boyutlar SetFormula(str): GeneralSimpleHoleDiameter/Depth, Counterbore/CountersinkDiameter/Depth/Angle,
#   ThreadSize="M10 x 1.5", TapDrillDiameter, ThreadDepth, TipAngle, chamfer offset/angle...
# KONUM:
fp = part.ScRuleFactory.CreateRuleFeaturePoints([sketchFeature], NXOpen.DisplayableObject.Null)
hp.HolePosition.SetAllowedEntityTypes(Section.AllowTypes.OnlyPoints)
hp.HolePosition.AddToSection([fp], null,null,null, helpPt, Section.Mode.Create, False)
hp.HolePosition.EvaluateAndAskOutputEntities(out refs)
# HEDEF GÖVDE:
col = hp.BooleanOperation.GetTargetBodiesCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleBodyDumb([body], True, opts)], False)
hp.ParentFeatureInternal = True
feat = hp.Commit()          # try/except AssertErrorCode(674704) "no closest face / Normal to Face"
```

### 7.10 EdgeBlend — KONİK/değişken kesit + tangent-seed kuralı — [C] (§2.1'i genişletir)
```python
ebb = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
ebb.Tolerance = 0.01
ebb.RollOverSmoothEdge = True; ebb.RollOntoEdge = True; ebb.RemoveSelfIntersection = True
ebb.BlendFaceContinuity = EdgeBlendBuilder.FaceContinuity.Curvature
col = part.ScCollectors.CreateCollector()
rule = part.ScRuleFactory.CreateRuleEdgeMultipleSeedTangent([seedEdge], 0.5, True, opts)  # teğet zincir
col.ReplaceRules([rule], False); col.AddEvaluationFilter(NXOpen.ScEvaluationFiltertype.LaminarEdge)
# Konik kesit (basit string yerine): (col, Section, Conic, Rhotype, radius, halfRadius, rho)
ebb.AddChainset(col, EdgeBlendBuilder.Section.Conic, EdgeBlendBuilder.Conic.BoundaryPlusCenter,
                EdgeBlendBuilder.Rhotype.Absolute, "20", "13.333", "0.6")
feat = ebb.CommitFeature()
# introspeksiyon: ebb.GetChainsetAndSectionValue(0, out col2, out sect, out conic, out rho, out e1,e2,e3)
```

### 7.11 Draft (Face) — TwoExpressionsCollectorSet alternatifi — [S] (§2.3'e ek yol)
```python
db = part.Features.CreateDraftBuilder(...); db.AngleTolerance=0.5; db.DistanceTolerance=0.001
db.DraftIsoclineOrTruedraft = DraftBuilder.Method.Isocline
db.TypeOfDraft = DraftBuilder.Type.Face
db.Direction = part.Directions.CreateDirection(face, Sense.Forward, upd)
db.StationaryReference.ReplaceRules([CreateRuleFaceDumb([stationaryFace])], False)
tecs = part.CreateTwoExpressionsCollectorSet(NXOpen.ScCollector.Null, "10","10","Angle", 0)
db.TwoDimensionFaceSetsData.Append(tecs)
tecs.Collector = <FaceTangentRule ile dolu ScCollector>
```
> §2.3'teki `CreateExpressionCollectorSet`+`FaceSetAngleExpressionList` yolu da geçerli; bu ikinci
> (2-expr) yol journal3'te kurulup undo edildi — imza doğru, sonuç doğrulanmadı.

### 7.12 ApexRangeChamfer — modern pah feature'ı — [S]
```python
ab = part.Features.DetailFeatureCollection.CreateApexRangeChamferBuilder(NXOpen.Features.ApexRangeChamfer.Null)
cs = ab.EdgeManager.CreateChamferEdgeChainSetBuilder()
cs.Distance1.SetFormula("5"); cs.Distance2.SetFormula("5"); cs.Angular.SetFormula("45")
cs.CrossSectionOptions = ChamferEdgeChainSetBuilder.CrossSectionType.Asymmetric   # simetrik de var
ab.EdgeManager.EdgeChainSetList.Append(cs)
ab.Tolerance = 0.01
# LengthLimitsList.CreatePointFacePlaneSelectionBuilder(...) ile sınır düzlemi
```
> §2.2'deki `CreateChamferBuilder` (basit, kanıtlı-committed) hâlâ tercih; bu modern feature journal2'de undo edildi.

### 7.13 EditWithRollbackManager — SONRAKİ feature'lar varken eski feature'ı düzenle — [C]
```python
erm = part.Features.StartEditWithRollbackManager(sketchFeature, redefineMarkId)
ses.BeginTaskEnvironment()
sketch.Activate(...)   #  ... düzenle ...
ActiveSketch.Deactivate(...); ses.EndTaskEnvironment()
erm.UpdateFeature(True); erm.Stop()
```

### 7.14 Yeni tuzaklar (journal2/3'ten)
| Tuzak | Belirti → Çözüm |
|---|---|
| Delik "Normal to Face" yönü yüz bulamıyor | `AssertErrorCode(674704)` → yön opsiyonunu **Along Vector**'e çevir ya da nokta yüzeyde olsun |
| `SetCornerPoints` datum tanımsız | `AssertErrorCode(670309)` "Datum plane undefinable" → try/except sar, offset'i önce Evaluate et |
| Builder expression silme | Sil'i `MeasureManager.SetPartTransientModification()` ↔ `ClearPartTransientModification()` arasına al; hâlâ kullanımdaysa `AssertErrorCode(1050029)` yut |
| Sketch scriptte kayıyor | §7.0: `CreateInferredConstraints=False` + `ContinuousAutoDimensioning=False` |
| Region seçilmiyor | `CreateRuleRegionBoundary` seed noktası bölge İÇİNDE olmalı; kapalı profil şart |
| Eksen yönü/nokta | `Axes.CreateAxis(null, dir)` sonra `axis.Direction`/`axis.Point`'i ayrı set + eski objeyi `AddToDeleteList` |

---

## 8. C# JOURNAL TARAFI — Loft/Yüzey Modelleme Reçeteleri (2026-07-18, canlı NX 2506'da doğrulandı)

> **Kaynak:** 4 GUI journal kaydı (`Desktop\journal.cs..journal4.cs`, ~42k satır, **UTF-16LE** —
> grep'ten önce `iconv -f UTF-16LE -t UTF-8` şart!) + `E:\program\UGOPEN\NXOpen\*.hxx` başlık
> doğrulaması + canlı çalıştırma (`Desktop\wing_parametric_nx.cs` → parametrik kanat ✓,
> `Desktop\aircraft_parametric_nx.cs` → tam uçak). Python değil **C# journal** kalıplarıdır;
> .NET eşleme kuralları §8.5'te.

### 8.1 DatumPlaneBuilder — Distance yöntemiyle offset düzlem — [A, canlı]
```csharp
var b = workPart.Features.CreateDatumPlaneBuilder((Features.Feature)null);
Plane plane = b.GetPlane();
plane.SetUpdateOption(SmartObject.UpdateOption.WithinModeling);
plane.SetMethod(PlaneTypes.MethodType.Distance);
plane.SetGeometry(new NXObject[] { xzDatumPlane });   // "DATUM_CSYS(0) XZ plane"
plane.SetFlip(false); plane.SetReverseSide(false);
plane.Expression.RightHandSide = "200";               // offset mm (string!)
plane.SetAlternate(PlaneTypes.AlternateType.One);
plane.Evaluate();
b.ResizeDuringUpdate = true;                          // boyutu NX'e bırak
Features.Feature f = b.CommitFeature(); b.Destroy();
```
**TUZAK (canlı yaşandı):** `SetCornerPoints(c1..c4)` script'ten verilirse → `NXException:
Datum plane undefinable`. GUI journal'da görünse de scriptte **KULLANMA**; `ResizeDuringUpdate`
yeterli. (§7.14'teki kayıtla tutarlı — orada da aynı hata kodu.)

### 8.2 Bağımsız spline: StudioSplineBuilderEx — sketch GEREKMEZ — [A, canlı]
```csharp
var b = workPart.Features.CreateStudioSplineBuilderEx((NXObject)null);  // null = yeni
b.DrawingPlaneOption = Features.StudioSplineBuilderEx.DrawingPlaneOptions.General;
b.DrawingPlane   = workPart.Planes.CreatePlane(org, normal, SmartObject.UpdateOption.WithinModeling);
b.InputCurveOption = Features.StudioSplineBuilderEx.InputCurveOptions.Hide;
b.MatchKnotsType = Features.StudioSplineBuilderEx.MatchKnotsTypes.None;
b.IsAssociative = false;      // noktalardan bağımsız, sağlam
b.IsPeriodic    = false;      // true = dikişsiz kapalı (gövde kesiti); false = sivri TE (airfoil)
b.Degree        = 3;
foreach (Point3d p in pts) {
    Point pt = workPart.Points.CreatePoint(p);
    var gcd = b.ConstraintManager.CreateGeometricConstraintData();
    gcd.Point = pt; b.ConstraintManager.Append(gcd);
}
b.Commit();
Spline spline = b.Curve;      // sonuç eğrisi bu property'de!
b.Destroy();
// noktaları sil: workPart.Points.DeletePoint(pt) — IsAssociative=false ise güvenli
```
**TUZAK (canlı yaşandı):** `CreateSketchSplineBuilder` **aktif sketch ister**; sketch yokken
`Commit()` → `Incorrect object for this operation`. Serbest spline için HER ZAMAN
`CreateStudioSplineBuilderEx`. Kapalı airfoil döngüsünde ilk=son nokta çakışması (keskin TE)
sorun çıkarmaz; yumuşak kapalı kesitlerde `IsPeriodic=true` + son noktayı TEKRARLAMA.

### 8.3 Through Curves loft: ThroughCurvesBuilder1 — [A, canlı]
```csharp
var b = workPart.Features.FreeformSurfaceCollection.CreateThroughCurvesBuilder1((Features.Feature)null);
// toleranslar (journal değerleri): chain 0.0095 / dist 0.01 / açı 0.5
b.Alignment.AlignCurve.DistanceTolerance = 0.01;   // + Chaining/Angle; aynısı SectionTemplateString'e
b.Alignment.AlignType = GeometricUtilities.AlignmentMethodBuilder1.Type.Parameter; // kritik!
b.Construction   = Features.ThroughCurvesBuilder1.ConstructionMethod.Normal;
b.PatchType      = Features.ThroughCurvesBuilder1.PatchTypes.Multiple;
b.BodyPreference = Features.ThroughCurvesBuilder1.BodyPreferenceTypes.Solid; // kapalı kesit → kapaklı katı
b.PreserveShape = false; b.ClosedInV = false; b.NormalToEndSections = false;
foreach (kesit) {
    Section sec = workPart.Sections.CreateSection(0.0095, 0.01, 0.5);
    sec.SetAllowedEntityTypes(Section.AllowTypes.OnlyCurves);
    var rule = workPart.ScRuleFactory.CreateRuleCurveDumb(new Curve[] { spline });  // sketch'siz eğri için
    sec.AllowSelfIntersection(false); sec.AllowDegenerateCurves(false);
    sec.AddToSection(new SelectionIntentRule[]{rule}, spline, null, null, helpPoint, Section.Mode.Create, false);
    b.SectionsList.Append(sec);
}
Features.Feature loft = b.CommitFeature(); b.Destroy();
Body body = ((Features.BodyFeature)loft).GetBodies()[0];
```
**Temiz loft'un sırrı (kanatta kanıtlandı):** tüm kesit spline'larını AYNI nokta sırası ve
sayısıyla üret (TE→üst→LE→alt→TE) + `AlignType=Parameter` → TE↔TE, LE↔LE eşleşir, burulma
olmaz. GUI journal'lardaki `SetStartCurveOfClosedLoop`/`ReverseDirectionOfLoop` çağrıları
kullanıcının hizalama düzeltmeleridir — üretim aynı sıralıysa GEREKMEZ. Kesit sırası =
`SectionsList.Append` sırası (loft V yönü). Uç nokta-kesit için `CreateRuleCurveDumbFromPoints`
mevcut (denenmedi); pratik çözüm: küçük uç kesiti (r≈3mm) + uçlarda kosinüs kesit sıklaştırma.

### 8.4 Boolean Unite (collector'lı) + gövde kimliği — [A, canlı]
```csharp
var bb = workPart.Features.CreateBooleanBuilderUsingCollector((Features.BooleanFeature)null);
bb.Operation = Features.Feature.BooleanType.Unite;
bb.Target    = fuselageBody;
ScCollector col = workPart.ScCollectors.CreateCollector();
BodyDumbRule rule = workPart.ScRuleFactory.CreateRuleBodyDumb(new Body[]{ wing, htail, fin });
col.ReplaceRules(new SelectionIntentRule[]{ rule }, false);
bb.ToolBodyCollector = col;                      // NOT: SetTool deprecated (NX5'ten beri)
Features.Feature f = bb.CommitFeature(); bb.Destroy();
```
EdgeBlend fairing için kenar seçimi: `body.GetEdges()` → `edge.GetVertices(out v1, out v2)`
orta noktası geometrik testten geçenler → `CreateRuleEdgeDumb(edges)` → §0/1 kuralı
`AddChainset(collector, "18")`. Heuristik seçim kırılgandır → MUTLAKA try/catch + builder
Destroy ile sar (canlı kalıp: `aircraft_parametric_nx.cs::TryWingFairing`).

### 8.5 C++ başlık → C# (.NET) eşleme kuralları — [A]
| C++ başlıkta (UGOPEN\NXOpen\*.hxx) | .NET journal'da |
|---|---|
| `SetXxx(value)` + `Xxx()` getter çifti | TEK **property**: `b.Xxx = value` (örn. `SetDrawingPlaneOption` YOK → `DrawingPlaneOption =`) |
| `GetXxx()/SetXxx()` çok-parametreli | metot olarak kalır (`SetGeometry`, `GetVertices(out,out)`) |
| `NXException(msg)` kurucu | **YOK** — kendi hatan için `System.Exception` fırlat; NX hataları zaten `NXOpen.NXException` gelir |
| `std::vector<T*>` | `T[]` dizisi |
| `NXString` | `string` |

**Journal dosya kodlaması:** NX'in kaydettiği .cs journal'lar **UTF-16LE+CRLF**. `file`/`grep`
boş dönüyorsa önce `iconv` ile çevir (bu oturumda 4 dosyada da yaşandı).

### 8.6 Tuzak: `Datums.FindObject("DATUM_CSYS(0) XZ plane")` isme bagimli — [A, canli yasandi]
Bos/yeni ya da farkli sablonlu parcada bu isim YOK → `NXException: No object found with this name`.
Saglam recete (canli dogrulandi): (1) isimle dene, (2) `foreach (TaggedObject o in workPart.Datums)`
ile `as DatumPlane` yapip `dp.Normal` +Y olani ara, (3) hicbiri yoksa `CreateDatumCsysBuilder(null)` +
`Xforms.CreateXform(origin, xDir, yDir, WithinModeling, 1.0)` → `CoordinateSystems.CreateCoordinateSystem`
→ `db.Csys = csys; db.CommitFeature()` ile mutlak orijinde datum CSYS olustur, sonra (2)'yi tekrarla.
Kalip: `ucav_ttail_nx.cs::FindOrCreateXzDatumPlane`.

### 8.7 Loft'u NOKTAYA kapatma (pole) — sivri/puruzsuz burun — [A, canli]
Kapali kesitli solid loft'un ucu kucuk halka + duz kapakla biterse guduk gorunur ("tam sivri
olmamis"). Cozum: ilk (veya son) kesit olarak TEK NOKTA icereren section ekle:
```csharp
NXOpen.Point pole = workPart.Points.CreatePoint(new Point3d(x0,0,0));
pole.SetVisibility(SmartObject.VisibilityOption.Visible);   // dumb point gorunur olmali
Section psec = workPart.Sections.CreateSection(0.0095, 0.01, 0.5);
psec.SetAllowedEntityTypes(Section.AllowTypes.CurvesAndPoints);          // kritik
var prule = workPart.ScRuleFactory.CreateRuleCurveDumbFromPoints(new Point[]{ pole });
psec.AddToSection(new SelectionIntentRule[]{prule}, pole, null, null, pole.Coordinates,
                  Section.Mode.Create, false);
b.SectionsList.Append(psec);   // diger (egri) kesitlerden ONCE -> loft noktaya kapanir
```
Nokta loft'un parent'i olur — SILME, `Blank()` ile gizle. Kalip: `ucav_ttail_nx.cs::LoftSolidPole`.

### 8.8 Shell (icini bosaltma) — journal5.cs'ten (2026-07-19, GUI-kayit dogrulamali)
Kompozit kabuk/duvar kalinligi icin `Insert->Offset/Scale->Shell` recetesi:
```csharp
var b = workPart.Features.CreateShellBuilder((Features.Feature)null);
b.Tolerance = 0.01;
b.UseSurfaceApproximation = true;
b.TgtPierceOption = false;
b.SetDefaultThickness("5");                       // string! (expression formula)
ScCollector col = workPart.ScCollectors.CreateCollector();   // ACIK kalacak yuzler
var rule = workPart.ScRuleFactory.CreateRuleFaceTangent(seedFace, new Face[0], 0.5, opts);
col.ReplaceRules(new SelectionIntentRule[]{ rule }, false);
b.RemovedFacesCollector = col;                    // bu yuzler ACILIR (kapak yuzeyi)
b.Body = body;                                    // hedef kati
// (ops.) yuz-bazli kalinlik: CreateExpressionCollectorSet(null,"5","Length",0)
//        + set.ItemFlipFlag=true + b.FaceThicknesses.Append(set)
Features.Feature f = (Features.Feature)b.Commit();  // journal'da Commit() kullanildi
b.Destroy();
```
Script notu: `FindObject("FACE 140 {...}")` GUI-kayit kimligi olup script'te KIRILGANDIR —
yuzu geometrik sec: `body.GetFaces()` uzerinde donup yuz merkez/normal testine gore
`CreateRuleFaceDumb(Face[])` kullan (ScRuleFactory'de mevcut; FaceTangent yerine tekil yuz
icin de calisir). RemovedFacesCollector BOS kalirsa shell tamamen kapali ici bos govde uretir.
journal5 ayrica: extrude Boolean'lari (Create/Subtract/Intersect) §7 kaliplariyla ayni,
sketchMirrorPattern + Curve->Trim yogun kullanim — yeni API yok, §7'dekiler gecerli.

> **Kaynak (§8.9-8.18, 2026-07-20):** Desktop'taki 4 tam-uçak/UCAV üretim journal'ının
> karşılaştırmalı harmanlanması — `aircraft_parametric_nx.cs` (v2, UCAV aero güncellemesi),
> `ucav_delta_nx.cs` (kuyruksuz delta, üretim parça kiti), `ucav_pro_suite_nx.cs` (boyutlandırma
> döngülü pro suite), `ucav_ttail_nx.cs` (§8.1-8.7'nin kaynağı). Dördü de aynı çekirdek kalıpları
> (§8.1-8.6) paylaşıyor — aşağıda SADECE üçü arasında yeni/ortak-olmayan kalıplar var.

### 8.9 Loft'u İKİ UCA kapatma (çift-pole) — [A, canlı]
§8.7 tek uca (burun) kapanıyordu; `ucav_delta_nx.cs::BuildCenterBodyFrames` ve
`ucav_pro_suite_nx.cs::BuildFuselage` loft'un HEM burnunu HEM kuyruğunu noktaya kapatıyor
(motor gövdeye gömülü olmayan, tam sivri-uçlu gövdeler için — örn. planör/kuyruksuz delta):
```csharp
static Body LoftSolidPole(Point startPole, Point endPole,
    Spline[] sections, Point3d[] helps, string name) {
    var b = workPart.Features.FreeformSurfaceCollection.CreateThroughCurvesBuilder1(null);
    // ... §8.3'teki tolerans/Alignment/Construction ayarları ...
    if (startPole != null) AppendPoleSection(b, startPole);   // §8.7'deki pole-section kalıbı
    foreach (kesit) { /* §8.3'teki normal Section.Append */ }
    if (endPole != null) AppendPoleSection(b, endPole);       // AYNI kalıp, loft SONUNA
    Feature loft = b.CommitFeature(); b.Destroy();
    return ((Features.BodyFeature)loft).GetBodies()[0];
}
```
`AppendPoleSection` = §8.7'nin `CurveDumbRule.FromPoints` gövdesi, ayrı bir yardımcı metoda
çıkarılmış (tek pole veya çift pole için tekrar kullanılır). `ucav_pro_suite_nx.cs` bunu
motor konfigürasyonuna göre KOŞULLU kullanıyor: tractor'da burun motора gömülü olduğundan
`nosePole=null` + o istasyonda yarıçap spinner çapına (`floorR=spinR`) clamp'leniyor; pusher'da
aynısı kuyrukta; twin'de iki uç da sivri kalıyor. Yani AYNI gövde üretici fonksiyon, tek bir
`floorR` parametresiyle hem "sivri burun/kuyruk" hem "motor girişi için kesik" gövde üretebiliyor
— ayrı bir fonksiyon yazmaya gerek yok.

### 8.10 Parametrik Expression okuma/yazma (GUI'den düzenlenebilir üretici) — [A, canlı]
4 journal'ın da omurgası: NX Expressions'ı hem OKUR (varsa) hem YARATIR (yoksa varsayılanla),
boyutlandırma sonuçlarını da GERİ YAZAR — böylece journal Tools→Expressions'tan düzenlenip
tekrar çalıştırılabilir bir "canlı model" haline geliyor:
```csharp
static double P(string name, double def, string kind) {
    foreach (Expression ex in workPart.Expressions)
        if (string.Equals(ex.Name, name, StringComparison.OrdinalIgnoreCase)) return ex.Value;
    string rhs = def.ToString(CultureInfo.InvariantCulture);
    if (kind == "") workPart.Expressions.Create(name + "=" + rhs);                 // birimsiz (oran/NACA kodu)
    else workPart.Expressions.CreateWithUnits(name + "=" + rhs,
             FindUnit(kind == "deg" ? "Degrees" : "MilliMeter"));
    return def;                                    // ilk çalıştırmada varsayılanı da döndür
}
static void SetP(string name, double v) {           // sonuç geri-yazımı (§8.15 sizing loop sonrası)
    foreach (Expression ex in workPart.Expressions)
        if (string.Equals(ex.Name, name, StringComparison.OrdinalIgnoreCase))
        { ex.RightHandSide = v.ToString("F1", CultureInfo.InvariantCulture); return; }
}
```
Her proje kendi tekil önekini kullanıyor (`ucav_*`, `ucavd_*` delta için) — hem isim çakışmasını
önlüyor hem §8.11'deki temizlik filtresiyle eşleşiyor. **Expressions feature DEĞİLDİR** →
§8.11'in `CleanupPrevious`'ı onları SİLMEZ, yani parametreler çalıştırmalar arası KALICI kalır
(istenen davranış: değeri Expressions'ta değiştir → journal'ı tekrar çalıştır → model güncellenir).

### 8.11 Kendi-kendini-temizleyen yeniden-üretim (idempotent re-run) — [A, canlı]
Tüm 4 journal, çalışmaya başlamadan önce KENDİ önceki üretimini siler — aynı parçada tekrar
tekrar çalıştırılabilir (parametre değiştir → tekrar çalıştır → eski geometri gider, yenisi gelir):
```csharp
static void CleanupPrevious(Session.UndoMarkId mark) {
    var doomed = new List<TaggedObject>();
    foreach (Features.Feature f in workPart.Features)
        if (f.Name.StartsWith("UCAV", StringComparison.OrdinalIgnoreCase)) doomed.Add(f);
    foreach (Curve c in workPart.Curves)
        if (c.Name.StartsWith("UCAV_SEC", StringComparison.OrdinalIgnoreCase)) doomed.Add(c);
    foreach (Point p in workPart.Points)
        if (p.Name.StartsWith("UCAV_SEC", StringComparison.OrdinalIgnoreCase)) doomed.Add(p);
    if (doomed.Count > 0) {
        theSession.UpdateManager.AddObjectsToDeleteList(doomed.ToArray());
        theSession.UpdateManager.DoUpdate(mark);
    }
}
```
Ön koşul: HER `CommitFeature()`/spline/point'e `f.SetName("UCAV_...")` ile proje-öneki
verilmiş olmalı (aksi halde bu filtre onları bulamaz/silemez). §8.10'daki Expression'lar bu
listeye GİRMEZ → parametreler korunur, sadece geometri baştan üretilir.

### 8.12 Çok-eksenli (FS-frame) gövde loft'u — blended wing-body sıfır-boşluklu ek — [A, canlı]
§8.3'teki gövde loft'ları hep AÇIKLIK-yönünde (Y-normal) kesit alıyordu. `ucav_delta_nx.cs::
BuildCenterBodyFrames` bunun yerine X-normal FRAME (uçak "FS/bulkhead" istasyonu) kesitleri
üretiyor — ve kritik nokta: her frame'in yüksekliğini KANAT panelinin AYNI analitik yüzey
fonksiyonundan örnekliyor:
```csharp
static double SurfZ(double x, double y, bool upper) {
    double[] Xu, Zu, Xl, Zl;
    SectionWorld(y, out Xu, out Zu, out Xl, out Zl);   // kanat istasyonunun DÜNYA koordinatları
    return upper ? InterpClamped(Xu, Zu, x) : InterpClamped(Xl, Zl, x);  // x'te lineer interpolasyon
}
```
Gövde iskeleti ile kanat kökü böylece MATEMATİKSEL OLARAK AYNI yüzeyden geldiği için iki loft
kusursuz (sıfır boşluk/kesişim) birleşir — §8.4'teki Boolean Unite + EdgeBlend fairing'in
gizlemeye çalıştığı uyumsuzluk baştan oluşmaz. `FrameHalfWidth` ikili-arama (bisection) ile
frame'in LE/TE sınırlarını da buluyor (panel o istasyonda daralıyorsa). **Prodüksiyon güvenliği:**
frame inşası herhangi bir sebeple patlarsa eski (spanwise) yönteme otomatik düşer, kit ASLA
yarım kalmaz:
```csharp
static Body BuildCenterBody() {
    try { return BuildCenterBodyFrames(); }
    catch (Exception ex) { Say("Frame loft olmadi -> spanwise yedek."); return BuildCenterBodySpanwise(); }
}
```

### 8.13 Cant-zincirli kanat (blended winglet / V-kuyruk) — sayısal entegrasyonla anchor — [A, canlı]
Aciklik boyunca cant/dihedral açısı SÜREKLİ değişen bir yüzey (düz kanadın yumuşakça dikey
winglet'e kıvrılması, ya da sabit büyük cant'lı V-kuyruk) loft edilirken, her kesitin SADECE
düzlem normali döndürülmesi YETMEZ — kesitin (y,z) ANKOR konumu da birikimli açıyı takip
etmeli, yoksa istasyonlar düzgün bir eğriye oturmaz. `WingPhiDeg(s)` açı-profilini, `WingAnchor`
küçük adımlarla sayısal integre eder:
```csharp
static void WingAnchor(double s, out double yMag, out double z) {
    int steps = 240; double ds = s / steps, y = 0, zz = WING_Z;
    for (int k = 0; k < steps; k++) {
        double phi = Deg2Rad(WingPhiDeg((k + 0.5) * ds));
        y += ds * Math.Cos(phi); zz += ds * Math.Sin(phi);
    }
    yMag = y; z = zz;
}
```
Sonra her kesitin çizim-düzlemi normali de AYNI phi'den türetilir (`new Vector3d(0,
side*Math.Cos(phi), Math.Sin(phi))`) ve profil kalınlık yönü `PlaceCanted` içinde (y,z)'ye
katlanır. Sabit `phi(s) = cant` (V-kuyruk, `ucav_ttail_nx.cs`/`aircraft_parametric_nx.cs`)
bu fonksiyonun dejenere/sabit-açı halidir — aynı kalıp, tek satırlık `WingPhiDeg` farkıyla
hem blended winglet hem V-kuyruk üretiyor.

### 8.14 Çoklu-gövde kit için hacim/kütle ölçüm tuzağı — [A, canlı yaşandı]
**TUZAK:** §2'deki `MeasureManager.NewMassProperties(units, 0.99, collector)` TEK gövdeli
collector'da doğru çalışıyor (kanıtlı), ama collector'a BooleanBodyRule ile BİRDEN FAZLA
gövde (üretim kiti — kanat+gövde+elevon+...) eklenirse **0 hacim/0 alan döndürüyor** (canlı
yaşandı, `ucav_delta_nx.cs`/`ucav_pro_suite_nx.cs::Analyze` yorumu: "coklu-govde collector
sifir donduruyordu"). **Doğrulanmış çözüm:** her gövdeyi AYRI collector'la tek tek ölç, topla:
```csharp
double volSum = 0, areaSum = 0;
foreach (Body b in kitBodies) {
    try {
        ScCollector col = workPart.ScCollectors.CreateCollector();
        var rule = workPart.ScRuleFactory.CreateRuleBodyDumb(new Body[]{ b });
        col.ReplaceRules(new SelectionIntentRule[]{ rule }, false);
        MeasureBodies mb = workPart.MeasureManager.NewMassProperties(mu, 0.99, col);
        volSum += mb.Volume; areaSum += mb.Area;
    } catch (Exception) { Say("  " + name + " olculemedi."); }   // 1 kirik parca raporu durdurmasin
}
```

### 8.15 Boyutlandırma yakınsama döngüsü + motor veritabanından seçim — [A, canlı, mimari]
NX API değil ama üç journal'ın da (ve `motor_nx`/`drone_nx`'in — bkz. proje hafızası) ortak
üretici mimarisi, tekrar kullanılabilir bir kalıp olarak not düşülüyor: stall-CL kısıtından
kanat alanı → AR'den açıklık/veter → ıslak-alan sürüklemesinden güç ihtiyacı → küçük sabit
motor veritabanından (`piston`/`electric`, güç/kütle/tüketim/boyut) **toplam kütle** (motor+
enerji) en düşük olanı seç → kütle bütçesinden yeni MTOW tahmini → **gevşetilmiş güncelleme**
`W = 0.5*Wold + 0.5*Wnew` (salınımı önler) → <0.05 kg yakınsayana kadar (üst sınır 40 iterasyon)
tekrar. Yakınsayan sonuçlar §8.10'daki `SetP()` ile Expressions'a geri yazılır — GUI'de
izlenebilir. `SelectEngine` güç marjı (%10) + toplam-kütle (motor×1.18 kule payı + enerji)
minimize ederek seçiyor; piston için enerji=yakıt kütlesi (BSFC g/kWh), elektrik için
enerji=batarya kütlesi (Wh/kg alanı BsfcGkWh'i yeniden kullanıyor).

### 8.16 Üretim kiti alt-parçaları — hizalama pimi / lonjeron / kontrol yüzeyi menteşesi — [A, canlı]
`ucav_delta_nx.cs`'nin çok-parçalı üretim kitinden üç tekrar-kullanılabilir alt-kalıp:
- **Hizalama pimi:** panel ek hattı (`y=y1` sabit rib) üzerinde split çizgisinin HER iki
  yanına simetrik (±60 mm) giren küçük-çaplı (Ø12) dairesel loft — H7/g6 geçmeli fiziksel
  hizalama pimi; ayrı-parça üretiminde tekrarlanabilir montaj konumu garantisi.
- **Lonjeron borusu:** dairesel kesit (§CirclePtsY, Y-normal daire noktaları + periodic
  spline) düz eksende loft. Kanat lonjeronu profil kalınlığına "sığar mı" diye İTERATİF
  küçültülüyor: `while (yEnd > y1+60) { if (0.72*localHalfThickness(yEnd) >= r) break;
  yEnd -= 0.02*SEMI; }` — sığmazsa parça sessizce ATLANIR (log'lanır, journal düşmez).
- **Kontrol yüzeyi (elevon) menteşe boşluğu:** yerel firar kenarı NAİF `z=0` VARSAYILAMAZ
  (washout nedeniyle burulmalı TE dünya-Z'de sıfır değildir) — yerel TE noktası + yerel veter
  doğrultusu burulmadan hesaplanır, sonra `ELEV_GAP` kadar AYNI doğrultuda geriye kaydırılmış
  yeni bir çeyrek-veter etrafında küçük panel loft edilir → menteşe hattı burulmayla hizalı kalır.

### 8.17 Pervane palı loft'u (NACA + pitch-burulma) — [A, canlı]
`BuildProp`/`PropSection`/`PropPt` (3 dosyada birebir aynı): hub'dan uç yarıçapa `Lerp` edilen
veter + `beta(r) = atan(pitch / (2·π·r)) * thrustDir` burulmasıyla NACA kesitleri döndürülüp
loft ediliyor; pal sayısı kadar `az = 2π·blade/bladeCount` azimut ofsetiyle kopyalanıyor;
kesit düzlem-normali radyal yön (cA,sA), teğetsel/eksenel bileşenler `PropPt` içinde ayrıştırılıp
döndürülüyor. `thrustDir=±1` aynı fonksiyonu hem tractor hem pusher itki yönünde kullanılabilir
kılıyor (§8.9'daki motor-konfigürasyonu kalıbıyla birlikte).

### 8.18 CG / nötr nokta / statik marj — analitik kütle bütçesi (mimari, NX-harici) — [A]
Kuyruklu (`ucav_ttail_nx.cs`/`ucav_pro_suite_nx.cs`) konfigürasyon klasik kuyruk-hacim-katsayısı
yöntemini kullanıyor (`Vh = Sh·lh/(S·mac)` → `xNP = xMacQC + 0.92·(at/aw)·(1-dε/dα)·Vh·mac`);
kuyruksuz (`ucav_delta_nx.cs`) konfigürasyon basitleştirilmiş %25-MAC nötr nokta kullanıyor VE
hedef statik marja göre faydalı-yük istasyonunu (`xPay`) CG denkleminden GERİYE ÇÖZÜYOR — sınır
aşımında ("SINIR! ek burun balastı gerekir") uyarı basıyor. **Önemli:** her ikisi de NX
`MeasureManager`'ın gerçek centroid'ini DEĞİL, bileşen kütle bütçesinden (yapı/motor/yakıt/yük
tahmini x-konumları) türetilmiş analitik CG kullanıyor — NX centroid ayrıca "geo-centroid" olarak
raporlanıyor ama malzeme atanmadıkça gerçek kütle dağılımını temsil etmiyor (bkz. §2'deki
Steel-density doğrulama kalıbı, malzeme atandıktan SONRA NX centroid/kütle güvenilir olur).

---

## 9. Gövde-çifti gerçek kesişim hacmi (NX-PEN, Boolean Intersect ile doğrulama)

> Bu bölüm önceden yalnız `askeri/sentinel_ugv` ve `askeri/ucav_nx` fork'larında yaşıyordu
> (2026-07-12 canlı kanıt) ve hiç bu kanonik kopyaya taşınmamıştı — §9→§8 fork-ayrışmasının
> tam örneği. Şimdi buraya taşındı; diğer 3 kopya senkronize edilecek (bkz. dosya başı notu).

`probe_boolean_intersect.py` — iki bindirmeli blokta 8000.0/8000 mm3 birebir:
```python
bb = part.Features.CreateBooleanBuilderUsingCollector(NXOpen.Features.BooleanFeature.Null)
bb.Operation = NXOpen.Features.Feature.BooleanType.Intersect
bb.RetainTarget = True; bb.RetainTool = True          # orijinaller korunur
bb.Targets.Add(bodyA); bb.Tools.Add(bodyB)            # SelectObjectList.Add (ScCollector DEGIL)
before = {b.Tag for b in part.Bodies}; bb.Commit()
yeni = [b for b in part.Bodies if b.Tag not in before] # kesisim govdeleri
# hacim: MeasureManager.NewMassProperties(units[:take],0.99,[b]).Volume (kanitli kalip)
# temizlik: uf.Obj.DeleteObject(b.Tag); bb.Destroy()
```
- Kesişim YOKSA `Commit()` fırlatır — istisna = hacim 0 (PASS) olarak kullanılır.
- `uf.Modl.AskMassProps3d` bu sarmalayıcıda YOK (UF.Modl'da yalnız BooleanBody/IntersectType var).
- Kullanım alanı: iki parçanın (ör. §8.12'deki gövde-kanat eki, ya da bir assembly'nin ayrı
  bileşenleri — motor+gearbox gibi) GERÇEKTEN kesişip kesişmediğini/ne kadar kesiştiğini
  sayısal olarak doğrulamak — `nx_inspect`'in interference sayımına tamamlayıcı, hacim bazlı
  bir ikinci doğrulama katmanı.
