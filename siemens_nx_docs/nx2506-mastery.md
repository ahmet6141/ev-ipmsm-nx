# Siemens NX 2506 — Profesyonel Journal Geliştirme Referansı

## NX Kurulum Bilgisi
- **NX Sürüm:** 2506 (Continuous Release serisi)
- **Kurulum yolu:** `E:\program\NXBIN`
- **PYTHON:** NX içinde Python 3.10, sıfır üçüncü-parti paket
- **Python stubs:** `E:\program\UGOPEN\pythonStubs\NXOpen\` (IDE otomatik tamamlama için)
- **Journal çalıştırma:** `run_journal.exe nx_journal.py -args arg1 arg2`
- **UGV Python:** `C:\Users\ahmet\Desktop\web\motor\.venv\Scripts\python.exe`

## NXOpen Python API — Temel Çağrılar (NX 2506 doğrulanmış)

### Session & Part
```python
import NXOpen
_session = NXOpen.Session.GetSession()
# Yeni mm parça: Parts.NewBaseDisplay(path, NXOpen.BasePart.Units.Millimeters)
# NOT: Part.Units.Millimeters "Second parameter is invalid" verir!
```

### Expressions
```python
e = part.Expressions.CreateWithUnits("name=value", unit)  # unit = Millimeter/Degrees/Number
part.Expressions.EditWithUnits(e, unit, "new_rhs")  # RightHandSide kullan, .Value değil!
```

### Extrude (En kritik feature)
```python
ext = part.Features.CreateExtrudeBuilder(NXOpen.Features.Feature.Null)
ext.Section = section  # Section objesi
ext.Direction = part.Directions.CreateDirection(origin, vector, UpdateOption.WithinModeling)
ext.Limits.StartExtend.Value.RightHandSide = "0"
ext.Limits.EndExtend.Value.RightHandSide = "length"  # veya expression adı
ext.BooleanOperation.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Unite
ext.BooleanOperation.SetTargetBodies([target_body])
feat = ext.CommitFeature()
ext.Destroy()
```

### Revolve
```python
rev = part.Features.CreateRevolveBuilder(NXOpen.Features.Feature.Null)
rev.Section = section
rev.Axis = axis_obj  # part.Axes.CreateAxis(point, direction, UpdateOption)
rev.Limits.EndExtend.Value.RightHandSide = "360"
rev.BooleanOperation.Type = bool_type
feat = rev.CommitFeature(); rev.Destroy()
```

### Section (Eğri zinciri)
```python
section = part.Sections.CreateSection(0.0095, 0.001, 0.5)
section.AllowSelfIntersection(False)
rule = part.ScRuleFactory.CreateRuleCurveDumb(curves)  # CreateRuleBaseCurveDumb DEPRECATED!
section.AddToSection([rule], curves[0], null, null, help_pt, Section.Mode.Create, False)
```

### Curves
```python
line = part.Curves.CreateLine(p0, p1)  # NXOpen.Point3d(x,y,z) — float zorunlu!
arc = part.Curves.CreateArc(center, xDir, yDir, radius, startAngle, endAngle)  # radyan
```

### Boolean
```python
# Extrude/Revolve builder'ın BooleanOperation.SetTargetBodies([body]) ile yapılır
# Ayrı BooleanBuilder genelde gerekmez
```

### Update — HER ADIMDAN SONRA ZORUNLU
```python
_session.UpdateManager.DoUpdate(undoMark)
# Her feature öncesi: mark = _session.SetUndoMark(MarkVisibility.Visible, "label")
# Hata durumunda: _session.UndoToMark(mark, "label")
```

### Body Naming (Part Navigator'da görünür)
```python
body.SetName("Display_Name")  # Alphanumeric + underscore, max ~48 char
```

### STEP Export (AP242)
```python
sc = _session.DexManager.CreateStepCreator()
sc.ExportAs = NXOpen.StepCreator.ExportAsOption.Ap242
sc.ObjectTypes.Solids = True
sc.ExportSelectionBlock.SelectionScope = NXOpen.ObjectSelector.Scope.SelectedObjects
sc.ExportSelectionBlock.SelectionComp.Add(bodies)  # SADECE solid body'ler
sc.InputFile = part.FullPath; sc.OutputFile = out_path
sc.Commit(); sc.Destroy()
```

### Parasolid Export (OPT-IN, NON-FATAL)
```python
# NX 2506: DexManager.CreateParasolidExporter()
# Eski: DexManager.CreateParasolidCreator() veya UF.Ps.ExportData()
# ÖNEMLİ: Tüm part'ı değil SADECE body'leri seç — yoksa construction curve'ler
# "Modeler error: please report fault" verip NX session'ı zehirler!
```

### Circular Pattern (opsiyonel, sürüm kayması riskli)
```python
pfb = part.Features.CreatePatternFeatureBuilder(NXOpen.Features.Feature.Null)
pfb.PatternService.PatternType = PatternDefinition.PatternEnum.Circular
pfb.FeatureList.Add([feature])  # AddFeatureToPattern YOK!
circ = pfb.PatternService.CircularDefinition
circ.RotationAxis = axis_obj
circ.AngularSpacing.SpaceType = PatternSpacing.SpacingType.Offset  # CountAndPitch YOK!
circ.AngularSpacing.NCopies.RightHandSide = str(count)
circ.AngularSpacing.PitchDistance.RightHandSide = str(angle)
```

### Remove Parameters + Clean Curves
```python
# Build sonrası temiz statik .prt için:
rpb = part.Features.CreateRemoveParametersBuilder()
rpb.Objects.Add(solids); rpb.Commit(); rpb.Destroy()
# Sonra construction curve'leri sil
```

## v2.0 Finish Feature'lar — DOĞRULANMIŞ REÇETELER (NX 2506)

> Bu bölüm `nx2506-verified-api-playbook.md` §2'nin özeti; ilk turda (v3) 6/12 test başarısızdı,
> ancak v3b→v3e iterasyonlarında **hepsi** çalışan reçeteye kavuştu. Tam gerekçe/kanıt için
> playbook'a bakın — burada yalnız güncel, kopyala-kullan hali var.

### Edge Blend (fillet) ✅
```python
ebb  = part.Features.CreateEdgeBlendBuilder(NXOpen.Features.Feature.Null)
col  = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleEdgeDumb(edges)], False)
ebb.AddChainset(col, "4")                     # (ScCollector, "radius"), edge+index DEĞİL
feat = ebb.CommitFeature(); ebb.Destroy()
```

### Chamfer ✅
```python
cb  = part.Features.CreateChamferBuilder(NXOpen.Features.Feature.Null)
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleEdgeDumb(edges)], False)
cb.SmartCollector = col                        # AddChainsToCollector YOK
cb.Option      = NXOpen.Features.ChamferBuilder.ChamferOption.SymmetricOffsets
cb.FirstOffset = "2"                           # str; FirstOffsetExp salt-okunur
feat = cb.CommitFeature(); cb.Destroy()
```

### Draft (Face) ✅
```python
db = part.Features.CreateDraftBuilder(NXOpen.Features.Feature.Null)
db.AngleTolerance = 0.5; db.DistanceTolerance = 0.001     # ŞART, yoksa "tolerance too small"
db.TypeOfDraft = NXOpen.Features.DraftBuilder.Type.Face
db.DraftReferencesMethod = NXOpen.Features.DraftBuilder.DraftReferencesMethods.StationaryFace
db.Direction = part.Directions.CreateDirection(o, z, NXOpen.SmartObject.UpdateOption.WithinModeling)
db.StationaryReference.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb([sabit_yüz])], False)
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb(eğilecek_yüzler)], False)
ecs = part.CreateExpressionCollectorSet(col, "3", "", 0)   # birim argümanı BOŞ STRING
db.FaceSetAngleExpressionList.Append(ecs)
feat = db.CommitFeature(); db.Destroy()
```

### Symbolic Thread ✅
```python
tb = part.Features.CreateThreadBuilder(NXOpen.Features.Thread.Null)     # Thread.Null!
tb.ThreadType  = NXOpen.Features.ThreadBuilder.Type.Symbolic
tb.ThreadInput = NXOpen.Features.ThreadBuilder.Input.Manual             # tablo yolu headless'ta kırılgan
tb.ShaftPreference = NXOpen.Features.ThreadBuilder.ShaftSizePreference.MajorDiameter
tb.CylindricalFace.Value = silindirik_yüz
tb.StartObject.Value     = başlangıç_düzlem_yüzü
tb.MajorDiameterExp.RightHandSide = "20"; tb.MinorDiameterExp.RightHandSide = "17.5"
tb.ShaftDiameterExp.RightHandSide = "18.75"; tb.PitchExp.RightHandSide = "2.5"
tb.AngleExp.RightHandSide = "60"; tb.ThreadLength.RightHandSide = "20"
feat = tb.CommitFeature(); tb.Destroy()
# thread → chamfer sırasıyla uygula (chamfer önce açılırsa başlangıç yüzünü tüketir)
```

### Shell ✅
```python
sb = part.Features.CreateShellBuilder(NXOpen.Features.Feature.Null)
sb.Tolerance = 0.01                            # ŞART, yoksa "Tolerance error"
sb.Body = body; sb.SetDefaultThickness("2")
col = part.ScCollectors.CreateCollector()
col.ReplaceRules([part.ScRuleFactory.CreateRuleFaceDumb([kaldırılacak_yüz])], False)
sb.RemovedFacesCollector = col
feat = sb.CommitFeature(); sb.Destroy()
```

### Mirror Body ✅ (`CreateMirrorBodyBuilder` **VAR** — bazı kaynaklar yok der, yanlış)
```python
m = NXOpen.Matrix3x3()
m.Xx,m.Xy,m.Xz = 0.0,1.0,0.0; m.Yx,m.Yy,m.Yz = 0.0,0.0,1.0; m.Zx,m.Zy,m.Zz = 1.0,0.0,0.0
dp = part.Datums.CreateFixedDatumPlane(NXOpen.Point3d(-30.0,0.0,0.0), m)   # SABİT datum şart
mb = part.Features.CreateMirrorBodyBuilder(NXOpen.Features.Feature.Null)
mb.MirrorBodyList.Add(body); mb.Plane.Value = dp    # Planes.CreatePlane KABUL EDİLMEZ
feat = mb.CommitFeature(); mb.Destroy()
```

### Malzeme atama ✅
```python
pm  = part.MaterialManager.PhysicalMaterials
mat = pm.LoadFromNxmatmllibrary("Steel")       # LoadMaterialsFromLibrary YOK
mat.AssignObjects([body])                      # AssignMaterialToBody YOK
```

### PMI Not ✅
```python
nb = part.Annotations.CreatePmiNoteBuilder(NXOpen.Annotations.SimpleDraftingAid.Null)  # PmiNotes.CreatePmiNote YOK
nb.Text.TextBlock.SetText(["SATIR 1", "SATIR 2"])          # Text'te değil TextBlock'ta
nb.Origin.Origin.SetValue(NXOpen.TaggedObject.Null, part.Views.WorkView, NXOpen.Point3d(0.0,0.0,80.0))
note = nb.Commit(); nb.Destroy()
```

### Hole Package ✅
```python
hp = part.Features.CreateHolePackageBuilder(NXOpen.Features.HolePackage.Null)
hp.HoleType = NXOpen.Features.HolePackageBuilder.Holetype.Simple
hp.GeneralSimpleHoleDiameter.SetFormula("8")
hp.HoleDepthLimitOption = NXOpen.Features.HolePackageBuilder.HoleDepthLimitOptions.Value
hp.GeneralSimpleHoleDepth.SetFormula("40"); hp.GeneralTipAngle.SetFormula("118")
hp.Tolerance = 0.01                            # ŞART; yoksa "Tolerance Specification requires three numbers"
# ... HolePosition Section'ı doldur (bkz. playbook §2.7) ...
hp.BooleanOperation.SetTargetBodies([body])    # UNUTMA: yoksa "Missing target body"
feat = hp.CommitFeature(); hp.Destroy()
```

### Headless PNG — KESİNLEŞTİ: MÜMKÜN DEĞİL ❌
`CreateImageExportBuilder()` ve `uf.Disp.CreateImage(...)` ikisi de headless'ta hata verir
(run_journal.exe'de grafik penceresi yok). Görüntü almak GUI ister.

## KRİTİK TUZAKLAR (NX 2506 doğrulandı)

1. **BasePart.Units.Millimeters** (Part.Units değil!)
2. **CreateRuleCurveDumb** (CreateRuleBaseCurveDumb deprecated)
3. **PatternSpacing.SpacingType.Offset** (CountAndPitch yok)
4. **FeatureList.Add([feat])** (AddFeatureToPattern yok)
5. **Expression RightHandSide** kullan (.Value 25.4× hatası verir)
6. **Point3d/Vectors3d float zorunlu** (int geçersen "Expecting double" hatası)
7. **DoUpdate HER ADIMDAN SONRA** (atlanırsa model güncellenmez)
8. **CreateParasolidExporter** body-seçimli (EntirePart curve'leri de alır → fault)
9. **NewBaseDisplay var olan .prt'ye yazmayı reddeder** → önce sil
10. **Birleştirme sırası:** plaka→pim→plaka zinciri gibi yüz temas sırasıyla unite
11. **Delik yüzeye teğet olamaz** → ≥2 mm et bırak
12. **Prism-subtract kanalları hedefi gerçekten kesmeli**
13. **hole kind konumu cx/cy/z0'dan okur** (origin3 değil)
14. **Tube unite→hedefe kaynar, bore hedeften kesilir**
15. **NX session zehirlenmesi:** "please report fault" → NX'i tamamen kapatıp yeniden başlat
16. **DraftBuilder.AngleTolerance** default'u geçersiz → 0.5 ata, yoksa "Angle tolerance is too small"
17. **ShellBuilder.Tolerance** default'u geçersiz → 0.01 ata, yoksa "Tolerance error"
18. **HolePackage ThroughBody yolu** → "Tolerance Specification requires three numbers"; Value+derinlik+uç açısı+Tolerance kullan
19. **HolePackage hedef gövde** → "Missing target body"; `BooleanOperation.SetTargetBodies` çağır
20. **CreateExpressionCollectorSet birim argümanı** → boş string `""` ver, "Degrees"/"deg" invalid unit hatası verir
21. **Thread tablo yolu (headless)** → "Standard data not found"; `Input.Manual` + Exp'ler kullan
22. **Thread StartObject** → komşu düzlemsel yüz; diş adımını **pahtan ÖNCE** uygula; adayları sırayla dene
23. **Başarısız Commit builder'ı kirletir** → aynı builder'la ikinci deneme yapma, yeniden yarat
24. **Fonksiyon içi `import NXOpen.X`** → `NXOpen` tüm fonksiyonda yerel olur, "cannot access local variable" hatası; alt-modül import'larını modül başına koy
25. **Headless görüntü** → ImageExportBuilder + UF.Disp.CreateImage ikisi de çalışmaz, PNG yalnız GUI'de mümkün

## BuildStep Schema (sx_engine)

Tüm CAD geometrisi bu şema ile tanımlanır:
- **prism:** kapalı (u,v) profil, keyfî eksende extrude
- **cylinder:** katı silindir, keyfî eksende
- **tube:** içi boş silindir (outer/inner radius)
- **hole:** keyfî eksende silindirik kesim
- **extrude:** kapalı XY profil, +Z ekseninde
- **revolve:** kapalı (d,a) profil, Z etrafında döndür
- **loft_twist:** iki profil arası loft (create-only)
- **fillet / chamfer / thread / draft / mirror / material / pmi_note / hole_package:**
  v2.0 finish feature'lar — hepsi ✅ doğrulandı (yukarıdaki reçeteler + playbook §2)

Her step: id, kind, boolean (create/unite/subtract), target, body_name, material, color
+ kind'a özel parametreler (profile, axis, origin3, length, radius, pattern_count...)

## NX Öğrenme Kaynakları
- nxjournaling.com — NX Journal topluluğu (tutorial + örnek journal'lar)
- github.com/cfs-energy/nxlib — Profesyonel NXOpen kütüphanesi (Python)
- github.com/nikitamamay/nx12-nxopen-pyi — NXOpen Python type stubs
- E:\program\UGOPEN\pythonStubs\ — Yerel NX 2506 stubs (IDE için)
- Siemens docs: docs.sw.siemens.com (NXOpen Python API Reference)
