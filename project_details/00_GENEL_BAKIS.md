# 00 — Genel Bakış

## Proje nedir?

**motor_nx**, bir elektrikli araç (EV) traksiyon motorunu — **iç mıknatıslı senkron
motor (IPMSM)** — tek bir parametre setinden tam otomatik olarak Siemens NX'te katı
modele çeviren parametrik/generative bir CAD üretecidir. Referans tasarım Tesla
Model 3 arka tahrik ünitesi (RDU) sınıfındadır:

| Büyüklük | Değer |
|---|---|
| Topoloji | Radyal akı IPMSM, tek-V iç mıknatıs, hairpin dağıtılmış sargı |
| Oluk / kutup | 54 oluk / 6 kutup (q = 3, tam adım) |
| Stator OD / bore | 225 / 161 mm (split ratio ~0.715) |
| Aktif paket boyu | 134 mm |
| Hava aralığı | 0.70 mm (radyal) |
| Rotor OD | ~159.6 mm |
| Mıknatıs | Sinter NdFeB N42SH, kutup başına tek V (2 blok), 4 eksenel segment |
| Sargı | Hairpin, oluk başına 8 bar |
| Soğutma | Su ceketi (eksenel kanallar) |
| Şaft | İçi boş 45 mm (yağ besleme), iki rulman yatağı |

Birinci-mertebe analitik tahmin: ~440 Nm tepe tork, ~18 krpm azami hız sınıfı.
Kesin değerler için FEA (Motor-CAD / FEMM / Maxwell) gerekir — proje bunun için
hazır bir hand-off paketi üretir.

## Hedef

Bir parametre değişikliğinden (ör. paket boyu, mıknatıs genişliği, kutup sayısı)
**üretilebilir** bir motora kadar olan zinciri tek komutla koşturmak:

```
parametreler → boyutlandırma + doğrulama → blueprint → NX katı model + STEP
                                                     ↘ BOM / tolerans / donanım
                                                     ↘ 2D imalat resimleri
                                                     ↘ FEA hand-off paketi
```

## Kapsam: EM-aktif katı **+ montaj/üretim özellikleri**

Model iki katmandan oluşur:

1. **Elektromanyetik-aktif katı** (her zaman): stator/rotor lamine paketleri,
   oluk kesimleri, V-mıknatıs cepleri + mıknatıslar, hairpin barlar + uç-sargı,
   şaft, su ceketi + soğutma kanalları. Bunlar tork/akıyı üreten gövdelerdir.

2. **Montaj / üretim özellikleri** (varsayılan AÇIK, `assembly.enabled`): gerçek,
   *monte edilebilir* bir parçanın ihtiyaç duyduğu üretim detayları — bağlama/
   tie-rod delikleri, dönmez kamalar, rotor balans/perçin delikleri, şaft kama
   yuvası + segman kanalı + yağ delikleri, gövde montaj flanşı + civata daireleri,
   soğutucu portları, terminal geçişi, kaldırma halkası. Tümü parametrik, tek tek
   kapatılabilir ve **build'den önce geometrik olarak doğrulanır**.

> Tasarım kararı: her özellik bağımsızdır. `assembly.enabled = False` (veya bir
> özelliğin sayısını/boyutunu 0 yapmak) yalnız o özelliği kaldırır; saf EM katı
> elde edilir. Bu sayede EM-FEA için temiz bir gövde, imalat için tam donanımlı
> bir gövde aynı parametre setinden çıkar.

Detaylar: [03_MONTAJ_OZELLIKLERI.md](03_MONTAJ_OZELLIKLERI.md).

## Kim için?

- **İnsan mühendis:** parametreleri ayarlar, `cli report/validate` ile kontrol
  eder, NX'te build alır, BOM/tolerans/resim/donanım listesini imalata verir.
- **Yapay zekâ ajanı:** bu klasörü + kod docstring'lerini okuyarak değişiklik
  yapar; her değişikliği `tests/` ve `cli validate` ile doğrular.

## Tasarım felsefesi (neden bu mimari?)

`params → saf-matematik blueprint → CAD builder` ayrımı kanıtlanmış bir desendir
(kullanıcının diğer generative-CAD projelerinden taşındı). Faydası: geometri
matematiği **NX'siz** test edilebilir (saf CPython), yalnız son adım NX'e bağımlıdır.
Böylece 78 birim testi bir motorun build edilip edilemeyeceğini NX açılmadan yakalar.
