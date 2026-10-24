<div align="center">

<img src="docs/kapak.jpg" alt="Ters Kinematik ile Al-Bırak — Praxtify" width="100%">

# Ters Kinematik ile Al-Bırak · MuJoCo + SO101

**Bir robot kol, küpün yerini nereden bilir?** Bu repo, SO101 robot kolunun bir küpü A noktasından alıp B noktasına bıraktığı
simülasyonu sıfırdan kurar: ters kinematiği kosinüs teoremiyle elle çözer, hareketi yumuşatır ve her şeyi
**MuJoCo** fizik motorunda gerçek temas, sürtünme ve yerçekimiyle çalıştırır.

[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.x-1E88E5)](https://mujoco.org/)
[![Lisans: MIT](https://img.shields.io/badge/Lisans-MIT-green.svg)](LICENSE)
[![YouTube](https://img.shields.io/badge/YouTube-@praxtify-FF0000?logo=youtube&logoColor=white)](https://www.youtube.com/@praxtify)

</div>

<p align="center">
  <img src="docs/simulasyon.gif" alt="SO101 küpü A noktasından alıp B noktasına bırakıyor" width="720">
</p>

---

## İçindekiler

- [Ne öğreneceksin?](#ne-öğreneceksin)
- [Hızlı başlangıç](#hızlı-başlangıç)
- [Proje yapısı](#proje-yapısı)
- [Nasıl çalışıyor?](#nasıl-çalışıyor)
- [Sonuç](#sonuç)
- [Kendin dene](#kendin-dene)
- [Kaynaklar ve lisanslar](#kaynaklar-ve-lisanslar)
- [Praxtify](#praxtify)

## Ne öğreneceksin?

| Konu | Bu projede |
|---|---|
| Fizik motoru | MuJoCo ile sahne kurma, `MjSpec` ile sahneyi koddan düzenleme, `MjModel` / `MjData` farkı |
| Ters kinematik | Parmak ucunun hedef konumundan dört motor açısını kosinüs teoremiyle hesaplama |
| Hareket planlama | Dokuz adımlık al-bırak görevi, *smoothstep* ile sarsıntısız geçiş |
| Simülasyon döngüsü | Her adımda IK → motor komutu → `mj_step` (5 ms) → ekran |

Kodun tamamı **tek dosya, 60 satırdan kısa**: [`pick_and_place.py`](pick_and_place.py). Videoda satır satır yazılan kodla birebir aynıdır.

## Hızlı başlangıç

```bash
git clone https://github.com/praxtify/0-mujoco-ters-kinematik.git
cd 0-mujoco-ters-kinematik

python -m venv .venv
# Windows:      .venv\Scripts\activate
# macOS/Linux:  source .venv/bin/activate
pip install -r requirements.txt

python pick_and_place.py
```

MuJoCo penceresi açılır ve kol görevi ~11 saniyede tamamlar.

> [!NOTE]
> - Kodu **repo klasöründen** çalıştır: robot modeli `models/…` göreli yoluyla açılır.
> - **macOS'ta** `mujoco.viewer.launch_passive` için `python` yerine `mjpython pick_and_place.py` kullan (MuJoCo ile birlikte gelir).
> - Test edilen ortam: Python 3.13 · MuJoCo 3.15 · NumPy 2.5 (Windows 11).

## Proje yapısı

```text
0-mujoco-ters-kinematik/
├── pick_and_place.py                       # bütün proje: sahne + ters kinematik + görev + simülasyon
├── requirements.txt                        # mujoco, numpy
├── models/
│   └── mujoco_menagerie/robotstudio_so101/ # SO101 robot modeli (MuJoCo Menagerie, Apache 2.0)
├── docs/                                   # README görselleri
└── LICENSE                                 # MIT (proje kodu)
```

## Nasıl çalışıyor?

### 1 · Sahne: robot + küp + işaretler

SO101 modeli `MjSpec` ile **düzenlenebilir** açılır; XML dosyasına dokunmadan sahneye kodla eklemeler yapılır:

- **Küp:** 3 cm, 30 g, serbest eklem (düşer, itilir, tutulur) ve yüksek sürtünme (`friction = 1.5`) — tutucu kaydırmasın.
- **A / B işaretleri:** yalnız görsel diskler (`contype = conaffinity = 0`), fiziğe karışmaz.

`spec.compile()` sahneyi **model**e çevirir: robotun değişmeyen tarifi. `MjData` ise her an değişen **durum**dur: açılar, hızlar, kuvvetler.

### 2 · Ters kinematik: konumdan eklem açılarına

`ik(x, y, z)` parmak ucunun gideceği noktayı alır, dört motorun açısını döndürür.

**Taban** hedefin yönüne döner:

$$\text{yaw} = -\,\mathrm{atan2}\big(y,\; x - x_{\text{taban}}\big)$$

Problem düzleme iner: omuzdan bileğe yatay uzaklık $r$, dikey uzaklık $h$ (tutucu boyu $\text{TOOL}$ eklenir, çünkü bilek parmak ucunun üstündedir):

$$r = \sqrt{(x - x_{\text{taban}})^2 + y^2} - s_x \qquad h = z + \text{TOOL} - s_z$$

Üst kol $L_1 = 11{,}6$ cm, ön kol $L_2 = 13{,}5$ cm ve hedef bir üçgen kurar. **Kosinüs teoremi** dirsek açısını verir;
eksi işaret dirseği yukarıda tutar:

$$\cos\varphi = \frac{r^2 + h^2 - L_1^2 - L_2^2}{2\,L_1 L_2} \qquad \varphi = -\arccos(\cos\varphi)$$

<p align="center"><img src="docs/kosinus-teoremi.jpg" alt="Kosinüs teoremi satırı ve açıklaması" width="720"></p>

**Omuz** açısı, hedefin yönünden dirseğin bükülmesiyle oluşan sapmanın çıkarılmasıdır:

$$a_1 = \mathrm{atan2}(h, r) - \mathrm{atan2}\big(L_2 \sin\varphi,\; L_1 + L_2 \cos\varphi\big)$$

Gerçek SO101'in kolları dümdüz değildir; `B1, B2, B3` bu küçük bükülmeleri motor açısına çevirirken hesaba katar.
**Bilek** kalan açıyı tamamlar, böylece tutucu her zaman dik aşağı bakar (toplam açı −90°).

### 3 · Yumuşak hareket

Her geçiş `smoothstep` eğrisiyle yapılır — yavaş başlar, hızlanır, yumuşakça durur:

$$s(t) = 3t^2 - 2t^3, \qquad \mathbf{p}(t) = \mathbf{p}_0 + (\mathbf{p}_1 - \mathbf{p}_0)\, s(t)$$

### 4 · Görev planı: dokuz adım

<p align="center"><img src="docs/gorev-plani.jpg" alt="Görev planı: dokuz adım" width="720"></p>

| # | Adım | Konum | Tutucu |
|:-:|---|---|:-:|
| 1 | A'nın üstüne gel | A, z = 7 cm | açık |
| 2 | Aşağı in | A, z = 1,2 cm | açık |
| 3 | Kavra | A, z = 1,2 cm | **kapalı** |
| 4 | Kaldır | A, z = 7 cm | kapalı |
| 5 | B'ye taşı | B, z = 7 cm | kapalı |
| 6 | Aşağı in | B, z = 1,4 cm | kapalı |
| 7 | Bırak | B, z = 1,4 cm | **açık** |
| 8 | Yukarı çık | B, z = 7 cm | açık |
| 9 | Başlangıca dön | (20, 0, 7) cm | açık |

### 5 · Simülasyon döngüsü

Zamanı biz yönetiriz: her plan adımı **240 fizik adımı** (240 × 5 ms = 1,2 s) sürer. Her adımda hedef konum
`smooth` ile ilerler, ters kinematik çözülür, motorlara hedef açı (`data.ctrl`) verilir ve `mj_step` dünyayı 5 ms ileri alır:
temas, sürtünme, yerçekimi. `viewer.sync()` ekranı günceller, `time.sleep` gerçek zamanla eşitler.

## Sonuç

Görev **10,8 saniyelik** simülasyonda tamamlanır; küp B noktasının **4,8 mm** yakınına bırakılır
(ölçüm: `data.body("cube").xpos` → `[0.195, 0.100, 0.015]`). Tutma, kaldırma ve bırakma tamamen fizik motorunun temas
hesabıyla olur — küp hiçbir yerde "yapıştırılmaz".

## Kendin dene

- `A` ve `B`'yi değiştir: kolun erişebildiği alanı bul (kosinüs `[-1, 1]` dışına çıkınca ne olur?).
- `friction` değerini 0,3'e indir: küp tutucudan kayıyor mu?
- `range(240)`'ı 120 yap: hareket iki kat hızlanır — küp hâlâ taşınabiliyor mu?
- Plana yeni bir nokta ekle: küpü üçüncü bir yere, ya da üst üste iki küp diz.

## Kaynaklar ve lisanslar

Bu repodaki **proje kodu** ([`pick_and_place.py`](pick_and_place.py)) [MIT lisansı](LICENSE) ile yayımlanır. Üçüncü taraf parçalar
kendi lisanslarıyla gelir:

| Bileşen | Kaynak | Lisans | Repoda |
|---|---|---|---|
| **SO101 robot modeli** (MJCF + STL) | [MuJoCo Menagerie · `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101) — The Robot Studio'nun [SO-ARM100 / SO101](https://github.com/TheRobotStudio/SO-ARM100) tasarımından | [Apache 2.0](models/mujoco_menagerie/robotstudio_so101/LICENSE) | ✔ `models/` (değiştirilmeden) |
| **MuJoCo** fizik motoru | [Google DeepMind](https://github.com/google-deepmind/mujoco) | Apache 2.0 | `pip` ile kurulur |
| **NumPy** | [numpy.org](https://numpy.org/) | BSD 3-Clause | `pip` ile kurulur |

Model dosyaları MuJoCo Menagerie'den **değiştirilmeden** kopyalanmıştır; ayrıntılar ve değişiklik geçmişi için
modelin kendi [README](models/mujoco_menagerie/robotstudio_so101/README.md) ve [CHANGELOG](models/mujoco_menagerie/robotstudio_so101/CHANGELOG.md) dosyalarına bakın.

## Praxtify

**Teoriyi pratiğe dönüştürüyoruz.** Robotik, gömülü sistemler ve yapay zekâyı çalışan kodla, adım adım anlatıyoruz.
Bu projenin videosu ve yeni projeler için:

<p align="center">
  <a href="https://praxtify.com"><img src="https://img.shields.io/badge/praxtify.com-Web-111111?style=for-the-badge" alt="praxtify.com"></a>
  <a href="https://www.youtube.com/@praxtify"><img src="https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://www.instagram.com/praxtify"><img src="https://img.shields.io/badge/Instagram-E4405F?style=for-the-badge&logo=instagram&logoColor=white" alt="Instagram"></a>
  <a href="https://www.tiktok.com/@praxtify"><img src="https://img.shields.io/badge/TikTok-000000?style=for-the-badge&logo=tiktok&logoColor=white" alt="TikTok"></a>
  <a href="https://x.com/praxtify"><img src="https://img.shields.io/badge/X-000000?style=for-the-badge&logo=x&logoColor=white" alt="X"></a>
  <a href="https://www.threads.net/@praxtify"><img src="https://img.shields.io/badge/Threads-000000?style=for-the-badge&logo=threads&logoColor=white" alt="Threads"></a>
  <a href="https://www.linkedin.com/company/praxtify"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.facebook.com/praxtify"><img src="https://img.shields.io/badge/Facebook-1877F2?style=for-the-badge&logo=facebook&logoColor=white" alt="Facebook"></a>
  <a href="https://www.pinterest.com/praxtify"><img src="https://img.shields.io/badge/Pinterest-BD081C?style=for-the-badge&logo=pinterest&logoColor=white" alt="Pinterest"></a>
</p>

Soru, hata ya da öneri için bir [issue](../../issues) aç. Projeyi beğendiysen ⭐ vermeyi unutma!

**Serinin sıradaki projesi:** [1-arduino-robot-kol](https://github.com/praxtify/1-arduino-robot-kol) — bu kolu gerçek bir Arduino ile yönetiyoruz.
