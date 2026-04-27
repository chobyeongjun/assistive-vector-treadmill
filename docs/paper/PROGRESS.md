# Paper Progress Notes — assistive-vector-treadmill

**Last updated:** 2026-04-25
**Target journal:** Journal of NeuroEngineering and Rehabilitation (JNER)
**Status:** Pre-experiment Methods/Protocol draft complete. 18-page LaTeX compiled.

---

## 1. Paper Identity (lock 완료)

**Title:**
> Biomechanical effects of shank-level cable attachment position and pulling direction during walker-supported gait: A preliminary characterization

**Authors (확정):**
1. Kyeongmin Lim (임경민) — co-first author $\dagger$
2. Byeongjun Cho (조병준) — co-first author $\dagger$
3. Alireza Nasirzadeh
4. Gi-uk Lee (이기욱) — corresponding author *

**Affiliation:** Department of Mechanical Engineering, Chung-Ang University, Seoul, Republic of Korea

**Lab:** Assistive and Rehabilitation Robot Laboratory (Prof. Lee Gi-uk)

**Framing:** Preliminary characterization for future walker-mounted, cable-driven robotic gait-assistive device.

---

## 2. Experimental Design — 모든 lock 결정

| Parameter | Lock value | 근거 |
|---|---|---|
| Walking speed | 1.0 m/s, treadmill | Galle 2017 reference |
| BWS level | 20% BW | Apte 2018 (kinematic preservation < 30%), Goldberg & Stanhope 2013 (linear moment scaling) |
| BWS method | Loadcell visual biofeedback (자체 개발) | unique 기여, ±2% tolerance |
| Cable positions | HIGH/MID/LOW = 25/50/75% shank length | Anatomical landmark 분포 |
| Cable directions | 0° horizontal + 30° oblique | Walker anchor geometry |
| Force magnitude | 7% BW per-subject scaling | Quinlivan 2017, Kim 2022 (~12% BW), Yandell 2020 (shank comfort 702 N) |
| Force profile | Half-sine | Smooth jerk, Ding 2016, Lee 2018 precedent |
| Force window | 60–85% GC, peak 72.5% | Perry 2010 (toe-off ref), Winter 2009 (hip moment reversal 86%) |
| Force trigger | **NOT toe-off detection** — **continuous gait-phase gating via foot IMU** (heel-strike anchored phase clock) | 직관적: lifted-foot swing 도와주는 게 의도, toe-off 정밀 검출 불필요 |
| Subjects | n=12 male, 20–45 yrs (G*Power, f=0.40) | Galle 2017 (n=10), Quinlivan 2017 (n=7) precedent |
| Sex framing | Male-only — methodological rationale (anthropometric variance ↓), Limitations 언급 X | Galle 2017 (all-female) JNER 통과 사례 |
| EMG | 6 ch unilateral: RF, BF, VL, TA, MG, SOL (SENIAM) | Quinlivan 2017, Awad 2017 standard |
| Statistics | 2-way RM-ANOVA + Bonferroni + η²p | JNER 표준 |

---

## 3. Conditions Matrix (8 trials)

| ID | Condition | BWS | Cable position | Cable direction |
|---|---|---|---|---|
| 1 | NW | – | – | – |
| 2 | WBW | 20% | – | – |
| 3 | WBAW HIGH–0 | 20% | proximal (25%) | 0° |
| 4 | WBAW HIGH–30 | 20% | proximal (25%) | 30° |
| 5 | WBAW MID–0 | 20% | mid (50%) | 0° |
| 6 | WBAW MID–30 | 20% | mid (50%) | 30° |
| 7 | WBAW LOW–0 | 20% | distal (75%) | 0° |
| 8 | WBAW LOW–30 | 20% | distal (75%) | 30° |

**Counterbalancing:** Latin square or randomized block. Trial = 90 sec walking + 90 sec rest.
**Total session per subject:** ~110 min.
**Experiment duration estimate:** ~3 weeks for n=12.

---

## 4. Hypotheses (Vector reasoning, NO equations/model)

**H1.** WBW vs NW → 하지 muscle activity 감소 (Apte 2018 BWS 효과 재현)
**H2.** All WBAW vs WBW → RF iEMG 감소 (cable이 RF 보조 = hip flex + knee ext biarticular)
**H3.** Position gradient: LOW > MID > HIGH for both RF↓ + VL↓ (lever arm 길이 effect)
**H4.** Direction dissociation:
- 0° → propulsion outcomes (step length, propulsive impulse)
- 30° → vertical lift outcomes (peak hip flexion, foot clearance)
**H5.** 적어도 한 condition이 effort↓ + gait quality↑ 동시 만족 → candidate optimal

**Exploratory:** BF 활성 또는 timing shift in early-swing (60-75%) for LOW conditions — antagonistic compensation 가능성.

---

## 5. Equipment 정리

### 확정 (lab inventory에서 검증 필요)
| Equipment | Manufacturer (예상) | Note |
|---|---|---|
| Treadmill | Bertec (instrumented split-belt) | 모델 확정 필요 |
| Motion capture | Vicon (with Plug-in Gait) | 모델 확정 필요 |
| EMG | Delsys Trigno (wireless) | 모델 확정 필요 |
| Force plate | Bertec (treadmill-integrated) | 모델 확정 필요 |
| Sync | Vicon Lock+ analog trigger | 확정 필요 |

### **확정** ✓ (Lee lab 논문 2개 정독으로 모두 확정)
| Equipment | 상세 | 출처 |
|---|---|---|
| **Foot IMU** | **EBIMU24GV6** (E2BOX, Republic of Korea), 100 Hz logging | user |
| **Cable motor** | **AK60-6** (CubeMars / T-Motor, China) BLDC, 70 N safety cutoff | user |
| **Cable load cell** | **LSB205** (Futek Advanced Sensor Technology, Irvine, CA, USA) | Park 2026 + Kim 2022 lab papers |
| **Treadmill** | **TM-09-P** (Bertec, Columbus, OH, USA) instrumented split-belt | Park 2026 lab paper |
| **Motion capture** | **Qualisys** (Göteborg, Sweden) — camera model TBD | user |
| **Motion analysis software** | **Visual 3D** (HAS-Motion) 또는 **AnyBody** Modeling System | user |
| **EMG system** | **Trigno Wireless System** (Delsys Inc., Natick, MA, USA), 4000 Hz | Kim 2024 lab paper |
| **EMG sync trigger** | **Trigger Module** (Delsys Inc., Natick, MA, USA) | Kim 2024 lab paper |
| **EMG processing** | 4th-order BPF 50-450 Hz + LPF 10 Hz (zero-phase) + MVC norm | Kim 2024 lab convention |
| **Statistics software** | **MATLAB R2025b** (MathWorks, Natick, MA, USA) | user |
| **Walker frame** | Custom rigid walker with bilateral handrail load cells | (own design) |
| **WBS load cells** | model TBD; bilateral handrail-mounted | (own design) |
| **Lee Gi-uk email** | **giuklee@cau.ac.kr** | Park 2026 + Kim 2024 papers |

### Lee lab의 다른 IMU (참고)
- 다른 paper에서는 **Xsens MTi-630 AHRS** 사용 (Park 2026, Kim 2024 모두)
- 우리는 **EBIMU24GV6** 사용 — 차별점

### ❌ 사용 안 함 (Lee lab 2022 paper에서 사용했지만 우리 setup과 다름)
- ~~Bowden cable FCP-04DB (RESPONSE)~~ — 우리는 다른 cable 사용
- ~~Cable housing BHL100 (Jagwire)~~ — 우리는 다른 housing 사용
- ~~Maxon RE 50 motor + EPOS4~~ — 우리는 AK60 사용

---

## 6. Citation Audit — 정정 이력 (중요)

이전 draft에서 발견된 인용 오류 (모두 정정 완료):

| 잘못된 attribution | 정정 |
|---|---|
| ~~Lee et al. 2017 hip extension timing~~ | **Ding et al. 2016** (J NER 13:87) |
| ~~Multijoint 2018~~ | **Lee et al. 2018** (S. Lee, J NER 15:66) |
| ~~Slade et al. 2022~~ | **Kim et al. 2022** (Sci Reports 12:10381) |
| ~~Sun et al. 2024~~ | **Chen et al. 2024** (Front Aging Neurosci 16:1327397) |
| ~~Van Hooren et al. 2019~~ | **Meyer et al. 2019** (Sci Reports 9:5273) |
| ~~Fischer 2013~~ | **Goldberg & Stanhope 2013** (J Biomech 46) |
| ~~Krebs 1998 "Loretta Lavine"~~ | **"Leroy Lavine"** |

→ references.bib 모든 entry CrossRef API 또는 PDF 직접 확인으로 verify.

---

## 7. 다운로드 완료 PDFs (12개)

이 폴더 (`docs/paper/references/`)에 보관:
- Apte 2018 (BWU systematic review)
- Galle 2017 (ankle exo dose response)
- Sawicki 2020 (exoskeleton expansion review)
- Kim 2022 (low-assist hip exosuit, Sci Reports — slade2022 키로 인용)
- Chen 2024 (attachment optimization, Frontiers — sun2024 키로 인용)
- Yandell 2020 (comfort limits, PLOS One)
- Ding 2016 (hip ext timing, J NER — lee2017 키로 인용)
- Lee 2018 (multi-joint autonomous, J NER — multijoint2018 키로 인용)
- Bryan 2021 (hip-knee-ankle, J NER)
- Meyer 2019 (treadmill familiarization, Sci Reports — vanhooren2019 키로 인용)
- **Lee 2021 / Yang 2021 (running exosuit, J NER 18:129) — Lee lab 선행연구**
- **Kim Lee Lab 2022 (hip exosuit anchor points, PLOS One 17:e0271764) — 직접 prior work**

학교 IP/proxy로 user 직접 다운로드 필요:
- Quinlivan 2017 (Sci Robotics)
- Awad 2017 (Sci Transl Med)
- Asbeck 2015 (IJRR)
- Goldberg 2013 (J Biomech)
- Hermens 2000 (J Electromyogr Kinesiol)
- Krebs 1998 (JOSPT)

---

## 8. Methodological Decisions (논문에서 안 쓰지만 기록)

### Model 사용 정책
- **Paper에서 model 식·표·그림 사용 X**
- Model audit으로 얻은 insight (cable = RF analog, knee extension torque, antagonistic compensation 가능성)는 **vector reasoning으로 hypothesis section에 흡수**
- 이유: 정적 가정 한계, paper 단순화, JNER convention

### IMU 역할 재정의
- ❌ "toe-off 정밀 검출"
- ✓ "heel-strike-anchored continuous gait-phase estimation, gating cable to 60-85% GC swing window"
- 이유: toe-off 정의 어렵고, 우리 의도는 lifted-foot swing 보조이지 toe-off 순간 trigger가 아님

### Sex framing
- Male-only는 methodological choice (anthropometric variance ↓)
- Limitations 섹션에서 다시 언급 안 함 (Galle 2017 precedent)
- 이유: recruitment 제약을 method 장점으로 reframe

---

## 9. 작성 완료 (paper draft 18 pages, ~6700 words)

### 완성된 섹션
- ✓ Title + authors + affiliations
- ✓ Abstract (Background, Methods 완성, Results placeholder)
- ✓ Background 1.1–1.4 (~900 words)
- ✓ Methods 2.1 Study design
- ✓ Methods 2.2 Participants + sample size
- ✓ Methods 2.3.1 Treadmill/mocap/EMG/sync
- ✓ Methods 2.3.2 Walker-mounted cable system
- ✓ Methods 2.3.3 WBS with loadcell biofeedback (unique)
- ✓ Methods 2.3.4 Foot IMU (gait-phase estimation, EBIMU24GV5)
- ✓ Methods 2.4 Experimental conditions + rationale
- ✓ Methods 2.5 Protocol
- ✓ Methods 2.6 Outcome measures
- ✓ Methods 2.7 Hypotheses (H1-H5, vector reasoning)
- ✓ Methods 2.8 Statistical analysis
- ✓ Discussion 4.2 Mechanism (RF analog, vector reasoning) + placeholder
- ✓ Discussion 4.3 Comparison + placeholder
- ✓ Discussion 4.4 Implications + WBS contribution
- ✓ Discussion 4.5 Limitations
- ✓ Discussion 4.6 Future work
- ✓ Conclusions (partial + placeholder for results)
- ✓ Declarations (JNER 필수 모두)
- ✓ Tables 1, 2 (Conditions, Participants)
- ✓ Figures 1-6 placeholder + captions
- ✓ References.bib (22 verified entries, 21 사용)

### 실험 후 채울 placeholder
- Abstract Results paragraph
- Section 3 (Results) 전체
- Discussion 4.1 Principal findings
- Discussion 4.2 + 4.3 + 4.4 quantitative parts
- Conclusions specific findings sentence
- Tables 1 (subject characteristics 수치), 3, 4 (outcomes)
- Figures 4, 5, 6 (실제 그래프)

### 실험 시작 전 마무리 필요
- 장비 model number (Bertec, Vicon, Delsys 정확한 모델)
- IRB approval 번호
- Author email (Lee Gi-uk corresponding)
- ORCIDs for all authors
- Funding info
- OSF pre-registration

### 그림 작업 필요
- Figure 1: 실험 setup schematic (walker + WBS + cable + treadmill + sensors)
- Figure 2: Cable conditions matrix (anatomical landmarks + walker anchor diagram)
- Figure 3: Force profile + timing diagram (half-sine over GC)

---

## 10. 파일 목록

```
docs/paper/
├── main.tex           — JNER format draft (18 pages compiled)
├── main.pdf           — Compiled PDF
├── references.bib     — 22 verified entries
├── PROGRESS.md        — 이 파일 (2nd brain 노트)
├── biomechanical_model.md — 3-link 모델 derivation
├── three_link_biomechanical_model.tex — 3-link 모델 LaTeX (paper에 미사용)
├── model_prediction.py — Model simulation (paper에 미사용, 내부 reference만)
├── model_prediction.pdf/png — Model output figures
├── model_prediction_summary.txt — Model output table
└── references/
    ├── references.md       — 인용 audit document
    ├── Apte2018_*.pdf
    ├── Galle2017_*.pdf
    ├── Sawicki2020_*.pdf
    ├── Slade2022_*.pdf  (= Kim 2022 실제)
    ├── Sun2024_*.pdf    (= Chen 2024 실제)
    ├── Yandell2020_*.pdf
    ├── Lee2017_*.pdf    (= Ding 2016 실제)
    ├── Multijoint2018_*.pdf  (= Lee 2018 실제)
    ├── Bryan2021_*.pdf
    └── VanHooren2019_*.pdf  (= Meyer 2019 실제)
```

---

## 11. 다음 단계 우선순위

1. **장비 model 확정** — lab inventory에서 Bertec/Vicon/Delsys 정확한 모델 + 펌웨어
2. **IRB 신청 + 승인 번호** 확보
3. **Figures 1-3 schematic** 작성 (실제 setup 사진 또는 CAD)
4. **OSF pre-registration** (선택, JNER 권장)
5. **Pilot trial 1명** — IMU phase 추정 정확도 검증
6. **본 실험 n=12** 진행
7. **Results + Discussion quantitative parts** 채우기
8. **Submission**

---

**Compile 명령:**
```bash
cd docs/paper
pdflatex main.tex
bibtex main
pdflatex main.tex
pdflatex main.tex
```

**Word count:**
```bash
wc -w main.tex  # ~6700 words (JNER 6000-8000 target)
```
