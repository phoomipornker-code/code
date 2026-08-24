# code

ESP32 dual-path charger for 16S LiFePO4, plus a CC/CV PI learning module.

## เรียน CC/CV PI control

เริ่มที่ **[docs/learn-cc-cv-pi.md](docs/learn-cc-cv-pi.md)**

```bash
python3 learn/simulate_cc_cv.py
python3 learn/simulate_cc_cv.py --lab p-vs-pi
python3 -m unittest discover -s learn -p 'test_*.py'
```

โครงที่ใช้ในบทเรียน (และในเฟิร์มแวร์ Forward):

```text
CC:  current PI  → duty          (Iref = 5 A)
CV:  voltage PI  → Iref
     current PI  → duty
```

## Firmware (other branches)

Live Boost/Forward sketches and MATLAB/Simulink helpers live on topic branches such as `cursor/forward-pi-complete-f7e4` (`cv58-boost-v14-forward-pi/`, CV **57.6 V**).
