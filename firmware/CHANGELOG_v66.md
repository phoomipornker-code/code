# Changelog v66 — cv58-boost-v14-forward-v66

## CSV เลื่อนแกน Y ให้แต่ละเส้นห่างกันบนกราฟเดียว

```text
CSV	t_s	Vout	Iplot	Dplot	Vinplot
```

| คอลัมน์ | ค่าที่ส่ง | ค่าจริง |
|---------|-----------|---------|
| Vout | Vf | = Vout |
| Iplot | I×20 + 100 | I = (Iplot−100)/20 |
| Dplot | Duty% + 200 | Duty = Dplot−200 |
| Vinplot | Vin + 300 | Vin = Vinplot−300 |

แถบประมาณบนกราฟ: Vout ~55 · I ~100–160 · Duty ~200–245 · Vin ~400–570

Flash: `[BOOT] cv58-boost-v14-forward-v66 | …`
