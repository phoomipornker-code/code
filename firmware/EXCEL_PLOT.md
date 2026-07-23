# พล็อตกราฟจาก Serial ลง Excel

ค่าถูก**เลื่อนแกน**ให้เส้นแต่ละอันอยู่คนละช่วง (พล็อตซ้อนกราฟเดียวแล้วไม่ทับกัน)

```text
CSV	t_s	Vout	Iplot	Dplot	Vinplot
CSV	12.3	53.97	110.00	236.0	447.7
```

| เส้น | ช่วงบนกราฟ | แปลงกลับค่าจริง |
|------|-------------|------------------|
| Vout | ~50–56 | ใช้ตรง ๆ (โวลต์) |
| Iplot | ~100–160 | I(A) = (Iplot − 100) / 20 |
| Dplot | ~200–245 | Duty(%) = Dplot − 200 |
| Vinplot | ~400–570 | Vin(V) = Vinplot − 300 |

## ขั้นตอน

1. คัดลอกบรรทัดขึ้นต้น `CSV`
2. Excel → Text to Columns → **Tab**
3. Insert Chart (Line): X = `t_s`, Y = `Vout` + `Iplot` + `Dplot` + `Vinplot` ทั้งสี่เส้น
