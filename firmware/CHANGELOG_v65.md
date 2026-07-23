# Changelog v65 — cv58-boost-v14-forward-v65

## CSV เหลือเฉพาะค่าที่ต้องพล็อต

คอลัมน์:

```text
CSV	t_s	Vin	Vout	Iout	Duty
```

| คอลัมน์ | ความหมาย |
|---------|----------|
| t_s | เวลา (วินาที) |
| Vin | แรงดันเข้า (PV หรือ AC bridge) |
| Vout | แรงดันออก / แบต (Vf) |
| Iout | กระแสออก / แบต (If) |
| Duty | Duty % |

วิธี Excel เหมือนเดิม — คัดลอกบรรทัด `CSV` → Text to Columns → Tab

Flash: `[BOOT] cv58-boost-v14-forward-v65 | …`
