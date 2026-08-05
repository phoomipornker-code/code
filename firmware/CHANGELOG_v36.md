# Changelog v36 — cv58-boost-v14-forward-v36

## Stuck at duty ~31% with Ibat ~0.4 A

Field: `duty_acc=317`, `Iref=3A`, `Ibat≈0.37A`, Vac≈165 V — not climbing.

Cause: CC step had a **hard ceiling** `duty < dutyFf + 40`.  
At high Vac, `dutyFf ≈ 276` → stop at ~316 even though current still far below 3 A.

Fix: remove FF ceiling — step up whenever `I < Iref−band` until band or Dmax (still freeze if AC&lt;115 V).
