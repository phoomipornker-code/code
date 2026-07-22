# Changelog v39 — cv58-boost-v14-forward-v39

## CV = hold constant voltage (not current taper)

User clarification: CV must **maintain Vbat ≈ 56 V**, not force-reduce current.

| V vs 56 V | Action |
|-----------|--------|
| V &lt; 56 − hold band | step duty **up** (reach/hold voltage) |
| \|V − 56\| in hold band (±0.08 V) | **hold duty** (constant voltage) |
| V &gt; 56 + hold band | step duty **down** |

Removed iCap current-taper CV from v38. Current falls naturally as pack fills while V is held.  
CC over-current outer cut disabled in CV (only hard Ibat limit remains).
