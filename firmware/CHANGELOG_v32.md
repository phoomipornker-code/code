# Changelog v32 — cv58-boost-v14-forward-v32

## Field: AC 3 V was a temporary sag (แรงดันตกชั่วคราว)

Not a full shutdown event by intent — PWM should pause and resume.

| Behavior | v32 |
|----------|-----|
| AC &lt; `MIN_AC` | **Soft suspend** (duty=0), keep `system_ON` |
| Sag recovers | Resume **SoftStart** |
| Sustained AC loss | Auto-Shutdown only after **15 s** (was 2 s) |
| AC raw hold | Only on **multi-ch bus glitch** (AC+BAT both junk) — do **not** mask a real sag |

Forward CC still **3 A**. Boost unchanged.
