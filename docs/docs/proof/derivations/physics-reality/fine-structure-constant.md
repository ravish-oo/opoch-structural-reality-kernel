# The Fine-Structure Constant α

Complete Derivation with Certified Intervals and Replayable Receipts

## Summary

The fine-structure constant α is the dimensionless coupling invariant of electromagnetism. Being dimensionless, it survives unit recodings (gauge transformations) and is a valid Π-fixed constant candidate. This document specifies the complete, proof-carrying procedure for deriving α.

**Critical principle**: This procedure never outputs a naked number. It outputs either:
- A **certified interval** for α (and derived digits), with a replayable receipt, or
- A **frontier** that is still consistent with all witnesses, plus the single cheapest missing separator needed to collapse it further.

---

## 1. What α Is (The Only Admissible Definition)

α is defined as:

```
α = e² / (4πε₀ℏc) ≈ 1/137.036
```

where:
- e = elementary charge
- ε₀ = vacuum permittivity
- ℏ = reduced Planck constant
- c = speed of light

Since c and ℏ are exact by SI definition (2019), α encodes the electromagnetic coupling strength as a pure number.

**Dimensionless invariant**: α does not depend on unit choices. Any valid measurement route must yield the same interval (within uncertainties).

---

## 2. The Witness Contract for α

### 2.1 Contract Object

Define the contract:

```
P_α = (A, W_wit, V, c, B, Canon)
```

with:

- **Answer space A**: Rational intervals [L, U] (endpoints as exact rationals)
- **Witness space W_wit**: Bundles of experimental data + theory coefficients
- **Total verifier V**: Checks that a proposed α interval is implied by witnesses
- **Cost c**: Explicit (experiment cost, compute cost)
- **Budget B**: Allowed cost/precision threshold
- **Canonicalization Canon**: Deterministic JSON schema + sorted keys + explicit units

### 2.2 Required Invariants

1. **No floats in receipts**: Store rationals (fractions) and integer counts
2. **All tests total**: Any failure is explicit (FAIL/TIMEOUT), never undefined
3. **Gauge invariance**: Results identical under equivalent serializations

---

## 3. Witness Channels (Independent Separators)

To collapse α robustly, we use independent measurement routes with different physics and systematics.

### Channel A: Electron Anomalous Magnetic Moment (g−2) Route

**Physical basis**: The electron's magnetic moment differs from the Dirac value by the anomaly a_e, which depends on α through QED loop corrections.

**Witness bundle W_A**:
- Measured electron anomaly a_e^exp with uncertainty
- QED perturbation coefficients {C_n} (as rationals with audit trail)
- Hadronic and electroweak contributions a_had, a_weak with uncertainties
- Truncation order N and remainder bound R_N(α)

**Prediction function**:

```
a_e(α) = Σ_{n=1}^{N} C_n (α/π)^n + a_had + a_weak + r(α)
```

where |r(α)| ≤ R_N(α) is the certified truncation remainder.

**Verifier procedure**:
1. Pick initial bracket [α_min, α_max] containing α
2. For candidate α, compute prediction interval:
   ```
   I_pred(α) = [a_e(α) - R_tot, a_e(α) + R_tot]
   ```
3. Measurement gives:
   ```
   I_exp = [a_e^exp - δ, a_e^exp + δ]
   ```
4. PASS if I_pred(α) ∩ I_exp ≠ ∅
5. Output tightest α interval by certified bisection

### Channel B: Atom Recoil Route (h/m + Rydberg)

**Physical basis**: The Rydberg constant relates α to fundamental constants:

```
R_∞ = α² m_e c / (2h)
```

Solving for α²:

```
α² = 2h R_∞ / (m_e c)
```

**Operational formula** (using recoil measurement):

```
α² = (2R_∞/c) × (A_r(X)/A_r(e)) × (h/m_X)
```

where:
- R_∞ = Rydberg constant (measured)
- h/m_X = Planck constant over atom mass (from recoil)
- A_r(X), A_r(e) = relative atomic masses
- c = speed of light (exact by SI)

**Witness bundle W_B**:
- Certified interval for R_∞
- Certified interval for h/m_X (atom interferometry)
- Certified intervals for A_r(X) and A_r(e)
- Exact c, h (SI definitions as exact rationals)

**Verifier procedure**:
1. Build interval I_{α²} from product of input intervals
2. Take square root as certified interval operation:
   ```
   I_B = [√L, √U]
   ```

---

## 4. Channel Coequalization (Global Closure)

Let each channel j produce certified interval I_j = [L_j, U_j].

### Forced Truth Merge (Π-fixed)

The surviving α set is the intersection:

```
I_∩ = ∩_j I_j
```

**Case 1: I_∩ ≠ ∅**
- α is certified to lie in that interval (UNIQUE at resolution)
- Output the interval with full receipt

**Case 2: I_∩ = ∅**
- Witness bundles are inconsistent under declared uncertainty model
- Output: FRONTIER with conflict witness (which intervals failed to overlap)
- Include: minimal separator to resolve inconsistency

---

## 5. Deterministic Next Separator τ*

When frontier is not tight enough (need more digits) or channels disagree:

**Minimax separator principle**:
- Candidate space D_0 = discretized α intervals
- Each measurement τ partitions candidates by outcomes
- Cost c(τ) = expected resource cost
- Choose τ* = cheapest test maximally shrinking frontier

**Practical options for α**:
- Improve a_e^exp precision (Channel A)
- Improve h/m_X or mass ratios (Channel B)
- Choose based on shrink-per-cost ratio

---

## 6. Output Format (What "α" Means)

The constant is NOT "0.007297..." as a naked decimal.

The constant IS the object:
- Certified rational interval [L, U]
- Full proof bundle that replays verifiers
- Hash receipt

### Digits are Derived, Not Asserted

Displayed digits = maximal prefix common to all numbers in [L, U].

For n correct digits, must shrink [L, U] below 10^{-n} width (certified).

---

## 7. Complete Verification Bundle

### A) Canonical Contract
```json
{
  "type": "ALPHA_CONTRACT",
  "answer_space": "rational_intervals",
  "witness_space": ["channel_A", "channel_B"],
  "verifier_ids": ["V_A", "V_B"],
  "canonicalization": "JSON_sorted_keys",
  "unit_gauge": "SI_2019",
  "result": "PASS"
}
```

### B) Witness Bundle Schema
```json
{
  "type": "ALPHA_WITNESS_BUNDLE",
  "channel": "<A|B>",
  "data_hash": "<sha256>",
  "coefficients_hash": "<sha256>",
  "uncertainty_model": "<explicit>",
  "provenance": "<source>",
  "result": "PASS"
}
```

### C) Channel Result Schema
```json
{
  "type": "ALPHA_CHANNEL_RESULT",
  "channel_id": "<string>",
  "interval_lower_num": "<integer>",
  "interval_lower_den": "<integer>",
  "interval_upper_num": "<integer>",
  "interval_upper_den": "<integer>",
  "width": "<rational>",
  "receipt_hash": "<sha256>",
  "result": "PASS"
}
```

### D) Intersection Result Schema
```json
{
  "type": "ALPHA_INTERSECTION",
  "channels_count": "<integer>",
  "intervals_compatible": "<boolean>",
  "final_interval_lower": "<rational>",
  "final_interval_upper": "<rational>",
  "certified_digits": "<integer>",
  "frontier_status": "<UNIQUE|FRONTIER>",
  "result": "PASS"
}
```

### E) Final α Object Schema
```json
{
  "type": "ALPHA_CERTIFIED",
  "alpha_id": "<string>",
  "interval_lower": "<integer_ratio>",
  "interval_upper": "<integer_ratio>",
  "certified_digits": "<integer>",
  "inverse_interval": "[136.xxx, 137.xxx]",
  "channel_hashes": ["<sha256>", ...],
  "bundle_fingerprint": "<sha256>",
  "result": "PASS"
}
```

---

## 8. Derivation Algorithm

```
INPUT:
  W_A, W_B    // witness bundles
  V_A, V_B    // total verifiers
  Canon       // canonicalization rules

STEP 1: I_A = V_A(W_A) -> Interval[L_A, U_A] + receipt_A

STEP 2: I_B = V_B(W_B) -> Interval[L_B, U_B] + receipt_B

STEP 3: I_cap = intersect(I_A, I_B)
  if I_cap empty:
    output FRONTIER:
      - I_A, I_B
      - minimal separator: which channel tightened would restore overlap
      - exact gap: distance between intervals
    stop

STEP 4: output UNIQUE object:
  alpha_interval = I_cap
  digits = common_prefix_digits(I_cap)
  receipt = SHA256(Canon({I_A, I_B, I_cap, verifier_hashes}))

OPTIONAL REFINEMENT LOOP:
  while desired_digits not achieved:
    choose τ* via costed shrink estimate
    request new witness bundle
    repeat steps 1-4
```

---

## 9. Current Best Values (PDG 2022)

### Channel A (Electron g-2)
- a_e^exp = 0.001 159 652 180 59 (13)
- Yields: α^{-1} = 137.035 999 150 (33)

### Channel B (Rb recoil)
- h/m_Rb via atom interferometry
- Yields: α^{-1} = 137.035 999 206 (11)

### Combined (CODATA 2018)
- α = 7.297 352 5693 (11) × 10^{-3}
- α^{-1} = 137.035 999 084 (21)
- Relative uncertainty: 1.5 × 10^{-10}

**As certified interval** (11 significant figures):
```
α ∈ [7297352568/10^12, 7297352570/10^12]
```

---

## 10. What This Achieves

1. **α is a provable object**: interval + receipts, not a naked number
2. **Increasing precision = explicit separators**: each digit requires a witnessable measurement
3. **Disagreement is never handwaved**: becomes conflict witness + cheapest next test
4. **Complete closure**: derive α by collapsing frontier with witnesses

**No mystery. No guessing. Only structure, calibration, and canonical receipts.**
