# Physical Constants, Real Numbers, and Verification

A complete, code-checkable specification from nothingness (bottom)

## Summary

A "physical constant" is not a metaphysical number. It is a verifiable object produced by a measurement contract: a finite witness bundle + a total verifier that yields a certified value at a declared resolution. In a closed, witness-first universe, constants split into exactly two classes:

1. **Defined constants** (exact by convention): they define units.
2. **Inferred constants** (measured): they are returned as certified intervals with receipts; digits are derived from the interval.

Real numbers appear only as refinement processes (nested intervals with explicit error bounds), never as assumed completed infinities.

This document gives:
- the forced mathematics
- the full classification of famous constants
- exact derivation relations
- and a complete verification+code protocol so values come from code under any chosen unit system

## Verification Code

<div className="verification-code-section">

- [constants_verify.py](https://github.com/ravish-oo/opoch-structural-reality-kernel/blob/main/structural_reality_kernel/empirical_evidences/constants_verify.py) — Main verification suite (5 checks)
- [physical_constants.py](https://github.com/ravish-oo/opoch-structural-reality-kernel/blob/main/structural_reality_kernel/empirical_evidences/physical_constants.py) — Physical constants definitions
- [measurement_contract.py](https://github.com/ravish-oo/opoch-structural-reality-kernel/blob/main/structural_reality_kernel/empirical_evidences/measurement_contract.py) — Measurement contract framework

</div>

---

## 1. What is a "Number" Here?

### 1.1 The Only Forced Numeric Substrate is Finite Distinguishability

A finite run fixes a finite domain D_0. Evidence is a ledger L, and survivors are:

```
+----------------------------------------------------------+
|  W(L) = {x in D_0 : for all (tau, a) in L, tau(x) = a}   |
+----------------------------------------------------------+
```

The only Pi-fixed invariant of "how much remains" is |W|, and the unique additive scalar of multiplicative counts is log|W|. So:
- counts are primary
- logs are forced when you want additivity

### 1.2 Naturals, Integers, Rationals are Finite Witnesses

- **N**: sizes of finite sets (witness = enumeration with no duplicates)
- **Z**: differences of counts (pairs modulo cancellation)
- **Q**: ratios of counts (pairs modulo scaling)

All are fully finite.

### 1.3 Real Numbers (Operational, No Infinity Assumed)

A "real number" is a certified refinement process:
- A nested interval sequence: [L_1, U_1] >= [L_2, U_2] >= ... with rational endpoints
- A verification rule that each interval is correct given its witness bundle

A digit string is derived, not fundamental: the guaranteed digits are the maximal common prefix of decimal expansions of all numbers in [L_n, U_n].

---

## 2. What is a "Physical Constant"?

### 2.1 The Only Admissible Definition: A Measurement Contract

A constant is defined by a finite contract:

```
+----------------------------------------------------------+
|  C = (ValueFormat, WitnessFormat, V, Canon, Units)        |
+----------------------------------------------------------+
```

where:
- **ValueFormat**: exact rational, or certified interval [L,U], or digits with error bound
- **WitnessFormat**: raw data + instrument configuration + calibration artifacts
- **V**: total verifier returning PASS/FAIL and (on PASS) the certified output value object
- **Canon**: canonicalization removing slack (ordering, encodings, unit tags)
- **Units**: declared unit gauge (SI or another unit convention)

Kernel output rule is absolute:
- UNIQUE+witness if the contract forces one value object
- else Omega frontier with the exact surviving value family and the minimal missing separator

---

## 3. Why Unit Choice is Gauge and Why Dimensionless Constants are Special

A unit system is a recoding of descriptions. Under gauge:
- if a "constant value" changes under unit recoding, the number is not Pi-fixed; it is convention + calibration.

Therefore:
- **dimensionless constants** are the only candidates for universal numerical invariants across unit gauges
- **dimensional constants** must be treated as definitions (if fixed) or calibrations (if measured)

---

## 4. The SI Defining Constants (Exact by Definition)

The SI is now defined by fixing exact numerical values for seven defining constants.

### Exact Defining Constants (SI)

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| Cesium hyperfine frequency | Delta_nu_Cs | 9,192,631,770 | Hz |
| Speed of light | c | 299,792,458 | m/s |
| Planck constant | h | 6.62607015 x 10^-34 | J s |
| Elementary charge | e | 1.602176634 x 10^-19 | C |
| Boltzmann constant | k_B | 1.380649 x 10^-23 | J/K |
| Avogadro constant | N_A | 6.02214076 x 10^23 | mol^-1 |
| Luminous efficacy | K_cd | 683 | lm/W |

**Interpretation**: these are not "measured constants" in the SI - they are definitions. Their "verification" is purely formal: the unit system is defined so these equalities hold exactly.

---

## 5. Derived Exact Constants (Pure Algebra Once SI Definitions are Set)

From the exact SI constants, many famous "constants" become exact derived values:

### Reduced Planck Constant

```
hbar = h / (2 * pi)
```

Here pi is a mathematical constant computed by an algorithm with a certified error bound; this is a real-number refinement object.

### Josephson Constant

```
K_J = 2e / h    (exact, since e, h exact)
```

### Von Klitzing Constant

```
R_K = h / e^2   (exact, since e, h exact)
```

These are "constants" only in the sense of exact algebraic consequences of the chosen unit definitions.

---

## 6. Measured Dimensionless Constant: The Anchor of Electromagnetism

The key dimensionless constant is the fine-structure constant:

```
+----------------------------------------------------------+
|  alpha = e^2 / (4 * pi * eps_0 * hbar * c)                |
+----------------------------------------------------------+
```

CODATA 2022 gives (with uncertainty):

```
alpha = 7.2973525643(11) x 10^-3
alpha^-1 = 137.035999177(21)
```

In this structure:
- alpha is NOT defined by SI
- it is an **inferred constant**: returned as a certified interval [L,U] at stated confidence

---

## 7. Vacuum Permittivity/Permeability are Derived (Not Exact)

Because c, h, e are exact but alpha is measured, the electromagnetic "vacuum constants" become derived with uncertainty:

### Magnetic Constant (Vacuum Permeability)

```
mu_0 = (4 * pi * alpha * hbar) / (e^2 * c) = (2 * alpha * h) / (e^2 * c)
mu_0 = 1.25663706127(20) x 10^-6 N A^-2
```

### Electric Constant (Vacuum Permittivity)

```
eps_0 = 1 / (mu_0 * c^2)
eps_0 = 8.8541878188(14) x 10^-12 F m^-1
```

**Key point**: this is the correct modern resolution of a long-standing confusion: mu_0 and eps_0 are no longer exact definitions; they inherit the uncertainty of alpha.

---

## 8. The Gravitational Constant and Dark-Sector Scale Parameters

CODATA 2022 lists:

```
G = 6.67430(15) x 10^-11 m^3 kg^-1 s^-2
```

This is an inferred constant with a wide frontier relative to many others (measurement-limited). In this system it is always returned as an interval at a declared confidence, never as a metaphysical "true number."

---

## 9. "All Famous Constants" as a Complete, Verifiable Table Schema

Every constant must be one of:

### Type A - Defined (Exact)
- value is exact by unit definition (SI defining constants, and exact derived constants)

### Type B - Inferred (Interval)
- value is a certified interval [L,U] at a declared confidence; digits are derived

### Mandatory Metadata for Each Constant

- symbol, name
- dimension (unit exponents in chosen unit gauge)
- type = defined_exact or inferred_interval
- value:
  - exact rational (for defined), or
  - interval endpoints (for inferred)
- uncertainty (encoded as interval width; never just prose)
- relations (how it is computed from base constants)
- witness_bundle_hash (data)
- verifier_hash (code)
- canon_hash (canonicalization rules)
- receipt_hash (SHA-256 of canonical JSON)

---

## 10. Complete Verification Protocol

Verification is not "believe NIST." Verification is:

1. Run the verifier on the witness bundle
2. Reproduce the certified value object (exact rational or interval)
3. Check canonical receipts and hashes
4. Check gauge invariance under recoding
5. Derive digits only from certified intervals

### 10.1 Verification Object for an Inferred Constant

A verifier must output:
- certified interval endpoints (L, U) as rationals
- and a proof that the witness implies c in [L, U]

Digits shown are computed from [L, U] only.

### 10.2 Gauge Invariance Check

Take the same witness bundle and apply canonical recodings:
- reorder logs
- rename fields
- change serialization
- apply unit-recoding maps where applicable

The verifier must return the same canonical interval object (or an explicitly equivalent one) and the same Pi-fixed fingerprints. If not, the pipeline is minted and invalid.

---

## 11. Verification Specification

### A) Totality Verification
- Verifier never returns undefined
- All inputs produce PASS/FAIL with value or reason

### B) Gauge Invariance Verification
- Recodings don't change Pi-fixed outputs
- Dimensionless constants unchanged under unit transformation

### C) Interval Correctness Verification
- `lo <= hi` always
- Interval arithmetic propagation correct
- Derived intervals contain true values

### D) Consistency Verification
- Exact constants match SI definitions
- Derived constants computed correctly from bases
- Relations between constants preserved

### E) Canonical Receipts Verification
- All receipts serializable as canonical JSON
- SHA-256 hashes reproducible
- Hashes invariant under re-computation

---

## Canonical Receipt Schemas

### MEASUREMENT_CONTRACT Receipt
```json
{
  "type": "MEASUREMENT_CONTRACT",
  "constant_symbol": "<string>",
  "constant_type": "defined_exact | inferred_interval",
  "value_format": "<string>",
  "verifier_total": true,
  "contract_hash": "<sha256>",
  "result": "PASS"
}
```

### EXACT_CONSTANT Receipt
```json
{
  "type": "EXACT_CONSTANT",
  "symbol": "<string>",
  "numerator": "<integer>",
  "denominator": "<integer>",
  "unit": "<string>",
  "is_si_defining": true,
  "constant_hash": "<sha256>",
  "result": "PASS"
}
```

### INTERVAL_CONSTANT Receipt
```json
{
  "type": "INTERVAL_CONSTANT",
  "symbol": "<string>",
  "lo_numerator": "<integer>",
  "lo_denominator": "<integer>",
  "hi_numerator": "<integer>",
  "hi_denominator": "<integer>",
  "unit": "<string>",
  "confidence": "<string>",
  "interval_hash": "<sha256>",
  "result": "PASS"
}
```

### GAUGE_INVARIANCE Receipt
```json
{
  "type": "GAUGE_INVARIANCE",
  "constant_symbol": "<string>",
  "original_hash": "<sha256>",
  "recoded_hash": "<sha256>",
  "hashes_match": true,
  "result": "PASS"
}
```

### CONSTANTS_BUNDLE Receipt
```json
{
  "type": "CONSTANTS_BUNDLE",
  "bundle_id": "<string>",
  "si_defining_count": "<integer>",
  "derived_exact_count": "<integer>",
  "inferred_count": "<integer>",
  "all_verified": true,
  "bundle_fingerprint": "<sha256>",
  "result": "PASS"
}
```

---

## Closing Statement

**Numbers are finite invariants; reals are certified refinements; constants are verified contracts.**

- Exact SI constants are definitions
- Inferred constants are intervals
- Digits are derived from intervals
- Unit changes are gauge recodings
- The entire pipeline is reproducible from code with receipts

**No metaphysics. Only verified measurement contracts with canonical receipts.**
