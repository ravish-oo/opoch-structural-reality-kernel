# The Standard Model

Complete Structure, Parameter Map, and Verification Protocol

## Summary

The Standard Model (SM) is the minimal local quantum field theory that matches the observed interaction structure: non-abelian gauge forces (color and weak), an abelian hypercharge force, chiral fermions, and a single scalar that breaks electroweak symmetry and generates masses. Once you fix the gauge group, field content, and consistency requirements (locality + gauge invariance + quantum consistency/anomaly cancellation), the form of the SM Lagrangian is forced. What remains are parameters (couplings, masses, mixings) that are not guessed: they are calibration invariants determined by a reproducible global fit to data under a declared renormalization scheme and scale.

---

## Immediate Impact

- **Removes confusion**: separates what is structurally fixed (symmetries + allowed terms) from what must be measured (parameter values).
- **Makes "numbers" mechanical**: every SM number becomes an output of code from a specified input dataset, scheme, and scale—producing intervals + covariance, not vibes.
- **Turns disagreements into tests**: if two groups disagree, the disagreement is located in (i) data, (ii) theory order, (iii) scheme/scale choice, or (iv) missing separators; nothing else.

## Verification Code

<div className="verification-code-section">

- [sm_verify.py](https://github.com/ravish-oo/opoch-structural-reality-kernel/blob/main/structural_reality_kernel/empirical_evidences/sm_verify.py) — Main verification suite
- [sm_structure.py](https://github.com/ravish-oo/opoch-structural-reality-kernel/blob/main/structural_reality_kernel/empirical_evidences/sm_structure.py) — Particle families and gauge structure
- [sm_parameters.py](https://github.com/ravish-oo/opoch-structural-reality-kernel/blob/main/structural_reality_kernel/empirical_evidences/sm_parameters.py) — Coupling constants and parameters

</div>

---

## 1. What is Being Specified: The SM as a Constrained Action

### 1.1 The Gauge Group

The SM gauge symmetry is:

```
+----------------------------------------------------------+
|  G_SM = SU(3)_c × SU(2)_L × U(1)_Y                       |
+----------------------------------------------------------+
```

with gauge couplings g_s, g, g' respectively.

### 1.2 The Field Content (3 Generations)

For each generation a = 1, 2, 3, the chiral fermions are:

- **Quark doublet**: Q_L^a = (u_L^a, d_L^a)
- **Quark singlets**: u_R^a, d_R^a
- **Lepton doublet**: L_L^a = (nu_L^a, e_L^a)
- **Lepton singlet**: e_R^a

Plus one complex Higgs doublet Phi (an SU(2) doublet with hypercharge).

### 1.3 Quantum Consistency: Anomaly Cancellation

A chiral gauge theory is only consistent if gauge anomalies cancel. In the SM this strongly constrains (and, given mild assumptions, essentially fixes) the hypercharge assignments.

---

## 2. The Forced Lagrangian Form

Once you require: (i) locality, (ii) gauge invariance under G_SM, and (iii) renormalizable (dimension &lt;= 4) operators, the classical SM Lagrangian splits into four pieces:

```
L_SM = L_gauge + L_fermion + L_Higgs + L_Yukawa
```

### 2.1 Gauge Kinetic Terms

Let `G^A_{mu nu}`, `W^I_{mu nu}`, `B_{mu nu}` be the field strengths for SU(3), SU(2), U(1). Then:

```
L_gauge = -1/4 G^A_{mu nu} G^{A mu nu}
        - 1/4 W^I_{mu nu} W^{I mu nu}
        - 1/4 B_{mu nu} B^{mu nu}
```

For non-abelian groups, field strengths include the commutator/self-interaction terms.

### 2.2 Fermion Kinetic Terms (with Covariant Derivative)

Each fermion field psi has:

```
L_fermion = sum_psi psi_bar i gamma^mu D_mu psi
```

The covariant derivative is:

```
D_mu = partial_mu
     - i g_s G_mu^A T^A
     - i g W_mu^I tau^I/2
     - i g' Y B_mu
```

with representation matrices T^A (color), Pauli matrices tau^I (weak isospin), and hypercharge Y.

### 2.3 Higgs Sector

```
L_Higgs = (D_mu Phi)^dagger (D^mu Phi) - V(Phi)

V(Phi) = -mu^2 Phi^dagger Phi + lambda (Phi^dagger Phi)^2
```

Spontaneous symmetry breaking occurs when mu^2 > 0 and Phi acquires a vacuum expectation value (VEV).

### 2.4 Yukawa Interactions (Fermion Masses After Symmetry Breaking)

```
L_Yukawa = -Q_L_bar Y_u Phi_tilde u_R
         - Q_L_bar Y_d Phi d_R
         - L_L_bar Y_e Phi e_R
         + h.c.
```

where Y_u, Y_d, Y_e are 3x3 complex Yukawa matrices and Phi_tilde = i tau^2 Phi*.

**Quantization note**: to compute predictions you also add gauge-fixing and ghost terms (BRST structure). These do not change the classical content; they are part of the total verifier for perturbative calculations.

---

## 3. Electroweak Symmetry Breaking and the Mass Relations

Let the Higgs VEV be v. In unitary gauge, `<Phi> = (0, v/sqrt(2))^T`. Then:

```
+----------------------------------------------------------+
|  M_W = (1/2) g v                                          |
|  M_Z = (1/2) v sqrt(g^2 + g'^2)                          |
+----------------------------------------------------------+
```

and the electric charge satisfies:

```
e = g sin(theta_W) = g' cos(theta_W)
sin^2(theta_W) = g'^2 / (g^2 + g'^2)
```

Also m_H^2 = 2 lambda v^2 at tree level.

These relations are "structure"; the numbers come from calibration (next sections).

---

## 4. What are the SM "Numbers" and How Many are There?

### 4.1 Parameterization (Minimal SM, Massless Neutrinos)

A widely used accounting is: **19 free parameters** in the SM if neutrinos are massless (plus an additional strong CP parameter often included separately). The core point is not the exact count, but that the Lagrangian reduces to a finite parameter vector once the field content is fixed.

### 4.2 Canonical Parameter Blocks

A practical parameter vector theta(mu) (in a chosen renormalization scheme at scale mu) is:

**Gauge sector**:
- g_s(mu), g(mu), g'(mu)

**Higgs sector**:
- v(mu) (or equivalently G_F as input)
- lambda(mu) (or equivalently m_H as input)

**Yukawa sector**:
- 9 charged fermion masses `<->` Yukawas `y_f(mu) = sqrt(2) m_f(mu) / v(mu)` in a scheme
- CKM matrix parameters (3 mixing angles + 1 CP phase)

**QCD theta term (strong CP)**:
- theta_QCD (constrained by EDM bounds; practically an Omega frontier, not a derived number)

---

## 5. The Complete Method to Determine the Numbers (Verifier-First Pipeline)

Every "SM number" is an output of the same pipeline:

### 5.1 Choose the Scheme and Input Set

You must declare:
- renormalization scheme (commonly MS-bar, on-shell for some masses)
- renormalization scale(s) mu
- an input parameter set, e.g. `{alpha, G_F, M_Z}` or `{M_W, M_Z, G_F}` etc.

### 5.2 Define the Total Verifier (What "PASS" Means)

Given a parameter vector theta and a dataset D, define a total verifier that returns:
- **PASS** if theta matches the data within the declared likelihood model and theory truncation
- **FAIL** with a minimal conflict witness (which observable fails, by how much, under what theory order)

The PDG electroweak review formalizes this as global fits and "pulls" for observables.

### 5.3 Fit = Collapse the Frontier

You start with a prior candidate family D_0 (parameter grid / sampling / bounded intervals). Each measurement is a separator test that shrinks survivors. The output is not a single floating number but:
- best-fit point
- covariance matrix
- certified intervals for each parameter in the declared scheme

That is the "real number" meaning operationally: an interval that narrows as separators accumulate.

---

## 6. Explicit "How to Compute" for the Key Parameter Blocks

### 6.1 v from G_F

In the SM, the Higgs VEV is fixed by Fermi's constant:

```
v ≈ (sqrt(2) G_F)^{-1/2}
```

(with radiative corrections depending on scheme). This is the first anchor for electroweak scale calibration.

### 6.2 g, g' from (M_W, M_Z, v) (Tree Level) + Loop Corrections

Tree level:
```
g = 2 M_W / v
g' = (2/v) sqrt(M_Z^2 - M_W^2)
```

Then apply scheme-dependent electroweak radiative corrections (PDG global fit machinery).

### 6.3 QCD Coupling g_s from alpha_s(mu)

Define alpha_s(mu) = g_s^2(mu) / (4 pi). The value at M_Z and its running are standardized by QCD renormalization group equations.

### 6.4 Yukawas from Masses

Given v and a renormalization scheme for fermion masses m_f(mu):
```
y_f(mu) = sqrt(2) m_f(mu) / v(mu)
```

This is exact algebra once scheme/scale for m_f is declared.

### 6.5 CKM Parameters from Flavor Observables

The CKM matrix is inferred by fitting a set of flavor measurements (decay rates, CP violation). The result is a best-fit plus covariance; again: an interval object, not "a single number."

---

## 7. The "Missing Numbers" and How to Treat Them Correctly

### 7.1 Strong CP theta_QCD

The SM allows a theta-term; its value is not derived and is constrained by experiment. In this discipline it is Omega unless the separator exists (EDM measurement down to the needed sensitivity). It must be reported as a frontier interval.

### 7.2 Neutrino Masses and Mixings

Minimal SM has massless neutrinos; observed neutrino oscillations require extension (dimension-5 operator or right-handed neutrinos), adding parameters. These parameters are also inferred as intervals from data.

So "SM is complete" means "SM as a parameterized structure is complete"; it does not mean all parameters are derived from pure symmetry.

---

## 8. What Was Actually Confusing Humans (and What This Removes)

1. **Confusing structure with parameters**: The Lagrangian form is forced by symmetry + locality + renormalizability; the numerical values are calibration outputs.

2. **Mixing schemes without declaring them**: Quark masses, couplings, and mixing angles depend on renormalization scheme/scale; if not declared, numbers appear inconsistent. The correct object is (theta, mu, scheme) with receipts.

3. **Treating point estimates as truth**: A parameter is a certified interval + covariance under a declared dataset and theory order; point values are derived summaries.

4. **Treating "naturalness" arguments as facts**: Any "fine tuning" claim is a statement about priors or model classes; it must be expressed as a witnessable separator or stays Omega.

---

## 9. Full Verification Bundle (What Must Ship)

To make the SM "completely verifiable," publish a proof bundle with:

### A) Model Fingerprint

- gauge group and representations
- full Lagrangian term list (canonical form)
- renormalization scheme definition
- RGE equations used (canonical version hashes)

```json
{
  "type": "SM_MODEL_FINGERPRINT",
  "gauge_group": "SU(3)_c x SU(2)_L x U(1)_Y",
  "generations": 3,
  "higgs_doublets": 1,
  "lagrangian_terms": ["gauge", "fermion", "higgs", "yukawa"],
  "scheme": "<MS-bar|on-shell|...>",
  "scale_gev": "<rational>",
  "model_hash": "<sha256>",
  "result": "PASS"
}
```

### B) Dataset Witness Bundle

- all input measurements used (values + uncertainties + correlations)
- provenance hashes (tables / PDFs / extracted canonical JSON)

```json
{
  "type": "SM_DATASET_BUNDLE",
  "measurements_count": "<integer>",
  "observables": ["M_Z", "M_W", "G_F", "alpha", "alpha_s", ...],
  "correlations_included": true,
  "provenance_hash": "<sha256>",
  "result": "PASS"
}
```

### C) Prediction Engine (Total Verifier)

- code that maps theta -> predicted observables (with declared perturbative order)
- code that builds a likelihood / chi^2 with correlations
- PASS/FAIL predicate + minimal conflict witness

```json
{
  "type": "SM_PREDICTION_ENGINE",
  "engine_id": "<string>",
  "perturbative_order": "<LO|NLO|NNLO|...>",
  "observables_predicted": "<integer>",
  "code_hash": "<sha256>",
  "deterministic": true,
  "result": "PASS"
}
```

### D) Fit Output Objects

- best fit theta_hat
- covariance matrix Sigma
- certified intervals [L_i, U_i] for each parameter (at chosen confidence)
- scheme/scale tags

```json
{
  "type": "SM_FIT_OUTPUT",
  "fit_id": "<string>",
  "parameters_count": "<integer>",
  "best_fit_hash": "<sha256>",
  "covariance_hash": "<sha256>",
  "confidence_level": "<rational>",
  "scheme": "<string>",
  "scale_gev": "<rational>",
  "result": "PASS"
}
```

### E) Cross-Verification Tests

- reproduce key PDG electroweak fit observables and pulls (within stated theory order)
- reproduce alpha_s running relations (within stated perturbative order)
- anomaly cancellation consistency checks for the chosen representations

```json
{
  "type": "SM_CROSS_VERIFICATION",
  "check_id": "<string>",
  "pdg_ew_reproduced": true,
  "alpha_s_running_reproduced": true,
  "anomaly_cancellation_verified": true,
  "pulls_within_tolerance": true,
  "result": "PASS"
}
```

### F) Receipts

Every artifact is canonical JSON (sorted keys, no whitespace) hashed (SHA-256), including:
- model hash
- dataset hash
- code hash
- fit output hash

```json
{
  "type": "SM_RECEIPT_BUNDLE",
  "bundle_id": "<string>",
  "model_hash": "<sha256>",
  "dataset_hash": "<sha256>",
  "code_hash": "<sha256>",
  "fit_hash": "<sha256>",
  "all_verified": true,
  "bundle_fingerprint": "<sha256>",
  "result": "PASS"
}
```

---

## 10. The "Exact Way to Find Whatever Numbers You Need"

This is the algorithm, end to end:

1. **Declare scheme + scale + input set** (e.g., `{alpha, G_F, M_Z}`).
2. **Load dataset witness bundle** (canonical).
3. **Run prediction engine**: theta -> O_hat(theta) for all observables.
4. **Run fit** (minimize chi^2 / maximize likelihood), producing theta_hat and Sigma.
5. **Emit certified intervals + covariance** as the "real number" objects.
6. **Verify by replay**: same dataset + same code + same scheme -> same receipts.
7. **Any parameter not collapsed by existing data remains Omega frontier** with the minimal missing separator (which measurement would decide it further).

This is complete closure: structure forced; numbers are verified outputs; uncertainties are explicit frontiers; nothing is guessed.

---

## Canonical Receipt Schemas

### SM_GAUGE_GROUP Receipt
```json
{
  "type": "SM_GAUGE_GROUP",
  "group_id": "<string>",
  "factors": ["SU(3)", "SU(2)", "U(1)"],
  "coupling_count": 3,
  "anomaly_free": true,
  "result": "PASS"
}
```

### SM_FIELD_CONTENT Receipt
```json
{
  "type": "SM_FIELD_CONTENT",
  "content_id": "<string>",
  "generations": 3,
  "quark_doublets": 3,
  "quark_singlets": 6,
  "lepton_doublets": 3,
  "lepton_singlets": 3,
  "higgs_doublets": 1,
  "total_fields": 16,
  "result": "PASS"
}
```

### SM_LAGRANGIAN Receipt
```json
{
  "type": "SM_LAGRANGIAN",
  "lagrangian_id": "<string>",
  "gauge_terms": true,
  "fermion_terms": true,
  "higgs_terms": true,
  "yukawa_terms": true,
  "dimension_bound": 4,
  "gauge_invariant": true,
  "lorentz_invariant": true,
  "result": "PASS"
}
```

### SM_PARAMETER Receipt
```json
{
  "type": "SM_PARAMETER",
  "parameter_id": "<string>",
  "symbol": "<string>",
  "value_lower": "<integer_ratio>",
  "value_upper": "<integer_ratio>",
  "uncertainty": "<integer_ratio>",
  "scheme": "<string>",
  "scale_gev": "<integer_ratio>",
  "source": "<measurement|derived>",
  "result": "PASS"
}
```

### SM_EWSB Receipt
```json
{
  "type": "SM_EWSB",
  "ewsb_id": "<string>",
  "vev_gev": "<integer_ratio>",
  "m_w_gev": "<integer_ratio>",
  "m_z_gev": "<integer_ratio>",
  "sin2_theta_w": "<integer_ratio>",
  "m_h_gev": "<integer_ratio>",
  "relations_satisfied": true,
  "result": "PASS"
}
```

---

## Closing Statement

The Standard Model is a fully specified structure: once you fix gauge symmetry, representations, and renormalizable locality, the Lagrangian form is fixed. The "numbers" are not mysteries or opinions: they are certified interval objects produced by declared measurement contracts and global fits, with total verifiers and receipts. Any remaining ambiguity is not hand-waved; it is an explicit frontier with the next missing separator.

**No mystery. No guessing. Only structure, calibration, and canonical receipts.**
