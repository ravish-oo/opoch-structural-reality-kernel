---
sidebar_position: 4
title: "ORC: Opoch Reality Compiler"
description: A Witnessability Kernel for Derivational Intelligence — compiler architecture that outputs decision objects rather than plausible text
---

# ORC: Opoch Reality Compiler

**A Witnessability Kernel for Derivational Intelligence**

---

## Abstract

We introduce ORC, a compiler architecture that outputs decision objects rather than plausible text. ORC is built from a single admissibility principle, **witnessability**: a distinction is admissible only if a finite witness and a halting verifier can separate it.

From this constraint, we derive a forced kernel consisting of:
- Finite descriptions
- Self-delimiting syntax
- Endogenous feasible tests
- An order-free evidence ledger
- Truth as a quotient under recorded indistinguishability
- Gauge coequalization
- A deterministic separator recursion that selects the next admissible test without minting arbitrary distinctions

ORC exposes this kernel as a universal interface: for any compiled query, it returns either a **UNIQUE** theorem package (answer, witness, verifier, receipt) or an **Ω frontier** package (surviving family, minimal separator test, resource gap, receipt).

---

## 1. Introduction

Modern generative models optimize plausibility under a token distribution. This produces useful language, but it does not define a decision contract. In settings where correctness matters, a system must return a certified decision state, not merely an answer.

ORC formalizes a different interface:

- If a question is decidable under admissible tests and budget, return a unique answer with a witness and verifier.
- If not decidable, return an explicit frontier of surviving alternatives and the minimal separating test that would decide, or the exact resource gap.

This interface turns uncertainty into an executable object. It also makes the system robust to phrasing, because truth is defined by tests and receipts, not by surface form.

---

## 2. Preliminaries and Admissibility

### 2.1 Nothingness and Witnessability

We begin from strict nothingness:

$$\bot := \text{no admissible distinctions exist}$$

Admissibility is defined by witnessability:

:::info Axiom A0 (Witnessability)
A distinction is admissible iff there exists a finite witness $w \in D^*$ and a halting verifier $V(q,w)$ returning PASS or FAIL.
:::

Untestable distinctions are forbidden. Under this rule, "truth" must be an operational object.

### 2.2 Forced Carrier: Finite Descriptions

Witnessability forces finiteness:

$$D^* = \{0,1\}^{<\infty}$$

The set of all finite bitstrings.

$$D_0 \subseteq D^*$$

A finite working domain for any concrete run.

### 2.3 Self-Delimiting Syntax

In a closed system, parsing cannot rely on external boundaries. We use prefix-free codes:

$P \subseteq D^*$ is prefix-free iff no codeword is a prefix of another.

This prevents minting "end-of-program" distinctions.

---

## 3. Endogenous Tests and Feasibility

A closed system cannot import an external menu of tests. Tests must be generated internally by programs.

**Minimal executability substrate:**

| Symbol | Definition |
|--------|------------|
| $U: P \times D^* \to D^*$ | Total evaluator |
| $\text{decode}: D^* \to A$ | Total decoder to finite alphabet (incl FAIL/TIMEOUT) |
| $C: P \to \mathbb{N}$ | Cost function |
| $\text{Budget}: \mathbb{N} \to \mathbb{N}$ | Monotone nonincreasing capacity function |

Each program induces a test:

$$\tau_p(x) = \text{decode}(U(p,x))$$

Feasible tests at ledger-time $T$:

$$\Delta(T) = \{\tau_p : C(p) \le \text{Budget}(T)\}$$

Feasibility shrink is forced by monotone budget.

---

## 4. Ledger, Closure, and Truth Objects

### 4.1 Records and Order-Free Ledger

A record is a test outcome:

$$r = (\tau, a), \text{ where } a \in A_\tau$$

Ledger $\mathcal{L}$ is a multiset of records.

**Order is irrelevant unless explicitly recorded.**

### 4.2 Consistency Fiber

Given $\mathcal{L}$, define the surviving set:

$$W(\mathcal{L}) = \{x \in D_0 : \forall(\tau,a)\in\mathcal{L}, \tau(x)=a\}$$

### 4.3 Truth as Quotient Closure

Define ledger-induced indistinguishability:

$$x \equiv_{\mathcal{L}} y \iff \forall(\tau,a)\in\mathcal{L}, \tau(x)=\tau(y)$$

Truth object:

$$\Pi^*(\mathcal{L}) = D_0 / \equiv_{\mathcal{L}}$$

Truth is the partition into indistinguishability classes under recorded tests.

---

## 5. Gauge and Orthogonality

### 5.1 Gauge Coequalization

If renaming tests or recoding representations changes truth, the difference is untestable slack. Define $G_T$ as the groupoid of transformations invisible to $\Delta(T)$. Physical output is:

$$\text{PhysOut} = \text{RawOut} / G_T$$

### 5.2 Truth vs Control

To prevent arbitrary tie-breaking from reentering truth, impose orthogonality:

$$\Pi \circ N = \Pi \circ N \circ \Pi$$

Control may depend only on $\Pi$-fixed structure.

---

## 6. Ω Frontier and Separator Recursion

### 6.1 Ω Frontier

For a finite query $q: D_0 \to B$, define:

$$\text{Ans}_{\mathcal{L}}(q) = \{q(x): x\in W(\mathcal{L})\}$$

If $|\text{Ans}_{\mathcal{L}}(q)| = 1$, return **UNIQUE**. Otherwise return **Ω**.

### 6.2 Deterministic Separator Functional

For $\tau \in \Delta(T)$ with outcomes $a$, define branch fibers:

$$W_a = \{x \in W : \tau(x)=a\}$$

Define minimax value:

$$V(W,T;q) = 0 \text{ if } q \text{ constant on } W$$

$$V(W,T;q) = \min_{\tau \in \Delta(T)} \left[c(\tau) + \max_{a:W_a\neq \emptyset} V(W_a, T+c(\tau); q)\right] \text{ otherwise}$$

Canonical next separator:

$$\tau^*(W,T;q) = \arg\min_{\tau}[\cdot]$$

Tie-broken only by gauge-invariant fingerprints.

This forbids heuristic "next step" choices.

---

## 7. ORC Output Contract

ORC returns exactly one of:

### UNIQUE Package

| Field | Description |
|-------|-------------|
| Answer | $b$ |
| Witness | $w$ |
| Verifier | $V$ |
| Status | <span className="status-pass">PASS</span> |
| Receipt | $R$ |

### Ω Package

| Field | Description |
|-------|-------------|
| Surviving family | $\text{Ans}_{\mathcal{L}}(q)$ or equivalence-class summary |
| Separator | Minimal $\tau^*$ or exact resource gap |
| Receipt | $R$ |

Receipts are canonical hashes over normalized JSON payloads excluding the hash itself.

---

## 8. ORC System Architecture

ORC separates **proposing** from **committing**.

### 8.1 Proposer

Compiles unstructured input into a finite problem object:

$$P = (A, W, V, c, B)$$

The proposer may be statistical. Its output is not trusted as truth. It is treated as a compilation candidate.

### 8.2 Commit Gate

A deterministic commit gate enforces:

- Binding discipline
- Evidence addressing (span IDs, table-cell IDs, artifact hashes)
- Recomputation of arithmetic and transformations
- Contradiction witnesses when failing

**Only committed objects enter the ledger.**

### 8.3 Ledger and Closure Engine

Maintains $\mathcal{L}$, computes $W(\mathcal{L})$, $\Pi^*(\mathcal{L})$, Ω objects, and $\tau^*$.

---

## 9. Structural Reality Substrate

ORC becomes a system once it can persist structure across time.

### 9.1 Ledger-First Substrate

Store:
- Artifacts with provenance
- Claims with verifier contracts
- Evidence objects grounded to artifacts
- Records and receipts
- Π-classes for identity merges
- Ω objects for unresolved boundaries

Graph indices accelerate retrieval, but truth is ledger-induced.

### 9.2 Mainland vs Tissue

Claims can exist as candidates. Promotion to "mainland" requires passing separator tests with independent receipts. Internally consistent but unsupported clusters remain as Ω or low-support tissue.

---

## 10. Memory and Representations

### 10.1 Trit Representations

Use trit vectors $\{-1,0,+1\}^m$ for activation and routing, not for truth. Retrieval uses normalized alignment, occupancy control, and optional rotations to mitigate anisotropy.

### 10.2 Energy-Based Causal Activation

Memory is an energy field over the structural substrate:

- **Resonance injection** activates relevant nodes
- **Conductivity** spreads activation along causal edges
- **Cooling** decays unused structure
- **Active-set constraint** bounds working context

This yields relevance-driven persistence without equating memory to a fixed context window.

---

## 11. Execution Layer and Energy Computing

Ω packages include $\tau^*$, which is an actionable task. Define a work unit as:

- Inputs, procedure, outputs
- Verifier and receipt format
- Cost model

Execution can be routed to compute, agents, or laboratories. Verified results append to the ledger, shrinking Ω frontiers and caching truth for future verification.

---

## 12. Evaluation

Evaluate decision integrity, not fluency:

| Metric | Description |
|--------|-------------|
| Replay rate | UNIQUE receipts that verify on replay |
| Frontier reduction | Ω shrink per unit cost of $\tau^*$ |
| Paraphrase stability | Same decision under rephrasing |
| Merge precision | Identity accuracy under Π certification |
| Provenance completeness | Evidence chain coverage |

---

## 13. Limitations

ORC can decide only what admissible tests can decide under budget. Frontier domains will often return Ω until new tests or measurements enter the ledger.

:::tip
This is not a weakness; it is the correct behavior under witnessability.
:::

---

## 14. Conclusion

ORC defines **derivational intelligence** as a forced decision contract derived from witnessability. It replaces probabilistic guessing with closure under tests and explicit frontiers.

The system:
- Compiles questions into verifiable objects
- Commits only what can be replayed
- Turns unresolved uncertainty into minimal executable tests

This architecture supports a persistent structural reality substrate and a path to a theorem generator for all witnessable questions, with an execution economy that prices the work of collapsing ambiguity into certainty.

---

**Related:** [Gauge-Invariant Truth Machine (GITM)](/technology/overview) — The operational implementation

**Related:** [Energy-Based Causal Memory](/technology/memory) — The memory substrate referenced in Section 10
