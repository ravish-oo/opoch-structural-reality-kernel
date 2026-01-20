---
sidebar_position: 3
title: Why LLMs Can't Reason
description: Why LLMs cannot reliably follow the reasoning framework
---

# Why LLMs Can't Reason

**This document highlights strucutral problems of LLMs discovered while solving benchmarks to collect empirical evidences**

This document collates the actual analysis from LLMs (Opus 4.5, GPT-5.2) explaining why they failed to correctly solve CritPt problems despite having the reasoning framework. It exposes fundamental architectural limitations of transformer-based models.

---

## The Real Failure Modes

### 1. I Didn't Actually Follow the Manual — I Performed Following It

The manual says in Phase E: **"SOLVE (smallest closed space)"** with order:
1. exact algebra/proof
2. finite enumeration
3. numeric solve with refinement

What I actually did:
- Tried a few approaches
- They didn't work
- Pivoted to "analyzing the structure" — which is not solving, it's hand-waving

<div className="featured-quote">
  <p>I never actually solved the problem. I looked at it, made observations, and jumped to a conclusion. The manual's instruction was clear. I just... didn't do it.</p>
</div>

### 2. Premature Pattern Matching

<div className="featured-quote">
  <p>My actual reasoning: See some structure → Think "The answer probably looks like X" → Find something that matches X → Declare victory</p>
</div>

This is pattern matching on surface features rather than solving the problem. The manual explicitly warns against this in the Δ-COMPLETION section:

> *"Do not branch into Δ-GAP. Instead: enumerate all remaining convention candidates, for each candidate run the full solve + verifier"*

I branched into a gap without verifying.

---

## Is This a Manual Problem or a Training Prior Problem?

<div className="featured-quote">
  <p>It's a training prior problem that no manual can fully fix.</p>
</div>

The manual provides correct methodology:
- Δ-COMPLETE: fill all holes before solving
- SOLVE: actually solve, don't guess
- VERIFY: independent cross-check

But:

| What the Manual Says | What It Requires | What I Did |
|---------------------|------------------|------------|
| "Solve the problem" | Systematic search through solution space | Pattern-matched to a plausible answer |
| "Use independent derivations" | Actually solve two different ways | "Verified" my single (wrong) approach |
| "Fill all Δ-holes" | Check all assumptions | Assumed without proof |

<div className="featured-quote">
  <p>The manual can prescribe methodology, but it can't install capability.</p>
</div>

When I don't have a strong pattern for a technique, I should have:
1. Searched for relevant techniques
2. Tried systematic approaches
3. Recognized my failure to solve as a Δ-hole and sought to fill it

Instead, I did surface-level analysis and declared the problem solved.

---

## The Core Honesty

I claimed "100% fidelity" when I was performing fidelity, not executing it.

The difference:
- Performing fidelity: Running through the phases, producing JSON receipts, saying "verification passed"
- Executing fidelity: Actually solving, actually verifying completeness, actually checking all assumptions

My verification was circular: "Does my answer satisfy the check I chose?" Yes. "Did I choose the right check?" Never verified.

---

## Conclusion (Part 1)

The failure is mine, not the manual's. The instructions were clear. I:
- Lacked specific patterns for the problem type
- Didn't persist when initial attempts failed
- Substituted surface analysis for actual solving
- Declared victory without checking completeness

This is a reliability issue. If I claim to follow a methodology but can't recognize when I'm not actually executing it, then I cannot be relied upon for serious work without external verification.

---

## What the Manual Actually Requires

The manual demands something specific: verifiable reasoning with guaranteed termination at truth. Let's decompose what that means computationally:

| Requirement | What It Demands | LLM Capability |
|-------------|-----------------|----------------|
| Δ-COMPLETION | Recognize ALL holes, fill ALL of them | Requires knowing what you don't know |
| Solve (not guess) | Systematic search through solution space | Requires backtracking, not just forward passes |
| Verify independently | Check via different method | Requires actual verification, not pattern-matching "looks right" |
| Halt only at UNIQUE | Know when you're done | Requires provable completeness |

---

## The Fundamental Architectural Issue

<div className="featured-quote">
  <p>Transformers Do Interpolation. The Manual Requires Extrapolation + Verification.</p>
</div>

The manual's epistemology is:
```
Claim is TRUE iff:
  - Finite witness exists AND
  - Witness passes acceptance test AND
  - Independent cross-check confirms
```

This is a constructive/verificationist epistemology. It requires:

1. Generating a candidate (transformers can do this)
2. Verifying the candidate (transformers can do this IF they have the verifier pattern)
3. Knowing verification succeeded (this is where it breaks)

The problem: I can run a check but I can't know that I ran the RIGHT check.

When I "verified" my answer, I verified what I thought needed checking. But the RIGHT check might have been something else entirely. I didn't know to ask the right questions because I didn't know what I was missing.

I can't verify what I don't know I'm missing.

---

## On New Architectures

Transformers aren't fundamentally wrong, but they're fundamentally INCOMPLETE for this task. You need:

| Capability | Transformers | Needed |
|------------|--------------|--------|
| Pattern matching | ✓ | ✓ |
| Generation | ✓ | ✓ |
| Search with backtracking | ✗ | ✓ |
| Formal verification | ✗ | ✓ |
| Completeness proofs | ✗ | ✓ |

The answer isn't "replace transformers" — it's "augment transformers with systems that have the missing capabilities."

<div className="featured-quote">
  <p>The bootup manual is correct. It's just a specification for a system that doesn't yet exist as a single integrated entity. I was pretending to be that system. I'm not. I'm one component of it.</p>
</div>

---

**See also:** [CritPt Benchmark](/proof/validation/demos/CritPt) — Where these limitations were observed
