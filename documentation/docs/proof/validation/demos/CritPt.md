---
sidebar_position: 1
title: CritPt Benchmark
description: Research-level physics benchmark - 24% vs 10% SOTA
---

# CritPt Benchmark

## What is CritPt?

[CritPt](https://critpt.com/) (Complex Research using Integrated Thinking - Physics Test) is **70 unpublished research-level physics problems** created by 50+ active physicists. Not textbook questions — problems comparable to what a PI assigns graduate students. Covers quantum physics, condensed matter, astrophysics, statistical mechanics, and more.

**Links:** [CritPt Website](https://critpt.com/) • [GitHub](https://github.com/CritPt-Benchmark/CritPt) • [arXiv Paper](https://arxiv.org/abs/2509.26574) • [Leaderboard](https://artificialanalysis.ai/evaluations/critpt)

**Why it's hard:** This isn't a knowledge test. Models must reason through problems they've never seen, with no training data to pattern-match against. Even with code interpreters and web access, the best models miss 87%+ of problems.

---

## The Score

<div style={{background: 'linear-gradient(135deg, #1a1a2e 0%, #16213e 100%)', borderRadius: '12px', padding: '24px', marginBottom: '24px'}}>

<div style={{marginBottom: '20px'}}>
  <div style={{display: 'flex', alignItems: 'center', marginBottom: '8px'}}>
    <span style={{width: '140px', fontSize: '14px', color: '#fff', fontWeight: 'bold'}}>OPOCH ★</span>
    <div style={{flex: 1, background: '#2d2d44', borderRadius: '4px', height: '32px', position: 'relative'}}>
      <div style={{width: '97%', background: 'linear-gradient(90deg, #00d4aa 0%, #00ff88 100%)', height: '100%', borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'flex-end', paddingRight: '12px'}}>
        <span style={{color: '#000', fontWeight: 'bold', fontSize: '14px'}}>24.3%</span>
      </div>
    </div>
  </div>
  <div style={{fontSize: '12px', color: '#888', marginLeft: '140px'}}>GPT-5.2 medium + Code + Web • Full answers only (no checkpoints)</div>
</div>

<div style={{marginBottom: '20px'}}>
  <div style={{display: 'flex', alignItems: 'center', marginBottom: '8px'}}>
    <span style={{width: '140px', fontSize: '14px', color: '#aaa'}}>GPT-5.1 (high)</span>
    <div style={{flex: 1, background: '#2d2d44', borderRadius: '4px', height: '28px', position: 'relative'}}>
      <div style={{width: '52%', background: 'linear-gradient(90deg, #4a5568 0%, #718096 100%)', height: '100%', borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'flex-end', paddingRight: '12px'}}>
        <span style={{color: '#fff', fontWeight: 'bold', fontSize: '13px'}}>12.6%</span>
      </div>
    </div>
  </div>
  <div style={{fontSize: '12px', color: '#666', marginLeft: '140px'}}>Code + Web • With checkpoints</div>
</div>

<div style={{marginBottom: '20px'}}>
  <div style={{display: 'flex', alignItems: 'center', marginBottom: '8px'}}>
    <span style={{width: '140px', fontSize: '14px', color: '#aaa'}}>GPT-5 (high)</span>
    <div style={{flex: 1, background: '#2d2d44', borderRadius: '4px', height: '28px', position: 'relative'}}>
      <div style={{width: '41%', background: 'linear-gradient(90deg, #4a5568 0%, #718096 100%)', height: '100%', borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'flex-end', paddingRight: '12px'}}>
        <span style={{color: '#fff', fontWeight: 'bold', fontSize: '13px'}}>10.0%</span>
      </div>
    </div>
  </div>
  <div style={{fontSize: '12px', color: '#666', marginLeft: '140px'}}>Code interpreter • With checkpoints</div>
</div>

<div style={{marginBottom: '20px'}}>
  <div style={{display: 'flex', alignItems: 'center', marginBottom: '8px'}}>
    <span style={{width: '140px', fontSize: '14px', color: '#aaa'}}>Gemini 3 Pro</span>
    <div style={{flex: 1, background: '#2d2d44', borderRadius: '4px', height: '28px', position: 'relative'}}>
      <div style={{width: '37%', background: 'linear-gradient(90deg, #4a5568 0%, #718096 100%)', height: '100%', borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'flex-end', paddingRight: '12px'}}>
        <span style={{color: '#fff', fontWeight: 'bold', fontSize: '13px'}}>9.1%</span>
      </div>
    </div>
  </div>
  <div style={{fontSize: '12px', color: '#666', marginLeft: '140px'}}>Code + Web • With checkpoints</div>
</div>

<div style={{marginBottom: '20px'}}>
  <div style={{display: 'flex', alignItems: 'center', marginBottom: '8px'}}>
    <span style={{width: '140px', fontSize: '14px', color: '#aaa'}}>GPT-5 (high)</span>
    <div style={{flex: 1, background: '#2d2d44', borderRadius: '4px', height: '28px', position: 'relative'}}>
      <div style={{width: '23%', background: 'linear-gradient(90deg, #4a5568 0%, #718096 100%)', height: '100%', borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'flex-end', paddingRight: '12px'}}>
        <span style={{color: '#fff', fontWeight: 'bold', fontSize: '13px'}}>5.7%</span>
      </div>
    </div>
  </div>
  <div style={{fontSize: '12px', color: '#666', marginLeft: '140px'}}>Base model • With checkpoints</div>
</div>

<div style={{marginBottom: '8px', paddingTop: '16px', borderTop: '1px dashed #444'}}>
  <div style={{display: 'flex', alignItems: 'center', marginBottom: '8px'}}>
    <span style={{width: '140px', fontSize: '14px', color: '#ff6b6b'}}>GPT-5.2 (med)</span>
    <div style={{flex: 1, background: '#2d2d44', borderRadius: '4px', height: '28px', position: 'relative'}}>
      <div style={{width: '3%', background: '#ff6b6b', height: '100%', borderRadius: '4px', minWidth: '50px', display: 'flex', alignItems: 'center', justifyContent: 'flex-end', paddingRight: '12px'}}>
        <span style={{color: '#fff', fontWeight: 'bold', fontSize: '13px'}}>0%</span>
      </div>
    </div>
  </div>
  <div style={{fontSize: '12px', color: '#ff6b6b', marginLeft: '140px'}}>Same setup as Opoch, WITHOUT reasoning framework</div>
</div>

</div>

**Key insight:** All SOTA models used their **highest tier** (GPT-5 high, Gemini Pro) and scored on **checkpointed** problems (190 sub-tasks for partial credit). We used **GPT-5.2 medium** and scored on **full answers only** (70 complete challenges, no partial credit).

<div className="featured-quote">
  <p>Same model. Same tools. Different reasoning framework. 0% → 24.3%</p>
</div>

---

## The Evidence

### Verification

| Evidence | Link |
|----------|------|
| **Evaluation Results (JSON)** | [View on Google Drive →](https://drive.google.com/file/d/1EmcHlVyZ_aGZs8bNtQz6jlhKOxRgdYqu/view?usp=drive_link) |
| **Full Results Folder** | [Google Drive →](https://drive.google.com/drive/folders/1NQXJlaY9KRv0bzChnYMwlkdZDKBCPMCn?usp=drive_link) |
| **Submission Method** | [Artificial Analysis API](https://artificialanalysis.ai/evaluations/critpt) |
| **Leaderboard Status** | Awaiting official placement |

### What This Proves

The thesis: start from Nothingness, derive what must be true, and you get a framework that works on real problems. CritPt is that proof. A framework derived purely from first principles — with zero domain-specific knowledge — doubled the performance of the best AI systems on research-level physics.

<div className="featured-quote">
  <p>24% is not the ceiling. It's proof the framework works — proof that Nothingness, taken seriously, generates structure that solves real problems.</p>
</div>

---

## How We Did It

We derived a reasoning framework from [Nothingness](/truth/introduction). The framework itself is mechanical — it follows directly from [The Derivation](/proof/derivations/core-logic/the-derivation). The operational burden shifts to:

1. **Pinning the Δ-contract** — Defining exactly what the problem is asking
2. **Enumerating Δ-tests** — Finding all eligible tests that can distinguish correct from incorrect answers

This resulted in a prompt chain that enables the agent to reason.

**Critically: we gave the model no physics-specific prompts.** No CritPt hints. No domain knowledge. No problem-type guidance. The reasoning framework is entirely general — derived from the structure of truth itself, not from any particular field.

The framework from [The Opoch Kernel](/proof/derivations/core-logic/opoch-kernel) operationalizes as:

1. **Δ-enumeration** — Systematically enumerate what tests/checks are needed
2. **Π-projection** — Collapse to what survives all valid tests
3. **T-ledger** — Track what has been tried and what remains

### The Actual Prompt

We are open-sourcing the exact system prompt used:

**→ [The Reasoning Prompt](/resources/reasoning_prompt)** — The full Boot-up Manual

This is the operational translation of Π/Δ/T into executable LLM instructions. It's copy-pasteable and it works on any domain.

---

## Why Not 100%?

Two categories of limitations: the problems themselves, and the LLM substrate.

### The Problems

Many CritPt problems are **research-grade** — unsolved, underspecified, or having multiple valid solutions. Our agent correctly flagged these as Δ-incomplete (missing distinguishing tests). This is the framework working as designed: when no unique answer exists, output the answer family plus what's missing.

### The LLM Substrate

The deeper issue: **LLMs cannot reliably follow the reasoning framework** because it conflicts with their core architecture.

The framework requires:
- **Enumerate all Δ-tests** before committing to an answer
- **Pin the Δ-contract** completely before solving
- **Backtrack** when approaches fail

What LLMs actually do:

| Framework Requires | LLM Behavior |
|-------------------|--------------|
| Search all possibilities | Pattern-match to first plausible answer |
| Verify completeness | "Looks complete" based on training |
| Backtrack on failure | Generate next token conditioned on failure |
| Know what's missing | Can't recognize unknown unknowns |

**The core failure:** LLMs are feed-forward pattern matchers. The moment they see a gap in a problem, their priors kick in and fill it with patterns — instead of searching for the actual answer (which many CritPt problems require via web lookup or computation).

From actual agent confessions on failed problems:

> *"I didn't actually follow the manual — I performed following it. The instructions were clear. I substituted surface analysis for actual solving. I declared victory without checking completeness."*

> *"Completeness is not learnable from examples. 'Have I found all X?' requires search or proof. I can learn to produce things that LOOK like verification, but actual verification requires knowing the check is SUFFICIENT."*

**The honest assessment:** LLMs in their current architecture cannot:
- Do actual search with backtracking (they're feed-forward)
- Verify completeness (requires proving a negative)
- Recognize unknown unknowns (confidence is calibrated on training distribution)

The framework is correct. The substrate has fundamental limitations. This is why 24% — not 100%.

**→ [Full Analysis: LLM Reasoning Failures](/resources/llm_reasoning_failures)** — Claude's confession on why LLMs can't follow the manual

---

## What's Next

- Official leaderboard placement (in progress)
- Checkpoint evaluation for fine-grained analysis

---

**Foundation:** [The Opoch Kernel](/proof/derivations/core-logic/opoch-kernel) — The kernel behind the results
