// Falsification data and content

export interface FalsifyStats {
  attempts: number;
  successful: number;
  mostRecent: string;
}

export const falsifyStats: FalsifyStats = {
  attempts: 20,
  successful: 0,
  mostRecent: 'Jan 11, 2026'
};

export interface FalsifySection {
  id: string;
  title: string;
  content: string;
}

export const falsifySections: FalsifySection[] = [
  {
    id: 'alternative-closure',
    title: '1. Show a Second Closure Exists Under the Same Contract',
    content: `You win if you can build an alternative "universe semantics" that:

• Uses the same ⊥ (no admissible distinctions) and A0 (witnessability)
• Has no external distinctions
• Supports testable facts

But does NOT collapse to our ledger + Π-quotient + gauge structure (even up to equivalence).

If you can do that, our "forced" claim is false.

**This is the most direct falsification:** "Here is another consistent world from the same starting point."`
  },
  {
    id: 'smuggled-assumptions',
    title: '2. Show We Smuggled in Extra Assumptions',
    content: `You win if you identify a specific step where we introduced structure that is NOT implied by ⊥ + A0. For example:

• Assuming prefix-free programs are forced (instead of merely internal decodability)
• Assuming a total universal interpreter is forced (instead of a chosen execution substrate)
• Assuming a specific cost model or budget law is forced
• Assuming a realized-history selection rule is forced or hidden

Then you must show that removing the smuggled assumption changes the conclusions.

**This falsifies "no hidden assumptions."**`
  },
  {
    id: 'internal-contradiction',
    title: '3. Show an Internal Contradiction',
    content: `You win if you can show our system asserts two different "physical" outcomes while also claiming no admissible test can ever distinguish them.

That would mean we minted a distinction accidentally. That is a logical failure inside the system.

**This is a direct violation of our own rules.**`
  }
];

export const whatDoesntCount = [
  '"I reject witnessability" — That rejects the contract, it does not refute consequences under the contract.',
  '"This doesn\'t derive the Standard Model" — Not a refutation. Our claim is about forced minimal structure and explicit remaining primitives.',
  '"This is philosophy" — Not a counterexample.',
  'Numerical precision errors or implementation bugs',
  'Open-system dissipation without accounting for hidden channels'
];
