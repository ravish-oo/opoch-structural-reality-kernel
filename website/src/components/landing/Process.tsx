import React, { useMemo } from "react";

// Static data outside component
const stepsData = [
  { n: 1, title: "Apply", blurb: "Tell us who you are, the problem, and constraints." },
  { n: 2, title: "Deep dive (SF or remote)", blurb: "We map variables, invariants, and success conditions—in plain language." },
  { n: 3, title: "Delivery", blurb: "Action plan, timelines, and working artifacts. If blocked, we show the fastest way to unblock." },
];

const Process = React.memo(() => {
  const steps = useMemo(() => stepsData, []);

  return (
    <section id="process" className="border-t border-white/10">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <h2 className="text-3xl font-semibold">How it works</h2>
        <div className="mt-8 grid gap-6 md:grid-cols-3">
          {steps.map((s) => (
            <div key={s.n} className="rounded-2xl border border-white/10 bg-white/5 p-6">
              <div className="flex items-center gap-3"><div className="flex h-7 w-7 items-center justify-center rounded-full bg-white/10 text-sm">{s.n}</div><p className="font-medium">{s.title}</p></div>
              <p className="mt-2 text-sm text-white/70">{s.blurb}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
});

Process.displayName = "Process";

export default Process;