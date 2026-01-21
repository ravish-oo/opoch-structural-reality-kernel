import React, { useMemo } from "react";
import { LineChart, Target, ShieldCheck } from "lucide-react";

// Static data moved outside component
const outcomeData = [
  { icon: LineChart, title: "Decisions you can defend", blurb: "Clear yes/no, trade‑offs, and next steps you can stand behind." },
  { icon: Target, title: "Faster problem unblocking", blurb: "We diagnose in hours, plot the path in days, and get you moving." },
  { icon: ShieldCheck, title: "Reduced risk", blurb: "No vague promises. We show what's possible now and what it costs to get the rest." },
];

const Outcomes = React.memo(() => {
  const outcomes = useMemo(() => outcomeData, []);

  return (
    <section className="border-t border-white/10 bg-white/5" id="outcomes">
      <div className="mx-auto max-w-7xl px-4 py-14">
        <div className="grid gap-6 md:grid-cols-3">
          {outcomes.map((o, i) => (
            <div key={i} className="rounded-2xl border border-white/10 bg-black p-6">
              <div className="flex items-center gap-3"><o.icon className="h-5 w-5 text-opoch-cyan-light" /><p className="font-medium">{o.title}</p></div>
              <p className="mt-2 text-sm text-white/70">{o.blurb}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
});

Outcomes.displayName = "Outcomes";

export default Outcomes;