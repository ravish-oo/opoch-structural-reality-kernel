import React, { useMemo } from "react";
import { Rocket, FlaskConical, Network } from "lucide-react";

// Static data outside component
const whoData = [
  { icon: Rocket, title: "Deep‑tech founders", blurb: "You need the short path to truth, not another deck." },
  { icon: FlaskConical, title: "PI‑level researchers", blurb: "Tighter experiments. Fewer dead ends. Better funding shots." },
  { icon: Network, title: "Staff+ engineers", blurb: "Hard systems decisions with high confidence and a clear plan." },
];

const Who = React.memo(() => {
  const who = useMemo(() => whoData, []);

  return (
    <section id="who" className="border-t border-white/10 bg-white/5">
      <div className="mx-auto max-w-7xl px-4 py-16">
        <h2 className="text-3xl font-semibold">Who it's for</h2>
        <div className="mt-8 grid gap-6 md:grid-cols-3">
          {who.map((w, i) => (
            <div key={i} className="rounded-2xl border border-white/10 bg-black p-6">
              <div className="flex items-center gap-3"><w.icon className="h-5 w-5 text-opoch-cyan-light" /><p className="font-medium">{w.title}</p></div>
              <p className="mt-2 text-sm text-white/70">{w.blurb}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
});

Who.displayName = "Who";

export default Who;