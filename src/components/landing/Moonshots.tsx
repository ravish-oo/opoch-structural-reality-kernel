import React, { useMemo } from "react";
import { Link } from "react-router-dom";
import { motion } from "framer-motion";
import { ArrowRight } from "lucide-react";
import { Button } from "../ui/button";
import ReceiptBadge from "./ReceiptBadge";

// Static data outside component
const moonshotsData = [
  { title: "Alpha Fixed Point", tagline: "metrology under control", emoji: "🧭" },
  { title: "H(z) + Cosmic Time", tagline: "universe clock", emoji: "⏳" },
  { title: "Rotation/Lensing", tagline: "dark matter residuals", emoji: "🕳️" },
  { title: "QEC Thresholds", tagline: "logical error bounds", emoji: "🧪" },
  { title: "Fold/Bind Explorer", tagline: "biophysics short path", emoji: "🧬" },
  { title: "Fusion Control", tagline: "minimum paid work", emoji: "⚡" },
  { title: "Solid‑State Battery", tagline: "capacity proofs", emoji: "🔋" },
  { title: "Grid Phasor Flow", tagline: "routing receipts", emoji: "🌐" },
  { title: "Robotics Control", tagline: "Z₄ rhythm", emoji: "🤖" },
  { title: "Climate MRV", tagline: "measure what matters", emoji: "🌿" },
  { title: "Port Logistics", tagline: "throughput lift", emoji: "🚢" },
  { title: "Deep Space Traj", tagline: "Δv budgets", emoji: "🛰️" },
  { title: "Safety Refuters", tagline: "mint/commit changes", emoji: "🛡️" },
  { title: "Phase Maps (CKL)", tagline: "holonomy collapse", emoji: "🧊" },
  { title: "Causal Route‑Finding", tagline: "policy proofs", emoji: "🧠" },
];

const Moonshots = React.memo(() => {
  const moonshots = useMemo(() => moonshotsData, []);

  return (
    <section id="moonshots" className="border-t border-white/10 px-4 py-24 sm:px-6 lg:px-8">
      <div className="mx-auto max-w-7xl">
        <motion.div
          initial={{ opacity: 0 }}
          whileInView={{ opacity: 1 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="flex items-end justify-between mb-8"
        >
          <h2 className="text-3xl font-bold md:text-5xl">
            <span className="bg-gradient-to-r from-white to-white/60 bg-clip-text text-transparent">
              15 Moonshots
            </span>
          </h2>
          <Link 
            to="/moonshots" 
            className="text-sm text-white/70 hover:text-white transition-colors"
          >
            See all →
          </Link>
        </motion.div>
        
        <div className="grid gap-4 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4">
          {moonshots.map((moonshot, index) => (
            <motion.div
              key={moonshot.title}
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.4, delay: index * 0.02 }}
              className="group rounded-2xl border border-white/10 bg-white/5 p-5 hover:border-white/20 hover:bg-white/10 transition-all"
            >
              <div className="flex items-center justify-between">
                <span className="text-2xl">{moonshot.emoji}</span>
                <ReceiptBadge state="PASS" />
              </div>
              <h3 className="mt-3 text-lg font-semibold tracking-tight">{moonshot.title}</h3>
              <p className="text-sm text-white/70">{moonshot.tagline}</p>
              <div className="mt-4">
                <Button variant="ghost" className="text-white/80 hover:text-white p-0" asChild>
                  <Link to={`/moonshots/${encodeURIComponent(moonshot.title.toLowerCase().replace(/\s+/g, "-"))}`}>
                    Open case <ArrowRight className="ml-1 inline h-4 w-4" />
                  </Link>
                </Button>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
});

Moonshots.displayName = "Moonshots";

export default Moonshots;