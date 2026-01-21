import React from "react";
import { motion } from "framer-motion";
import { ArrowRight } from "lucide-react";
import { Button } from "../ui/button";

interface HeroProps {
  openApply: () => void;
}

const Hero = React.memo(({ openApply }: HeroProps) => {
  return (
    <section className="relative overflow-hidden">
      <div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]" />
      <div className="mx-auto max-w-7xl px-4 py-20">
        <motion.h1 initial={{ opacity: 0, y: 10 }} animate={{ opacity: 1, y: 0 }} transition={{ duration: 0.6 }} className="max-w-4xl text-balance text-5xl font-semibold leading-tight tracking-tight md:text-6xl">
          Get unstuck. Build faster. <span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">Decide with confidence</span>.
        </motion.h1>
        <motion.p initial={{ opacity: 0, y: 10 }} animate={{ opacity: 1, y: 0 }} transition={{ delay: 0.1, duration: 0.6 }} className="mt-5 max-w-2xl text-lg text-white/80">
          For deep‑tech founders, PI‑level researchers, and staff+ engineers who can't afford wrong answers. Bring your hardest problem—we'll give you a plan you can act on.
        </motion.p>
        <div className="mt-8 flex flex-wrap items-center gap-4">
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}><span className="flex items-center gap-2">Apply now <ArrowRight className="h-4 w-4" /></span></Button>
          <Button variant="outline" className="rounded-2xl border-white/20 bg-white/5 text-white hover:bg-white/10" asChild><a href="#process" className="flex items-center gap-2">See how it works</a></Button>
        </div>
      </div>
    </section>
  );
});

Hero.displayName = "Hero";

export default Hero;