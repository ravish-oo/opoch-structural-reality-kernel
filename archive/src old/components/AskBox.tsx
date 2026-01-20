import { useEffect, useState } from "react";
import { supabase, isSupabaseConfigured } from "../lib/supabase";
import { Button } from "./ui/button";

const prompts = [
  "Design a room-temperature superconductor with constraints…",
  "Stabilize fusion plasmas under X,Y,Z limits…",
  "Optimize a port to 2× throughput with capex ≤ …",
  "Prove P vs NP with bounded computational resources…",
  "Find dark matter candidates that satisfy observations…",
  "Design carbon capture at scale with energy efficiency > …",
];

export function AskBox() {
  const [value, setValue] = useState("");
  const [hint, setHint] = useState(prompts[0]);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [submitted, setSubmitted] = useState(false);

  useEffect(() => {
    if (value) return;
    let i = 0;
    const id = setInterval(() => {
      i = (i + 1) % prompts.length;
      setHint(prompts[i]);
    }, 3000);
    return () => clearInterval(id);
  }, [value]);

  async function onSubmit(e: React.FormEvent) {
    e.preventDefault();
    if (!value.trim() || isSubmitting) return;
    
    setIsSubmitting(true);
    try {
      if (isSupabaseConfigured()) {
        const { error } = await supabase.from("queries").insert({ 
          text: value,
          context: { source: "rbt-askbox" }
        });
        
        if (!error) {
          setValue("");
          setSubmitted(true);
          setTimeout(() => setSubmitted(false), 3000);
        }
      } else {
        // Just show success even if not connected to database
        console.log("Query (not saved to DB):", value);
        setValue("");
        setSubmitted(true);
        setTimeout(() => setSubmitted(false), 3000);
      }
    } catch (err) {
      console.error("Failed to submit query:", err);
    } finally {
      setIsSubmitting(false);
    }
  }

  return (
    <div className="mt-4">
      <form onSubmit={onSubmit} className="flex gap-2">
        <input
          type="text"
          value={value}
          onChange={(e) => setValue(e.target.value)}
          placeholder={value ? "" : hint}
          className="flex-1 rounded-xl bg-white/5 px-4 py-2 text-white placeholder:text-white/50 border border-white/10 focus:border-white/20 focus:outline-none focus:ring-1 focus:ring-white/20"
          disabled={isSubmitting}
        />
        <Button 
          type="submit"
          disabled={isSubmitting || !value.trim()}
          className="rounded-xl bg-white text-black hover:bg-white/90 disabled:opacity-50"
        >
          {isSubmitting ? "Sending..." : "Ask"}
        </Button>
      </form>
      {submitted && (
        <p className="mt-2 text-sm text-emerald-400">Query received! We'll process this soon.</p>
      )}
    </div>
  );
}