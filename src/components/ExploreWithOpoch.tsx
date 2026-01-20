import { useEffect, useState, useCallback, memo } from "react";
import { Button } from "./ui/button";
import { User } from "@supabase/supabase-js";
import { Sparkles, Loader2 } from "lucide-react";

const PROMPTS = [
  "Design a room-temperature superconductor under XYZ limits…",
  "Double port throughput without new berths…",
  "Stabilize fusion plasmas with power cap ≤ …",
  "Route policy levers to reduce PM2.5 by 40%…",
  "Find dark matter candidates that satisfy observations…",
  "Design carbon capture at scale with energy efficiency > …",
];

interface ExploreWithOpochProps {
  user: User | null;
  onQuerySubmit?: () => void;
  onError?: (error: any) => void;
  onSignInClick: () => void;
}

export const ExploreWithOpoch = memo(function ExploreWithOpoch({ 
  user, 
  onQuerySubmit, 
  onError,
  onSignInClick 
}: ExploreWithOpochProps) {
  const [value, setValue] = useState<string>("");
  const [hint, setHint] = useState(PROMPTS[0]);
  const [submitting, setSubmitting] = useState(false);
  const [submitted, setSubmitted] = useState(false);

  // Rotate hint until typing starts
  useEffect(() => {
    if (value) return;
    let i = 0;
    const id = setInterval(() => { 
      i = (i + 1) % PROMPTS.length; 
      setHint(PROMPTS[i]); 
    }, 2800);
    return () => clearInterval(id);
  }, [value]);

  // Persist typed draft (sunk-cost fallacy)
  useEffect(() => {
    const key = "opoch.ask.draft";
    const savedDraft = localStorage.getItem(key);
    if (savedDraft && !value) {
      setValue(savedDraft);
    }
  }, []);

  useEffect(() => {
    const key = "opoch.ask.draft";
    if (value) {
      localStorage.setItem(key, value);
    } else {
      localStorage.removeItem(key);
    }
  }, [value]);

  const onSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault();
    if (!user || !value.trim() || submitting) return;
    
    setSubmitting(true);
    try {
      // Use server endpoint to bypass client-side issues
      const response = await fetch('/api/submit-query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: value,
          context: { source: "explore-with-opoch" },
          userId: user?.id,
          userEmail: user?.email,
          userName: user?.user_metadata?.full_name || user?.user_metadata?.name || user?.email?.split('@')[0]
        })
      });
      
      const result = await response.json();
      
      if (!response.ok || !result.success) {
        console.error("Server error:", result);
        onError?.(new Error(result.error || 'Failed to submit query'));
      } else {
        setValue("");
        localStorage.removeItem("opoch.ask.draft");
        setSubmitted(true);
        setTimeout(() => setSubmitted(false), 3000);
        onQuerySubmit?.();
      }
    } catch (err) {
      console.error("Failed to submit query:", err);
      onError?.(err);
    } finally {
      setSubmitting(false);
    }
  }, [user, value, submitting, onQuerySubmit, onError]);

  return (
    <div className="space-y-4">
      <h3 className="text-lg font-semibold">Explore with Opoch</h3>
      <p className="text-sm text-white/70">
        Bring your hardest technical problems. We'll analyze constraints and show you what's possible.
      </p>

      <form onSubmit={onSubmit} className="space-y-3">
        {/* Input field - always enabled for typing */}
        <input
          value={value}
          onChange={(e) => setValue(e.target.value)}
          placeholder={value ? "" : hint}
          className="w-full h-12 px-4 rounded-xl bg-white/5 text-white placeholder:text-white/30 border border-white/10 focus:border-white/20 focus:bg-white/10 transition-colors"
          aria-label="Ask a research question"
        />

        {/* Action buttons - below the input */}
        {!user ? (
          <div className="space-y-2">
            <div className="flex flex-col sm:flex-row gap-2">
              <Button
                type="button"
                className="w-full sm:w-auto h-11 rounded-xl bg-white text-black hover:bg-white/90"
                onClick={onSignInClick}
              >
                <Sparkles className="h-4 w-4 mr-2" />
                Sign in with Google to submit
              </Button>
              <span className="text-xs text-white/50 self-center">
                You can type first; we'll save your draft
              </span>
            </div>
          </div>
        ) : (
          <div className="flex flex-col sm:flex-row items-stretch sm:items-center justify-between gap-3">
            <Button 
              type="submit"
              disabled={submitting || !value.trim()}
              className="w-full sm:w-auto h-11 rounded-xl bg-white text-black hover:bg-white/90 disabled:opacity-50"
            >
              {submitting ? (
                <>
                  <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                  Submitting…
                </>
              ) : submitted ? (
                "✓ Submitted"
              ) : (
                "Submit"
              )}
            </Button>
            <span className="text-xs text-white/50 text-center sm:text-left">
              We'll follow up by email
            </span>
          </div>
        )}
      </form>
    </div>
  );
});