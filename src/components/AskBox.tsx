import { useEffect, useState, useCallback, memo } from "react";
import { supabase, isSupabaseConfigured } from "../lib/supabase";
import { Button } from "./ui/button";
import type { Query, ApiError } from "../types";

const prompts = [
  "Design a room-temperature superconductor with constraints…",
  "Stabilize fusion plasmas under X,Y,Z limits…",
  "Optimize a port to 2× throughput with capex ≤ …",
  "Prove P vs NP with bounded computational resources…",
  "Find dark matter candidates that satisfy observations…",
  "Design carbon capture at scale with energy efficiency > …",
];

interface AskBoxProps {
  className?: string;
  onQuerySubmit?: (query: Partial<Query>) => void;
  onError?: (error: ApiError) => void;
}

export const AskBox = memo(function AskBox({ className, onQuerySubmit, onError }: AskBoxProps) {
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

  const onSubmit = useCallback(async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    if (!value.trim() || isSubmitting) return;
    
    setIsSubmitting(true);
    try {
      // Get current user
      const { data: { user } } = await supabase.auth.getUser();
      
      const queryData: Partial<Query> = {
        text: value,
        context: { source: "rbt-askbox" },
        user_id: user?.id // Add user_id if logged in
      };
      
      if (isSupabaseConfigured()) {
        const { data, error } = await supabase
          .from("queries")
          .insert(queryData)
          .select('id, created_at')
          .single();
        
        if (error) {
          const apiError: ApiError = {
            message: error.message,
            code: error.code,
            details: error.details
          };
          onError?.(apiError);
          return;
        }
        
        if (data) {
          const completeQuery = { ...queryData, id: data.id, created_at: data.created_at };
          onQuerySubmit?.(completeQuery);
          setValue("");
          setSubmitted(true);
          setTimeout(() => setSubmitted(false), 3000);
        }
      } else {
        // Demo mode
        console.log("Query (demo mode):", value);
        onQuerySubmit?.(queryData);
        setValue("");
        setSubmitted(true);
        setTimeout(() => setSubmitted(false), 3000);
      }
    } catch (err) {
      const error = err as Error;
      const apiError: ApiError = {
        message: error.message || "Failed to submit query",
        code: "QUERY_SUBMISSION_ERROR"
      };
      onError?.(apiError);
      console.error("Failed to submit query:", err);
    } finally {
      setIsSubmitting(false);
    }
  }, [value, isSubmitting, onQuerySubmit, onError]);

  return (
    <div className={`mt-4 ${className || ''}`}>
      <form onSubmit={onSubmit} className="flex gap-2" role="search">
        <input
          type="text"
          value={value}
          onChange={(e) => setValue(e.target.value)}
          placeholder={value ? "" : hint}
          className="flex-1 rounded-xl bg-white/5 px-4 py-2 text-white placeholder:text-white/50 border border-white/10 focus:border-white/20 focus:outline-none focus:ring-1 focus:ring-white/20"
          disabled={isSubmitting}
          aria-label="Ask a research question"
          aria-describedby={submitted ? "query-success" : undefined}
        />
        <Button 
          type="submit"
          disabled={isSubmitting || !value.trim()}
          className="rounded-xl bg-white text-black hover:bg-white/90 disabled:opacity-50"
          aria-label="Submit question"
        >
          {isSubmitting ? "Sending..." : "Ask"}
        </Button>
      </form>
      {submitted && (
        <p id="query-success" role="status" aria-live="polite" className="mt-2 text-sm text-emerald-400">
          Query received! We'll process this soon.
        </p>
      )}
    </div>
  );
});