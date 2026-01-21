import { useEffect, useState, memo } from "react";
import { Button } from "./ui/button";
import { Sparkles } from "lucide-react";

const prompts = [
  "Design a room-temperature superconductor with constraints…",
  "Stabilize fusion plasmas under X,Y,Z limits…",
  "Optimize a port to 2× throughput with capex ≤ …",
  "Prove P vs NP with bounded computational resources…",
  "Find dark matter candidates that satisfy observations…",
  "Design carbon capture at scale with energy efficiency > …",
];

interface AskBoxTeaserProps {
  className?: string;
  onSignInClick: () => void;
}

export const AskBoxTeaser = memo(function AskBoxTeaser({ className, onSignInClick }: AskBoxTeaserProps) {
  const [hint, setHint] = useState(prompts[0]);
  const [fadeClass, setFadeClass] = useState("opacity-100");

  useEffect(() => {
    let i = 0;
    const intervalId = setInterval(() => {
      setFadeClass("opacity-0");
      
      setTimeout(() => {
        i = (i + 1) % prompts.length;
        setHint(prompts[i]);
        setFadeClass("opacity-100");
      }, 300);
    }, 3000);

    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className={`mt-4 ${className || ''}`}>
      <div className="relative">
        <input
          type="text"
          placeholder=" "
          className="w-full rounded-xl bg-white/5 px-4 py-2 text-white placeholder:text-white/50 border border-white/10 cursor-not-allowed opacity-70"
          disabled
          aria-label="Sign in to ask a research question"
        />
        <div className={`absolute inset-0 flex items-center px-4 pointer-events-none transition-opacity duration-300 ${fadeClass}`}>
          <span className="text-white/50">{hint}</span>
        </div>
        <Button 
          onClick={onSignInClick}
          className="absolute right-1 top-1 bottom-1 rounded-xl bg-white text-black hover:bg-white/90 px-4"
          aria-label="Sign in to ask questions"
        >
          <Sparkles className="h-4 w-4 mr-2" />
          Sign in to ask
        </Button>
      </div>
      <p className="mt-3 text-xs text-white/50 text-center">
        Join with Google or GitHub to submit your technical challenges
      </p>
    </div>
  );
});