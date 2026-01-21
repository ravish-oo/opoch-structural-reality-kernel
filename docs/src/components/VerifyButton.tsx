import React, { useState, useRef, useEffect } from 'react';

interface VerifyButtonProps {
  pageUrl?: string;
  proofText?: string;
}

const DEFAULT_PROOF = `Please verify this mathematical proof:

KERNEL PROOF FROM NOTHINGNESS

Claim: Nothingness is the only unfalsifiable truth. Everything else (Π/Δ/T) is forced as the unique operational closure of describability.

PROOF STRUCTURE:
1. Starting Point: Only finite descriptions exist (D). No free structure assumed.
2. Δ is Forced: Meaning requires tests. If no test can separate x from y, "x differs from y" is meaningless.
3. Π is Forced: Truth = Δ-quotient. Any truth notion that distinguishes untestable differences depends on "minted" (meaningless) differences.
4. T is Forced: Time = irreversible ledger. Certainty requires irreversible commitment. T must be additive and gauge-invariant.
5. D* is Forced: Observer is inside universe. Any observer must be a finite subdescription.
6. Uniqueness: Any operational universe with (i) finite describability, (ii) meaning-by-test, (iii) irreversible records induces (Δ,Π,T) uniquely up to isomorphism.

Is this proof valid? Can you find any logical flaw or alternative that doesn't violate one of the three conditions?`;

export default function VerifyButton({ pageUrl, proofText = DEFAULT_PROOF }: VerifyButtonProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [copied, setCopied] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const currentUrl = pageUrl || (typeof window !== 'undefined' ? window.location.href : '');

  const chatGptUrl = `https://chatgpt.com/?hints=search&prompt=${encodeURIComponent(proofText)}`;
  const claudeUrl = `https://claude.ai/new?q=${encodeURIComponent(`Read from ${currentUrl} and verify the mathematical proof on this page. Can you find any logical flaw?`)}`;

  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const copyPage = async () => {
    try {
      const content = document.querySelector('.markdown')?.textContent || proofText;
      await navigator.clipboard.writeText(content);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy:', err);
    }
    setIsOpen(false);
  };

  return (
    <div className="verify-dropdown" ref={dropdownRef}>
      <div className="verify-dropdown-container">
        <a
          href={chatGptUrl}
          target="_blank"
          rel="noopener noreferrer"
          className="verify-main-btn"
        >
          Verify with ChatGPT
        </a>
        <button
          className="verify-toggle-btn"
          onClick={() => setIsOpen(!isOpen)}
          aria-label="More options"
        >
          <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor">
            <path d="M2.5 4.5L6 8L9.5 4.5" stroke="currentColor" strokeWidth="1.5" fill="none" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      </div>

      {isOpen && (
        <div className="verify-dropdown-menu">
          <button onClick={copyPage} className="verify-dropdown-item">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <rect x="9" y="9" width="13" height="13" rx="2" ry="2"/>
              <path d="M5 15H4a2 2 0 01-2-2V4a2 2 0 012-2h9a2 2 0 012 2v1"/>
            </svg>
            <span>{copied ? 'Copied!' : 'Copy page'}</span>
          </button>

          <a
            href={claudeUrl}
            target="_blank"
            rel="noopener noreferrer"
            className="verify-dropdown-item"
            onClick={() => setIsOpen(false)}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
              <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/>
            </svg>
            <span>Open in Claude</span>
          </a>
        </div>
      )}
    </div>
  );
}
