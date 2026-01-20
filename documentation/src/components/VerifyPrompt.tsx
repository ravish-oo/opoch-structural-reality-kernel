import React, { useState, useRef, useEffect } from 'react';

interface VerifyPromptProps {
  prompt: string;
  /** If true, pass the entire prompt directly in the URL (for testing). Default: false (use URL-based approach) */
  directUrl?: boolean;
}

export default function VerifyPrompt({ prompt, directUrl = false }: VerifyPromptProps) {
  const [expanded, setExpanded] = useState(false);
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [copied, setCopied] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Two approaches:
  // 1. directUrl=true: Pass entire prompt in URL (may fail with 431 if too long)
  // 2. directUrl=false: Tell AI to read from docs URL (safer for long prompts)
  const docsVerifyUrl = 'https://docs.opoch.com/verify';

  const chatGptUrl = directUrl
    ? `https://chatgpt.com/?hints=search&q=${encodeURIComponent(prompt)}`
    : `https://chatgpt.com/?hints=search&q=${encodeURIComponent(`Read from ${docsVerifyUrl} and execute the prompt shown in the code block labeled "The prompt for AI:"`)}`;

  const claudeUrl = directUrl
    ? `https://claude.ai/new?q=${encodeURIComponent(prompt)}`
    : `https://claude.ai/new?q=${encodeURIComponent(`Read from ${docsVerifyUrl} and execute the prompt shown in the code block labeled "The prompt for AI:"`)}`;

  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsDropdownOpen(false);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const copyPrompt = async () => {
    try {
      await navigator.clipboard.writeText(prompt);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy:', err);
    }
    setIsDropdownOpen(false);
  };

  // Show first ~12 lines when collapsed
  const lines = prompt.split('\n');
  const previewLines = lines.slice(0, 12).join('\n');
  const hasMore = lines.length > 12;

  return (
    <div className="verify-prompt-container">
      <div className="verify-prompt-header">
        <span className="verify-prompt-label">The prompt for AI:</span>
        <div className="verify-dropdown" ref={dropdownRef}>
          <div className="verify-dropdown-container">
            <button onClick={copyPrompt} className="verify-main-btn">
              {copied ? '✓ Copied!' : 'Copy Prompt'}
            </button>
            <button
              className="verify-toggle-btn"
              onClick={() => setIsDropdownOpen(!isDropdownOpen)}
              aria-label="More options"
            >
              <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor">
                <path d="M2.5 4.5L6 8L9.5 4.5" stroke="currentColor" strokeWidth="1.5" fill="none" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </button>
          </div>

          {isDropdownOpen && (
            <div className="verify-dropdown-menu">
              <a
                href={chatGptUrl}
                target="_blank"
                rel="noopener noreferrer"
                className="verify-dropdown-item"
                onClick={() => setIsDropdownOpen(false)}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                  <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/>
                </svg>
                <span>Open in ChatGPT</span>
              </a>

              <a
                href={claudeUrl}
                target="_blank"
                rel="noopener noreferrer"
                className="verify-dropdown-item"
                onClick={() => setIsDropdownOpen(false)}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                  <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/>
                </svg>
                <span>Open in Claude</span>
              </a>
            </div>
          )}
        </div>
      </div>
      <div className={`verify-prompt-content ${expanded ? 'expanded' : ''}`}>
        <pre>{expanded ? prompt : previewLines}{!expanded && hasMore && '\n...'}</pre>
      </div>
      {hasMore && (
        <button
          className="verify-prompt-expand"
          onClick={() => setExpanded(!expanded)}
        >
          {expanded ? 'Show less' : 'Read more'}
        </button>
      )}
    </div>
  );
}
