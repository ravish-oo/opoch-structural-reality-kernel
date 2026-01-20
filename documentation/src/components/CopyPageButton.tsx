import React, { useState } from 'react';

export default function CopyPageButton() {
  const [copied, setCopied] = useState(false);

  const copyPageContent = async () => {
    try {
      // Get the main article content
      const article = document.querySelector('article');
      if (!article) return;

      // Clone to avoid modifying the DOM
      const clone = article.cloneNode(true) as HTMLElement;

      // Remove elements we don't want to copy
      clone.querySelectorAll('.theme-code-block button, .copy-page-btn, nav, .pagination-nav').forEach(el => el.remove());

      // Get clean text content
      const text = clone.innerText || clone.textContent || '';

      await navigator.clipboard.writeText(text);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy page:', err);
    }
  };

  return (
    <button
      onClick={copyPageContent}
      className="copy-page-btn"
      title="Copy page content"
      aria-label="Copy page content to clipboard"
    >
      {copied ? (
        <>
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <polyline points="20 6 9 17 4 12"></polyline>
          </svg>
          <span>Copied!</span>
        </>
      ) : (
        <>
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
            <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
          </svg>
          <span>Copy page</span>
        </>
      )}
    </button>
  );
}
