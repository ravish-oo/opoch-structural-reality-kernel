import React from 'react';

interface PreReadProps {
  title: string;
  description: string;
  linkText: string;
  linkUrl: string;
}

export default function PreRead({ title, description, linkText, linkUrl }: PreReadProps) {
  return (
    <div className="pre-read-box">
      <div className="pre-read-label">Pre-read</div>
      <div className="pre-read-content">
        <div className="pre-read-title">{title}</div>
        <div className="pre-read-description">{description}</div>
        <a href={linkUrl} className="pre-read-link">
          {linkText} →
        </a>
      </div>
    </div>
  );
}
