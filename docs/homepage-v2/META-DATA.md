# PAPER PAGE - METADATA SPEC

---

## HTML META TAGS

### Basic SEO
```html
<title>The Theory of Everything - Complete Paper | Opoch</title>

<meta name="description" content="The complete Theory of Everything in plain English and full technical PDF. Three axioms, exact receipts, verifiable predictions—from quantum mechanics to consciousness.">

<meta name="keywords" content="theory of everything, TOE, unified theory, quantum gravity, consciousness, physics, mathematics, AI verification">

<meta name="author" content="Opoch">

<link rel="canonical" href="https://opoch.com/paper">
```

---

## OPEN GRAPH (Facebook, LinkedIn, etc.)

```html
<!-- Open Graph -->
<meta property="og:type" content="article">
<meta property="og:url" content="https://opoch.com/paper">
<meta property="og:title" content="The Theory of Everything - Complete Paper">
<meta property="og:description" content="Three axioms, exact receipts, verifiable predictions. Plain English version + full technical PDF. From quantum mechanics to consciousness.">
<meta property="og:image" content="https://opoch.com/og-paper.png">
<meta property="og:image:alt" content="Theory of Everything - Opoch Paper">
<meta property="og:site_name" content="Opoch">
<meta property="og:locale" content="en_US">

<!-- Article-specific (optional) -->
<meta property="article:published_time" content="2025-01-15T00:00:00Z">
<meta property="article:author" content="Opoch">
<meta property="article:section" content="Physics">
<meta property="article:tag" content="Theory of Everything">
<meta property="article:tag" content="Quantum Gravity">
<meta property="article:tag" content="Consciousness">
```

---

## TWITTER CARD

```html
<!-- Twitter -->
<meta name="twitter:card" content="summary_large_image">
<meta name="twitter:site" content="@opoch">
<meta name="twitter:creator" content="@opoch">
<meta name="twitter:title" content="The Theory of Everything - Complete Paper">
<meta name="twitter:description" content="Three axioms, exact receipts, verifiable predictions. Plain English + full technical PDF. From quantum mechanics to consciousness.">
<meta name="twitter:image" content="https://opoch.com/twitter-paper.png">
<meta name="twitter:image:alt" content="Theory of Everything - Opoch Paper">
```

---

## ADDITIONAL META TAGS

```html
<!-- Viewport (mobile) -->
<meta name="viewport" content="width=device-width, initial-scale=1.0">

<!-- Language -->
<meta http-equiv="content-language" content="en">

<!-- Robots -->
<meta name="robots" content="index, follow">

<!-- Google -->
<meta name="googlebot" content="index, follow">

<!-- No AI scraping (optional, if you want) -->
<meta name="robots" content="noai, noimageai">

<!-- Mobile app (if you build one) -->
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black">
```

---

## STRUCTURED DATA (JSON-LD)

```html
<script type="application/ld+json">
{
  "@context": "https://schema.org",
  "@type": "ScholarlyArticle",
  "headline": "A Theory of Everything",
  "alternativeHeadline": "Complete unified theory with verifiable predictions",
  "description": "Three axioms, exact receipts, verifiable predictions—from quantum mechanics to consciousness. Plain English and full technical versions.",
  "author": {
    "@type": "Organization",
    "name": "Opoch"
  },
  "publisher": {
    "@type": "Organization",
    "name": "Opoch",
    "url": "https://opoch.com"
  },
  "datePublished": "2025-01-15",
  "dateModified": "2025-01-15",
  "inLanguage": "en",
  "url": "https://opoch.com/paper",
  "mainEntityOfPage": {
    "@type": "WebPage",
    "@id": "https://opoch.com/paper"
  },
  "about": [
    {
      "@type": "Thing",
      "name": "Theory of Everything"
    },
    {
      "@type": "Thing",
      "name": "Quantum Gravity"
    },
    {
      "@type": "Thing",
      "name": "Consciousness"
    }
  ],
  "keywords": "theory of everything, TOE, quantum gravity, consciousness, unified theory, physics, mathematics",
  "isAccessibleForFree": true,
  "license": "https://creativecommons.org/licenses/by/4.0/",
  "encoding": {
    "@type": "MediaObject",
    "contentUrl": "https://opoch.com/paper.pdf",
    "encodingFormat": "application/pdf"
  }
}
</script>
```

---

## SOCIAL SHARE IMAGE REQUIREMENTS

### Image Specs

**Open Graph (Facebook, LinkedIn):**
- Dimensions: 1200 x 630 px
- Format: PNG or JPG
- Max size: 5 MB
- Aspect ratio: 1.91:1

**Twitter Card:**
- Dimensions: 1200 x 675 px (or 1200 x 630 works too)
- Format: PNG or JPG
- Max size: 5 MB
- Aspect ratio: 16:9 (or 1.91:1)

**Recommended single size:** 1200 x 630 px (works for both)

---

### What to Put in the Image

**Option A: Minimal (Brand-aligned)**
```
─────────────────────────────────
│                               │
│    Theory of Everything       │
│    Complete Paper             │
│                               │
│    Opoch                      │
│                               │
─────────────────────────────────
```
**Style:** Black background, white text, Ω logo

---

**Option B: More Info**
```
─────────────────────────────────
│  Ω                            │
│                               │
│  Theory of Everything         │
│  The Complete Paper           │
│                               │
│  Plain English + Full PDF     │
│  Three Axioms • Exact Receipts│
│                               │
│  opoch.com/paper              │
─────────────────────────────────
```

---

**My recommendation: Option A** (minimal, matches your brand)

---

## SAMPLE SOCIAL SHARE PREVIEWS

### When Shared on Twitter:
```
┌────────────────────────────────────┐
│ [Image: 1200x630 Theory of         │
│  Everything paper preview]         │
│                                    │
│ The Theory of Everything -         │
│ Complete Paper                     │
│                                    │
│ Three axioms, exact receipts,      │
│ verifiable predictions. Plain      │
│ English + full technical PDF.      │
│                                    │
│ opoch.com                          │
└────────────────────────────────────┘
```

---

### When Shared on LinkedIn:
```
┌────────────────────────────────────┐
│ [Image: 1200x630 preview]          │
│                                    │
│ The Theory of Everything -         │
│ Complete Paper                     │
│                                    │
│ Three axioms, exact receipts,      │
│ verifiable predictions. From       │
│ quantum mechanics to consciousness.│
│                                    │
│ OPOCH.COM                          │
└────────────────────────────────────┘
```

---

## TITLE & DESCRIPTION ALTERNATIVES

### If Current Feels Too Bold:

**Title Option 1 (Current):**
```
The Theory of Everything - Complete Paper | Opoch
```
Direct, bold, clear

**Title Option 2 (Softer):**
```
Complete Unified Theory Paper - Plain English & PDF | Opoch
```
Less absolute claim

**Title Option 3 (Academic):**
```
A Theory of Everything: Axioms, Receipts, Predictions | Opoch
```
"A" instead of "The"

---

**Description Option 1 (Current):**
```
The complete Theory of Everything in plain English and full 
technical PDF. Three axioms, exact receipts, verifiable 
predictions—from quantum mechanics to consciousness.
```
Direct, comprehensive

**Description Option 2 (Benefit-focused):**
```
Read the complete unified theory in plain English or download 
the full technical PDF. Three axioms explain quantum mechanics, 
relativity, consciousness, and more—with exact receipts.
```
More accessible

**Description Option 3 (Academic):**
```
Complete mathematical framework unifying quantum mechanics, 
general relativity, thermodynamics, and consciousness. 
Available in plain English and full technical PDF.
```
More formal

---

**My recommendation:**
- **Title:** Option 1 (you've earned the boldness)
- **Description:** Option 1 (matches your brand)

---

## SITEMAP.XML ENTRY

```xml
<url>
  <loc>https://opoch.com/paper</loc>
  <lastmod>2025-01-15</lastmod>
  <changefreq>monthly</changefreq>
  <priority>1.0</priority>
</url>
```

**Priority 1.0** because it's your core content

---

## ROBOTS.TXT

```
User-agent: *
Allow: /paper
Allow: /paper.pdf

# If you want to block AI scrapers:
User-agent: GPTBot
Disallow: /

User-agent: ChatGPT-User
Disallow: /

User-agent: CCBot
Disallow: /

User-agent: anthropic-ai
Disallow: /

User-agent: Claude-Web
Disallow: /
```

**Note:** Blocking AI scrapers is ironic given your product, but your choice

---

## COMPLETE METADATA BLOCK

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <!-- Basic Meta -->
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>The Theory of Everything - Complete Paper | Opoch</title>
  <meta name="description" content="The complete Theory of Everything in plain English and full technical PDF. Three axioms, exact receipts, verifiable predictions—from quantum mechanics to consciousness.">
  <meta name="keywords" content="theory of everything, TOE, unified theory, quantum gravity, consciousness, physics, mathematics">
  <meta name="author" content="Opoch">
  <link rel="canonical" href="https://opoch.com/paper">
  
  <!-- Open Graph -->
  <meta property="og:type" content="article">
  <meta property="og:url" content="https://opoch.com/paper">
  <meta property="og:title" content="The Theory of Everything - Complete Paper">
  <meta property="og:description" content="Three axioms, exact receipts, verifiable predictions. Plain English version + full technical PDF. From quantum mechanics to consciousness.">
  <meta property="og:image" content="https://opoch.com/og-paper.png">
  <meta property="og:image:alt" content="Theory of Everything - Opoch Paper">
  <meta property="og:site_name" content="Opoch">
  
  <!-- Twitter -->
  <meta name="twitter:card" content="summary_large_image">
  <meta name="twitter:site" content="@opoch">
  <meta name="twitter:title" content="The Theory of Everything - Complete Paper">
  <meta name="twitter:description" content="Three axioms, exact receipts, verifiable predictions. Plain English + full technical PDF.">
  <meta name="twitter:image" content="https://opoch.com/twitter-paper.png">
  
  <!-- Structured Data -->
  <script type="application/ld+json">
  {
    "@context": "https://schema.org",
    "@type": "ScholarlyArticle",
    "headline": "A Theory of Everything",
    "description": "Three axioms, exact receipts, verifiable predictions—from quantum mechanics to consciousness.",
    "author": {"@type": "Organization", "name": "Opoch"},
    "publisher": {"@type": "Organization", "name": "Opoch", "url": "https://opoch.com"},
    "datePublished": "2025-01-15",
    "url": "https://opoch.com/paper",
    "isAccessibleForFree": true,
    "encoding": {"@type": "MediaObject", "contentUrl": "https://opoch.com/paper.pdf", "encodingFormat": "application/pdf"}
  }
  </script>
</head>
```

---

## TESTING YOUR METADATA

**Tools to use:**
1. **Facebook Sharing Debugger:** https://developers.facebook.com/tools/debug/
2. **Twitter Card Validator:** https://cards-dev.twitter.com/validator
3. **LinkedIn Post Inspector:** https://www.linkedin.com/post-inspector/
4. **Google Rich Results Test:** https://search.google.com/test/rich-results

**Test before launch!**

---
