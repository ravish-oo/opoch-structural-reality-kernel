# Opoch Specification Document Creation Guide

This document defines the standard for creating Opoch Kernel Specification PDFs. It ensures brand consistency, proper linking to foundational documentation, and appropriate disclaimers.

---

## 1. Title Structure

The cover page title follows a hierarchical structure:

```
Opoch Kernel Specification          <- Framework identifier (subtle)
[Domain Name]                       <- Primary title (prominent, cyan)
([Abbreviation])                    <- Short form (subtle, muted)
Technical Reference · Version X.X   <- Document type and version
```

### Styling

| Element | Font Size | Color | Notes |
|---------|-----------|-------|-------|
| Opoch Kernel Specification | 14pt | text_60 (60% white) | Subtle, establishes framework |
| Domain Name | 28pt | Cyan (#1BCDFF) | Primary focus, bold |
| Abbreviation | 11pt | text_40 (40% white) | In parentheses, very subtle |
| Version line | 10pt | text_60 (60% white) | Centered, uses middle dot separator |

### Example

```
Opoch Kernel Specification
Multi-Agent Path Finding
(MAPF)
Technical Reference · Version 3.0
```

---

## 2. Home Page Link (Primer Callout)

The cover page includes a subtle glass-morphism callout directing readers to foundational documentation.

### Placement
- Below the contract box
- Above the OPOCH branding at bottom

### Content
```
New to Opoch?  Read the Kernel Primer
```
(Where "Kernel Primer" is a cyan link to docs.opoch.com)

### Styling (Glass Morphism Box)
- **Width**: 10cm centered
- **Background**: white with 3% opacity (`rgba(255,255,255,0.03)`)
- **Border**: 0.5px white with 8% opacity (`rgba(255,255,255,0.08)`)
- **Padding**: 12px left/right, 8px top/bottom
- **Text**: 9pt, white with 50% opacity
- **"Kernel Primer"**: Cyan (#1BCDFF) link text

### OPOCH Branding at Bottom
- **"OPOCH"**: 24pt, WHITE (not cyan), bold
- **"www.opoch.com"**: 10pt, muted gray

---

## 3. Footer Structure

The footer contains context-dependent information based on page content.

### Left Footer (All Pages)
```
Confidential - Opoch Research
```
- Font: Helvetica-Oblique, 7pt
- Color: white with 40% opacity
- Position: Left-aligned, 1cm from edge

### Right Footer (Context-Dependent)

**STRICT RULES (in order of precedence):**

1. **If page has code → AI disclaimer** (always takes precedence)
2. **Else if page is Mathematical Formulation → Kernel Primer link**
3. **Else → no right footer**

#### Code Pages (STRICT: Based on actual code presence, NOT section type)
**The AI disclaimer appears if and only if the page contains actual code blocks.**

```
AI-generated reference code; use discretion.
```
- Font: Helvetica, 6pt
- Color: white with 30% opacity
- Purpose: Appropriate disclaimer for generated code
- **IMPORTANT**: This is determined by actual code presence on the page, not by section name. A "Mathematical Formulation" page WITH code gets the AI disclaimer, not the Kernel Primer link.

#### Mathematical Formulation Pages (WITHOUT code)
The Kernel Primer link appears ONLY on Mathematical Formulation pages that do NOT contain code blocks.

```
Opoch Kernel Primer → docs.opoch.com
```
- Font: Helvetica, 6pt
- Color: white with 35% opacity
- Purpose: Points readers to foundational theory

#### Other Pages
No right footer content.

### Footer Decision Logic

```
for each page:
    if page contains code blocks:
        show "AI-generated reference code; use discretion."
    else if page is in Mathematical Formulation section:
        show "Opoch Kernel Primer → docs.opoch.com"
    else:
        no right footer
```

### Example Page Classification

| Page Content | Has Code? | Footer |
|--------------|-----------|--------|
| Kernel Statement (definitions only) | No | Kernel Primer |
| Problem Definition (with code examples) | Yes | AI disclaimer |
| Truth Gate (with verification code) | Yes | AI disclaimer |
| Truth Gate (theorems only) | No | Kernel Primer |
| Method Overview (bullet points) | No | None |
| Exact Algorithms (with pseudocode) | Yes | AI disclaimer |
| Correctness (theorem boxes) | No | None |
| Appendix with code | Yes | AI disclaimer |
| Appendix with checklists | No | None |

*Note: Always audit each page for actual code presence. Never assume based on section name alone.*

---

## 4. Recommended Section Structure

The following sections are recommended for a complete Opoch Kernel Specification:

### Core Sections

1. **Executive Summary**
   - The Problem (1-2 sentences)
   - The Solution (bullet points of key guarantees)
   - Key Results (metrics, benchmarks)

2. **Mathematical Formulation**
   - Formal Definitions (sets, tuples, constraints)
   - Constraint System
   - Objective Functions
   - Theoretical Properties (complexity, completeness)

3. **Exact Algorithms**
   - Primary Algorithm (with pseudocode)
   - Alternative Approaches
   - Correctness Proofs
   - Complexity Analysis

4. **Approximation Strategies** (if applicable)
   - Heuristics
   - Bounds and Guarantees
   - Trade-off Analysis

5. **Benchmark Results**
   - Test Methodology
   - Performance Data
   - Comparison with State-of-the-Art

6. **Engineering Guide**
   - Implementation Notes
   - Data Structures
   - Optimization Tips
   - Common Pitfalls

7. **Extensions** (if applicable)
   - Domain-Specific Variants
   - Advanced Features
   - Future Directions

8. **Appendices**
   - Full Proofs
   - Additional Code
   - Reference Tables

### Section Heading Style

- H1: Section number + Title (e.g., "2. Mathematical Formulation")
- H2: Subsection number + Title (e.g., "2.1 Formal Definitions")
- H3: Topic name only (e.g., "Constraint Types")

All headings use cyan (#1BCDFF) with horizontal rule under H1.

---

## 5. Brand Colors Reference

| Name | Hex | RGB | Usage |
|------|-----|-----|-------|
| Black | #000000 | (0, 0, 0) | Background |
| Cyan | #1BCDFF | (27, 205, 255) | Headings, accents, links |
| Yellow | #FFD700 | (255, 215, 0) | Definitions, CONTRACT label |
| Purple | #B19CD9 | (177, 156, 217) | Theorems, emphasis |
| Code BG | #0A1929 | (10, 25, 41) | Code block backgrounds |
| Border | #1A3A4A | (26, 58, 74) | Table/box borders |
| text_80 | - | (0.8, 0.8, 0.8) | Primary body text |
| text_60 | - | (0.6, 0.6, 0.6) | Subtle text, metadata |
| text_40 | - | (0.4, 0.4, 0.4) | Very muted text |

---

## 6. Contract Box

Every spec includes the Opoch contract on the cover page:

```
CONTRACT
"If I speak, I have proof. If I cannot prove, I return the exact boundary."
```

### Styling
- Background: code_bg (#0A1929)
- Border: 1px solid border color
- "CONTRACT" label: Yellow (#FFD700), bold
- Quote text: text_60, italic feel
- Width: ~12cm centered
- Padding: 10-15pt all sides

---

## 7. Code Block Guidelines

### Short Code Blocks (≤15 lines)
- Use styled table with code_bg background
- Left border accent in cyan
- Keep together on single page

### Long Code Blocks (>15 lines)
- Use Preformatted text (allows page breaks)
- Same styling but can split across pages
- Maintains readability for lengthy implementations

### Code Styling
- Font: Courier, 8pt
- Color: Cyan (#1BCDFF)
- Background: code_bg (#0A1929)
- Line height: 1.4

---

## 8. Multi-Page Sections

When a section spans multiple pages, **do NOT repeat the section heading** with "(continued)". Content should flow naturally across pages without interruption.

**DO NOT:**
```
Page 1: "3. Exact Algorithms"
Page 2: "3. Exact Algorithms (continued)"  ← WRONG
```

**DO:**
```
Page 1: "3. Exact Algorithms"
         [content...]
Page 2: [content continues without heading]
         "3.1 Branch-and-Bound"
```

Subsection headings (like "3.1 Branch-and-Bound") can appear at the top of a new page - that's fine. Just don't repeat the parent section heading.

---

## 9. Document Metadata

### Header (ALL Pages Including Cover)
- Left: "OPOCH" in cyan, bold, 10pt
- Right: "Page X" in text_60, 9pt
- Cyan horizontal line below header (1px)

**Note**: The header appears on EVERY page including the cover page.

### Page Size
- A4 (210mm × 297mm)
- Margins: 2cm all sides

### Fonts
- Primary: Helvetica Neue / Arial
- Code: Courier New / Consolas

---

## Checklist for New Specs

- [ ] Title follows hierarchy: Framework → Domain → Abbreviation → Version
- [ ] Cover includes Kernel Primer callout
- [ ] Contract box present with exact quote
- [ ] Footer left: "Confidential - Opoch Research"
- [ ] Footer right: Context-appropriate (Kernel link OR code disclaimer)
- [ ] All headings in cyan with proper numbering
- [ ] Code blocks use correct styling and page-break handling
- [ ] Brand colors applied consistently
- [ ] Mathematical sections link to Kernel Primer
- [ ] Code sections have AI disclaimer
