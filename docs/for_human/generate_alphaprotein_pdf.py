#!/usr/bin/env python3
"""
Generate styled PDF for AlphaProtein Kernel Spec using ReportLab.
Follows Opoch brand guidelines and spec_creation_spec.md.
"""

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch, cm
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, Preformatted, KeepTogether, HRFlowable
)
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_JUSTIFY, TA_RIGHT
import os

# Brand Colors from Opoch guidelines
BRAND = {
    'black': colors.Color(0, 0, 0),
    'white': colors.Color(1, 1, 1),
    'cyan': colors.Color(27/255, 205/255, 255/255),  # #1BCDFF
    'yellow': colors.Color(255/255, 215/255, 0/255),  # #FFD700
    'purple': colors.Color(177/255, 156/255, 217/255),  # #B19CD9
    'green': colors.Color(39/255, 174/255, 96/255),  # #27ae60
    'code_bg': colors.Color(10/255, 25/255, 41/255),  # #0A1929
    'border': colors.Color(26/255, 58/255, 74/255),  # #1A3A4A
    'muted': colors.Color(0.5, 0.5, 0.5),
    'text_80': colors.Color(0.8, 0.8, 0.8),
    'text_60': colors.Color(0.6, 0.6, 0.6),
    'text_40': colors.Color(0.4, 0.4, 0.4),
}

def create_styles():
    """Create custom paragraph styles following brand guidelines."""
    styles = getSampleStyleSheet()

    styles.add(ParagraphStyle(
        name='Title_Custom',
        fontName='Helvetica-Bold',
        fontSize=28,
        textColor=BRAND['cyan'],
        spaceAfter=20,
        alignment=TA_CENTER,
    ))

    styles.add(ParagraphStyle(
        name='Subtitle_Custom',
        fontName='Helvetica',
        fontSize=16,
        textColor=BRAND['cyan'],
        spaceAfter=10,
        alignment=TA_CENTER,
    ))

    styles.add(ParagraphStyle(
        name='Op_H1',
        fontName='Helvetica-Bold',
        fontSize=20,
        textColor=BRAND['cyan'],
        spaceBefore=30,
        spaceAfter=15,
    ))

    styles.add(ParagraphStyle(
        name='Op_H2',
        fontName='Helvetica-Bold',
        fontSize=16,
        textColor=BRAND['cyan'],
        spaceBefore=20,
        spaceAfter=10,
    ))

    styles.add(ParagraphStyle(
        name='Op_H3',
        fontName='Helvetica-Bold',
        fontSize=13,
        textColor=BRAND['cyan'],
        spaceBefore=15,
        spaceAfter=8,
    ))

    styles.add(ParagraphStyle(
        name='Op_H4',
        fontName='Helvetica-Bold',
        fontSize=11,
        textColor=BRAND['cyan'],
        spaceBefore=12,
        spaceAfter=6,
    ))

    styles.add(ParagraphStyle(
        name='Op_Body',
        fontName='Helvetica',
        fontSize=10,
        textColor=BRAND['text_80'],
        spaceAfter=8,
        alignment=TA_JUSTIFY,
        leading=14,
    ))

    styles.add(ParagraphStyle(
        name='Op_Code',
        fontName='Courier',
        fontSize=8,
        textColor=BRAND['cyan'],
        backColor=BRAND['code_bg'],
        spaceAfter=10,
        leftIndent=10,
        rightIndent=10,
        leading=11,
    ))

    styles.add(ParagraphStyle(
        name='Op_Definition',
        fontName='Helvetica-Bold',
        fontSize=10,
        textColor=BRAND['yellow'],
        spaceAfter=5,
    ))

    styles.add(ParagraphStyle(
        name='Op_Theorem',
        fontName='Helvetica-BoldOblique',
        fontSize=10,
        textColor=BRAND['purple'],
        spaceAfter=5,
    ))

    styles.add(ParagraphStyle(
        name='Op_Muted',
        fontName='Helvetica-Oblique',
        fontSize=9,
        textColor=BRAND['text_60'],
        alignment=TA_CENTER,
    ))

    styles.add(ParagraphStyle(
        name='Op_Bullet',
        fontName='Helvetica',
        fontSize=10,
        textColor=BRAND['text_80'],
        leftIndent=20,
        bulletIndent=10,
        spaceAfter=4,
    ))

    return styles

def header_footer(canvas, doc):
    """Draw header and footer on each page."""
    canvas.saveState()

    # Paint black background
    canvas.setFillColor(BRAND['black'])
    canvas.rect(0, 0, A4[0], A4[1], fill=1, stroke=0)

    # Header
    canvas.setFillColor(BRAND['cyan'])
    canvas.setFont('Helvetica-Bold', 10)
    canvas.drawString(1*cm, A4[1] - 1*cm, "OPOCH")

    # Page number
    canvas.setFillColor(BRAND['muted'])
    canvas.setFont('Helvetica', 9)
    canvas.drawRightString(A4[0] - 1*cm, A4[1] - 1*cm, f"Page {doc.page}")

    # Header line
    canvas.setStrokeColor(BRAND['cyan'])
    canvas.setLineWidth(1)
    canvas.line(1*cm, A4[1] - 1.3*cm, A4[0] - 1*cm, A4[1] - 1.3*cm)

    # Footer - left side (all pages)
    canvas.setFillColor(colors.Color(1, 1, 1, alpha=0.4))
    canvas.setFont('Helvetica-Oblique', 7)
    canvas.drawString(1*cm, 1*cm, "Confidential - Opoch Research")

    # Footer - right side (context-dependent)
    # STRICT RULES:
    # 1. If page has code → AI-generated reference code disclaimer
    # 2. Else if page is in Mathematical Formulation (1.1, 1.2, 1.3) WITHOUT code → Kernel Primer
    # 3. Else → no right footer
    #
    # Pages WITH code blocks (determined by actual content):
    # - Page 6-7: Problem Definition (input_code, space_code, obj_code, witness_code)
    # - Page 8: Truth Gate (v1_code, v2_code)
    # - Pages 11-12: Exact Algorithms (region_code, bb_code, bb_code2, feas_code)
    # - Page 14: Omega Semantics (unsat_code, multi_code, gap_code)
    # - Page 18: Implementation Playbook (receipt_code)
    # - Page 20: Appendix A (pass_code, fail_code)
    # - Pages 21-22: Appendix B (impl_code1, impl_code2)
    #
    # Mathematical Formulation pages WITHOUT code:
    # - Pages 4-5: Kernel Statement (definition boxes only)
    # - Page 9: Truth Gate theorems (theorem boxes only)

    page = doc.page

    # Pages with actual code blocks - AI disclaimer
    pages_with_code = {6, 7, 8, 11, 12, 14, 18, 20, 21, 22}

    # Mathematical Formulation pages without code - Kernel Primer
    math_pages_no_code = {4, 5, 9}

    if page in pages_with_code:
        canvas.setFillColor(colors.Color(1, 1, 1, alpha=0.3))
        canvas.setFont('Helvetica', 6)
        canvas.drawRightString(A4[0] - 1*cm, 1*cm, "AI-generated reference code; use discretion.")
    elif page in math_pages_no_code:
        canvas.setFillColor(colors.Color(1, 1, 1, alpha=0.35))
        canvas.setFont('Helvetica', 6)
        canvas.drawRightString(A4[0] - 1*cm, 1*cm, "Opoch Kernel Primer → docs.opoch.com")
    # All other pages: no right footer

    canvas.restoreState()

def cover_page_footer(canvas, doc):
    """Cover page has no header, just footer."""
    canvas.saveState()

    # Paint black background
    canvas.setFillColor(BRAND['black'])
    canvas.rect(0, 0, A4[0], A4[1], fill=1, stroke=0)

    # Footer only
    canvas.setFillColor(colors.Color(1, 1, 1, alpha=0.4))
    canvas.setFont('Helvetica-Oblique', 7)
    canvas.drawString(1*cm, 1*cm, "Confidential - Opoch Research")

    canvas.restoreState()

def create_code_block(code, styles, keep_together=True):
    """Create a styled code block."""
    lines = code.split('\n')
    if len(lines) > 15:
        code_style = ParagraphStyle(
            'CodeBlockLong',
            fontName='Courier',
            fontSize=8,
            textColor=BRAND['cyan'],
            leading=11,
            leftIndent=15,
            backColor=BRAND['code_bg'],
        )
        return Preformatted(code, code_style)

    code_data = [[Preformatted(code, ParagraphStyle(
        'CodeInner',
        fontName='Courier',
        fontSize=8,
        textColor=BRAND['cyan'],
        leading=11,
    ))]]
    code_table = Table(code_data, colWidths=[16*cm])
    code_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), BRAND['code_bg']),
        ('LEFTPADDING', (0, 0), (-1, -1), 15),
        ('RIGHTPADDING', (0, 0), (-1, -1), 15),
        ('TOPPADDING', (0, 0), (-1, -1), 10),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 10),
        ('BOX', (0, 0), (-1, -1), 0.5, BRAND['border']),
    ]))
    if keep_together:
        return KeepTogether([code_table])
    return code_table

def create_definition_box(title, content, styles):
    """Create a definition box with yellow title."""
    data = [
        [Paragraph(f'<font color="#FFD700"><b>{title}</b></font>', styles['Op_Body'])],
        [Paragraph(content, ParagraphStyle('def_content', parent=styles['Op_Body'], textColor=BRAND['text_80']))]
    ]
    table = Table(data, colWidths=[16*cm])
    table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), BRAND['code_bg']),
        ('BOX', (0, 0), (-1, -1), 1, BRAND['border']),
        ('LEFTPADDING', (0, 0), (-1, -1), 15),
        ('RIGHTPADDING', (0, 0), (-1, -1), 15),
        ('TOPPADDING', (0, 0), (-1, -1), 10),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 10),
    ]))
    return KeepTogether([table, Spacer(1, 0.3*cm)])

def create_theorem_box(title, content, styles):
    """Create a theorem box with purple title."""
    data = [
        [Paragraph(f'<font color="#B19CD9"><i><b>{title}</b></i></font>', styles['Op_Body'])],
        [Paragraph(content, ParagraphStyle('thm_content', parent=styles['Op_Body'], textColor=BRAND['text_80']))]
    ]
    table = Table(data, colWidths=[16*cm])
    table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), BRAND['code_bg']),
        ('BOX', (0, 0), (-1, -1), 1, BRAND['border']),
        ('LEFTPADDING', (0, 0), (-1, -1), 15),
        ('RIGHTPADDING', (0, 0), (-1, -1), 15),
        ('TOPPADDING', (0, 0), (-1, -1), 10),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 10),
    ]))
    return KeepTogether([table, Spacer(1, 0.3*cm)])

def create_insight_box(title, content, styles):
    """Create an insight box with green title."""
    data = [
        [Paragraph(f'<font color="#27ae60"><b>{title}</b></font>', styles['Op_Body'])],
        [Paragraph(content, ParagraphStyle('insight_content', parent=styles['Op_Body'], textColor=BRAND['text_80']))]
    ]
    table = Table(data, colWidths=[16*cm])
    table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), BRAND['code_bg']),
        ('BOX', (0, 0), (-1, -1), 1, BRAND['green']),
        ('LEFTPADDING', (0, 0), (-1, -1), 15),
        ('RIGHTPADDING', (0, 0), (-1, -1), 15),
        ('TOPPADDING', (0, 0), (-1, -1), 10),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 10),
    ]))
    return KeepTogether([table, Spacer(1, 0.3*cm)])

def create_cover_page(styles):
    """Create cover page following MAPF spec styling."""
    elements = []

    elements.append(Spacer(1, 3*cm))

    # Title hierarchy - matching MAPF styling exactly
    elements.append(Paragraph(
        "Opoch Kernel Specification",
        ParagraphStyle('spec_label', parent=styles['Op_Body'], fontSize=14, textColor=BRAND['text_60'], alignment=TA_CENTER)
    ))
    elements.append(Spacer(1, 0.5*cm))
    elements.append(Paragraph(
        "Protein Structure Inference",
        ParagraphStyle('domain_title', parent=styles['Title_Custom'], fontSize=28, textColor=BRAND['cyan'])
    ))
    elements.append(Spacer(1, 0.6*cm))
    elements.append(Paragraph(
        "Technical Reference  ·  Version 1.0",
        ParagraphStyle('version', parent=styles['Op_Body'], textColor=BRAND['text_60'], alignment=TA_CENTER, fontSize=10)
    ))

    elements.append(Spacer(1, 2*cm))

    # Contract box
    contract_data = [
        [Paragraph('<font color="#FFD700"><b>CONTRACT</b></font>', styles['Op_Body'])],
        [Paragraph('"If I speak, I have proof. If I cannot prove, I return the exact boundary."',
                   ParagraphStyle('contract', parent=styles['Op_Body'], textColor=BRAND['text_60']))]
    ]
    contract_table = Table(contract_data, colWidths=[12*cm])
    contract_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), BRAND['code_bg']),
        ('BOX', (0, 0), (-1, -1), 1, BRAND['border']),
        ('LEFTPADDING', (0, 0), (-1, -1), 15),
        ('RIGHTPADDING', (0, 0), (-1, -1), 15),
        ('TOPPADDING', (0, 0), (-1, -1), 10),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 10),
    ]))
    elements.append(contract_table)

    elements.append(Spacer(1, 1.5*cm))

    # Primer callout - glass morphism style matching MAPF exactly
    primer_data = [
        [Paragraph(
            'New to Opoch?  Read the <font color="#1BCDFF"><link href="https://docs.opoch.com">Kernel Primer</link></font>',
            ParagraphStyle('primer', parent=styles['Op_Body'],
                          textColor=colors.Color(1, 1, 1, alpha=0.5),
                          alignment=TA_CENTER, fontSize=9)
        )]
    ]
    primer_table = Table(primer_data, colWidths=[10*cm])
    primer_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.Color(1, 1, 1, alpha=0.03)),  # bg-white/3
        ('BOX', (0, 0), (-1, -1), 0.5, colors.Color(1, 1, 1, alpha=0.08)),  # border-white/8
        ('LEFTPADDING', (0, 0), (-1, -1), 12),
        ('RIGHTPADDING', (0, 0), (-1, -1), 12),
        ('TOPPADDING', (0, 0), (-1, -1), 8),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 8),
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
    ]))
    elements.append(primer_table)

    elements.append(Spacer(1, 1.5*cm))

    # Opoch branding - WHITE text like MAPF
    elements.append(Paragraph('<b>OPOCH</b>',
        ParagraphStyle('logo', parent=styles['Title_Custom'], fontSize=24, textColor=BRAND['white'])))
    elements.append(Paragraph('www.opoch.com',
        ParagraphStyle('url', parent=styles['Op_Muted'], fontSize=10)))

    elements.append(PageBreak())
    return elements

def create_index(styles):
    """Create table of contents with proper section numbering."""
    elements = []

    elements.append(Paragraph("Index", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    # TOC with proper numbering: (text, page, indent_level)
    # indent_level: 0 = main section, 1 = subsection, 2 = sub-subsection
    toc_items = [
        # 0. Executive Summary
        ("0. Executive Summary", "3", 0),

        # 1. Mathematical Formulation
        ("1. Mathematical Formulation", "", 0),
        ("1.1 Kernel Statement", "4-5", 1),
        ("1.1.1 Possibility Space W", "4", 2),
        ("1.1.2 Tests Delta", "4", 2),
        ("1.1.3 Truth Pi (Quotient)", "4", 2),
        ("1.1.4 Omega Frontier", "4", 2),
        ("1.1.5 tau* (Forced Separator)", "5", 2),
        ("1.2 Problem Definition", "6-7", 1),
        ("1.2.1 Inputs", "6", 2),
        ("1.2.2 Conformation Space", "6", 2),
        ("1.2.3 Objective (Pinned Contract)", "6", 2),
        ("1.2.4 Output Contract", "7", 2),
        ("1.3 Truth Gate (Verifier)", "8-9", 1),
        ("1.3.1 Verification Checks V1-V5", "8", 2),
        ("1.3.2 Verifier Theorems", "9", 2),

        # 2. Method Overview (single item - no subsections)
        ("2. Method Overview", "10", 0),

        # 3. Exact Algorithms
        ("3. Exact Algorithms", "11-13", 0),
        ("3.1 Branch-and-Bound", "11", 1),
        ("3.2 Feasibility Oracle", "12", 1),
        ("3.3 Algorithm Theorems", "13", 1),

        # 4. Failure Modes and Bounded Outputs
        ("4. Failure Modes and Bounded Outputs", "14", 0),

        # 5. Correctness and Guarantees (single item)
        ("5. Correctness and Guarantees", "15-16", 0),

        # 6. Implementation Playbook (single item)
        ("6. Implementation Playbook", "17-18", 0),

        # 7. Optional Extensions (single item)
        ("7. Optional Extensions", "19", 0),

        # Appendices
        ("Appendices", "", 0),
        ("A. Sample Verification Output", "20", 1),
        ("B. Reference Implementation Skeleton", "21-22", 1),
        ("C. Production Checklist", "23", 1),
    ]

    toc_data = []
    for item, page, level in toc_items:
        if level == 0:
            # Main section - bold cyan
            style = ParagraphStyle('toc_section', parent=styles['Op_Body'], fontName='Helvetica-Bold', textColor=BRAND['cyan'])
            indent = 0
        elif level == 1:
            # Subsection
            style = ParagraphStyle('toc_subsection', parent=styles['Op_Body'])
            indent = 15
        else:
            # Sub-subsection
            style = ParagraphStyle('toc_subsubsection', parent=styles['Op_Body'], textColor=BRAND['text_60'])
            indent = 30

        toc_data.append([
            Paragraph(item, ParagraphStyle('toc_item', parent=style, leftIndent=indent)),
            Paragraph(f'<font color="#1BCDFF">{page}</font>' if page else '', ParagraphStyle('toc_page', parent=styles['Op_Body'], alignment=TA_RIGHT))
        ])

    toc_table = Table(toc_data, colWidths=[14*cm, 2*cm])
    toc_table.setStyle(TableStyle([
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 0),
        ('RIGHTPADDING', (0, 0), (-1, -1), 0),
        ('TOPPADDING', (0, 0), (-1, -1), 2),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 2),
    ]))
    elements.append(toc_table)

    elements.append(PageBreak())
    return elements

def create_executive_summary(styles):
    """Create Executive Summary section (Section 0)."""
    elements = []

    elements.append(Paragraph("0. Executive Summary", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("The Problem", styles['Op_H3']))
    elements.append(Paragraph(
        "Protein structure prediction attempts to guess the native 3D conformation from amino acid sequence. Current methods (including AlphaFold) produce predictions without certificates of correctness or explicit handling of degeneracy.",
        styles['Op_Body']
    ))

    elements.append(Paragraph("The Solution", styles['Op_H3']))
    elements.append(Paragraph(
        "We compile protein folding into Opoch's Kernel Interface—a mathematical framework that transforms the problem from \"prediction\" into \"quotient collapse under pinned tests.\" This provides:",
        styles['Op_Body']
    ))
    elements.append(Paragraph("• <b>Verifiable correctness</b>: Every solution is cryptographically receipted and polynomial-time checkable", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Certified optimization</b>: Branch-and-bound with LB/UB certificates proves optimality", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Honest ambiguity</b>: Multiple valid folds (OMEGA_MULTI) or resource limits (OMEGA_GAP) are explicitly reported", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Deterministic outputs</b>: Same inputs always produce same outputs with replayable receipts", styles['Op_Bullet']))

    elements.append(Paragraph("Key Results", styles['Op_H3']))
    elements.append(Paragraph(
        "The kernel approach guarantees:",
        styles['Op_Body']
    ))
    elements.append(Paragraph("• <b>Soundness</b>: Only verifier-pass structures are returned", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Completeness</b>: All admissible conformations are reachable given sufficient budget", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>No false positives</b>: UNIQUE requires certified closure", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>No false negatives</b>: UNSAT requires explicit impossibility proof", styles['Op_Bullet']))

    elements.append(create_insight_box(
        "Core Insight",
        "The verifier is truth. Everything else is proposal. By separating the verifier (source of truth) from solvers (proposal mechanisms), we transform protein folding from guessing to proving.",
        styles
    ))

    elements.append(PageBreak())
    return elements

def create_section_1(styles):
    """1.1 Kernel Statement section."""
    elements = []

    elements.append(Paragraph("1.1 Kernel Statement", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph(
        "Protein folding is not prediction. It is quotient-collapse under pinned tests. We compile protein structure inference into kernel primitives: possibility space, tests, truth quotient, frontier, and forced separator.",
        styles['Op_Body']
    ))

    # 1.1.1 Possibility Space W
    elements.append(Paragraph("1.1.1 Possibility Space W", styles['Op_H2']))
    elements.append(create_definition_box(
        "Definition: W",
        "W = set of all conformations X admissible under hard chemistry constraints for sequence S (and optional evidence ledger L). Each X encodes 3D atom coordinates or internal coordinates.",
        styles
    ))

    # 1.1.2 Tests Delta
    elements.append(Paragraph("1.1.2 Tests Delta", styles['Op_H2']))
    elements.append(Paragraph("Each verifier check is a finite, decidable test:", styles['Op_Body']))
    elements.append(Paragraph("• <b>GEOM test</b>: bond lengths/angles/chirality/planarity within tolerance", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>STERIC test</b>: no forbidden overlaps (hard exclusion radius)", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>LEDGER test</b>: each evidence item (restraint/density/contact) within tolerance", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>OBJECTIVE test</b>: energy/objective evaluation and bound certificates", styles['Op_Bullet']))

    # 1.1.3 Truth Pi
    elements.append(Paragraph("1.1.3 Truth Pi (Quotient)", styles['Op_H2']))
    elements.append(create_definition_box(
        "Definition: Pi",
        "Truth is defined modulo gauge/representation slack:<br/>- Global rigid motions (translation + rotation) are gauge<br/>- Equivalent coordinate parameterizations are gauge<br/><br/>Two conformations are equivalent if all feasible tests agree (and differ only by gauge). Pi collapses minted distinctions.",
        styles
    ))

    # 1.1.4 Omega Frontier
    elements.append(Paragraph("1.1.4 Omega Frontier", styles['Op_H2']))
    elements.append(create_definition_box(
        "Definition: Omega",
        "Omega is NOT guessing. It is one of two forms:<br/>- <b>OMEGA_MULTI</b>: multiple distinct optimal basins survive (true degeneracy under verifier)<br/>- <b>OMEGA_GAP</b>: undecided under budget; return frontier basins + best lower bound + next test",
        styles
    ))

    elements.append(PageBreak())

    # 1.1.5 tau*
    elements.append(Paragraph("1.1.5 tau* (Forced Separator)", styles['Op_H2']))
    elements.append(create_definition_box(
        "Definition: tau*",
        "tau* is the deterministic next distinguisher:<br/>- For optimization: the next split variable/region that maximally tightens bounds<br/>- For ambiguity: the cheapest missing evidence test that separates top competing basins<br/><br/>Tie-break must be Pi-invariant (canonical ordering).",
        styles
    ))

    elements.append(create_insight_box(
        "Key Insight",
        "Branch-and-bound on conformational regions IS the kernel refinement algorithm: split on tau* (best bounding variable), prune by certificates, repeat until UNIQUE (certified optimum) or Omega (degeneracy or budget limit).",
        styles
    ))

    elements.append(Paragraph(
        'This kernel view transforms protein folding from "guessing structures" to "collapsing possibility space under verifiable tests."',
        styles['Op_Body']
    ))
    elements.append(Paragraph(
        "<b>The verifier is truth. Everything else is proposal.</b>",
        ParagraphStyle('emphasis', parent=styles['Op_Body'], textColor=BRAND['cyan'])
    ))

    elements.append(PageBreak())
    return elements

def create_section_2(styles):
    """1.2 Problem Definition section."""
    elements = []

    elements.append(Paragraph("1.2 Problem Definition", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph(
        "Find the optimal protein conformation(s) for a given sequence under specified constraints.",
        styles['Op_Body']
    ))

    # 1.2.1 Inputs
    elements.append(Paragraph("1.2.1 Inputs", styles['Op_H2']))
    input_code = """S = (s_1, ..., s_L)   Amino-acid sequence of length L
C = conditions        Solvent, temperature, ionic strength, partners, etc.
L = evidence ledger   Optional: constraints/tests with declared tolerances
epsilon = tolerance   Optimality certificate precision
delta = ensemble tol  For credible sets (optional)"""
    elements.append(create_code_block(input_code, styles))

    # 1.2.2 Conformation Space
    elements.append(Paragraph("1.2.2 Conformation Space (Admissible Geometry)", styles['Op_H2']))
    space_code = """Let X encode 3D coordinates of atoms (or internal coordinates)
subject to hard constraints:
  - Covalent geometry constraints (bond lengths, angles)
  - Chirality constraints (L-amino acids, proline rings)
  - Steric exclusion constraints (van der Waals radii)

Denote admissible set: Omega(S) subset of R^n"""
    elements.append(create_code_block(space_code, styles))

    # 1.2.3 Objective
    elements.append(Paragraph("1.2.3 Objective (Pinned Contract)", styles['Op_H2']))
    obj_code = """Define the objective energy E(X) as a declared, total function:

  E: Omega(S) -> R

IMPORTANT: "Complete" requires E be pinned (versioned) and replayable.
Optionally incorporate evidence as hard constraints or penalty terms."""
    elements.append(create_code_block(obj_code, styles))

    elements.append(Paragraph("Two canonical query types:", styles['Op_Body']))
    elements.append(Paragraph("• <b>(A) MAP</b> (single best structure): Find X* in argmin_{X in Omega(S) ∩ L} E(X)", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>(B) Certified ensemble</b>: Return set of basins whose total certified mass >= 1-delta", styles['Op_Bullet']))

    elements.append(PageBreak())

    # 1.2.4 Output Contract
    elements.append(Paragraph("1.2.4 Output Contract", styles['Op_H2']))
    elements.append(Paragraph("Every query terminates in exactly one of four states:", styles['Op_Body']))
    elements.append(Paragraph("• <b>UNIQUE</b>: single certified optimum class (up to gauge) + receipt", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>OMEGA_MULTI</b>: multiple certified optimum classes survive (degeneracy) + receipt", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>UNSAT</b>: no X satisfies hard constraints + ledger (infeasible) + certificate", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>OMEGA_GAP</b>: undecided under compute budget; frontier basins + bounds + next distinguisher", styles['Op_Bullet']))

    witness_code = """Output witness W = {
  sequence:    S
  conditions:  C
  ledger:      L
  structure:   X* (or basins for OMEGA_MULTI)
  energy:      E(X*)
  certificate: LB/UB closure proof
  verifier:    PASS
  receipt:     SHA256(canonical(W))
}"""
    elements.append(create_code_block(witness_code, styles))

    elements.append(create_definition_box(
        "Note: Gauge Invariance",
        "UNIQUE means unique up to gauge (rigid body motion). Two structures related by translation and rotation are the same answer. The canonical form centers at centroid and aligns principal axes.",
        styles
    ))

    elements.append(PageBreak())
    return elements

def create_section_3(styles):
    """1.3 Truth Gate (Verifier) section."""
    elements = []

    elements.append(Paragraph("1.3 Truth Gate (Verifier)", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph(
        "The verifier is the SOURCE OF TRUTH. Branch-and-bound, neural networks, and all other solvers are proposal mechanisms. Only the verifier determines validity.",
        styles['Op_Body']
    ))

    # 1.3.1 Verification Checks
    elements.append(Paragraph("1.3.1 Verification Checks", styles['Op_H2']))

    check_data = [
        [Paragraph('<font color="#1BCDFF"><b>Check</b></font>', styles['Op_Body']),
         Paragraph('<font color="#1BCDFF"><b>Condition</b></font>', styles['Op_Body']),
         Paragraph('<font color="#1BCDFF"><b>On Failure</b></font>', styles['Op_Body'])],
        [Paragraph('V1: Geometry', styles['Op_Body']),
         Paragraph('bonds/angles within tol', styles['Op_Body']),
         Paragraph('first violation witness', styles['Op_Body'])],
        [Paragraph('V2: Sterics', styles['Op_Body']),
         Paragraph('no forbidden overlaps', styles['Op_Body']),
         Paragraph('violating atom pair', styles['Op_Body'])],
        [Paragraph('V3: Ledger', styles['Op_Body']),
         Paragraph('all evidence constraints', styles['Op_Body']),
         Paragraph('first failing constraint', styles['Op_Body'])],
        [Paragraph('V4: Objective', styles['Op_Body']),
         Paragraph('compute E(X) exactly', styles['Op_Body']),
         Paragraph('pinned spec result', styles['Op_Body'])],
        [Paragraph('V5: Certificate', styles['Op_Body']),
         Paragraph('LB/UB consistency', styles['Op_Body']),
         Paragraph('bound violation witness', styles['Op_Body'])],
    ]
    check_table = Table(check_data, colWidths=[4*cm, 5.5*cm, 5.5*cm])
    check_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), BRAND['code_bg']),
        ('BACKGROUND', (0, 1), (-1, -1), colors.Color(0.05, 0.05, 0.05)),
        ('GRID', (0, 0), (-1, -1), 0.5, BRAND['border']),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
    ]))
    elements.append(check_table)
    elements.append(Spacer(1, 0.5*cm))

    # 1.3.1.1 Detailed Check Specifications
    elements.append(Paragraph("Detailed Check Specifications", styles['Op_H3']))

    elements.append(Paragraph("V1: Geometry Check", styles['Op_H3']))
    v1_code = """def check_geometry(X, S, tolerances):
    for bond in covalent_bonds(S):
        d = distance(X[bond.atom1], X[bond.atom2])
        if abs(d - bond.ideal_length) > tolerances.bond:
            return FAIL(witness=("BOND", bond, d, bond.ideal_length))

    for angle in bond_angles(S):
        theta = compute_angle(X[angle.a1], X[angle.a2], X[angle.a3])
        if abs(theta - angle.ideal) > tolerances.angle:
            return FAIL(witness=("ANGLE", angle, theta, angle.ideal))

    # Check chirality, planarity...
    return PASS"""
    elements.append(create_code_block(v1_code, styles))

    elements.append(Paragraph("V2: Steric Check", styles['Op_H3']))
    v2_code = """def check_sterics(X, radii, tolerance):
    for i, j in all_nonbonded_pairs(X):
        d = distance(X[i], X[j])
        min_d = radii[i] + radii[j] - tolerance
        if d < min_d:
            return FAIL(witness=("CLASH", i, j, d, min_d))
    return PASS"""
    elements.append(create_code_block(v2_code, styles))

    elements.append(PageBreak())

    # 1.3.2 Verifier Theorems
    elements.append(Paragraph("1.3.2 Verifier Theorems", styles['Op_H2']))

    elements.append(create_theorem_box(
        "Theorem 3.1: Verifier Soundness",
        "If verify(X) = PASS, then X is admissible and satisfies the contract. Proof: The verifier checks all necessary conditions: geometry (V1), sterics (V2), ledger constraints (V3), objective computation (V4), and certificate validity (V5). Each check is deterministic with explicit tolerance. PASS requires all checks pass. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Theorem 3.2: Verifier Completeness",
        "If X satisfies the contract, then verify(X) = PASS. Proof: The verifier checks are exactly the defining conditions of an admissible conformation under the pinned specification. Any X satisfying all conditions will pass all checks. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Theorem 3.3: Minimal Separator Property",
        'If verify(X) = FAIL, the returned violation is a minimal separator witness. Proof: Each check returns the first violation found (deterministic ordering). This violation is sufficient to prove X is not admissible (single constraint failure). It is minimal in the sense that removing it would require re-checking. QED.',
        styles
    ))

    elements.append(create_definition_box(
        "Implementation Note",
        "The verifier must be deterministic and reproducible. Same inputs must produce identical outputs across implementations. Use exact arithmetic where possible, or document numerical tolerances precisely.",
        styles
    ))

    elements.append(PageBreak())
    return elements

def create_section_4(styles):
    """2. Method Overview section."""
    elements = []

    elements.append(Paragraph("2. Method Overview", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph(
        'We solve protein folding because we understand its structural reality: it is not "prediction"; it is quotient-collapse under pinned tests.',
        styles['Op_Body']
    ))

    elements.append(Paragraph("Core Principles", styles['Op_H2']))
    elements.append(Paragraph("• <b>Verifier defines reality</b>: admissible conformations are Pi-classes that PASS", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Ambiguity is Omega</b>: if multiple basins survive, output them or the missing distinguisher", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Certified optimization replaces guessing</b>: bounds are the only truthful proof of optimality", styles['Op_Bullet']))
    elements.append(Paragraph("• <b>Receipts make refinements reusable</b>: cost falls with use (compounding intelligence)", styles['Op_Bullet']))

    elements.append(Paragraph("Why This Works", styles['Op_H2']))
    elements.append(Paragraph(
        'Traditional protein structure prediction attempts to "guess" the native state. This is fundamentally wrong. The kernel approach instead:',
        styles['Op_Body']
    ))
    elements.append(Paragraph("1. <b>Pins the contract</b>: E(X) is versioned and reproducible", styles['Op_Bullet']))
    elements.append(Paragraph('2. <b>Defines truth via verifier</b>: no hand-waving about "close enough"', styles['Op_Bullet']))
    elements.append(Paragraph("3. <b>Uses certified optimization</b>: bounds prove optimality", styles['Op_Bullet']))
    elements.append(Paragraph("4. <b>Handles ambiguity honestly</b>: OMEGA_MULTI for true degeneracy", styles['Op_Bullet']))
    elements.append(Paragraph("5. <b>Compounds intelligence</b>: every solved problem accelerates future queries", styles['Op_Bullet']))

    elements.append(create_insight_box(
        "Key Insight",
        'The protein folding "problem" is not about predicting nature. It is about finding conformations that minimize a pinned energy function under verifiable constraints. Nature uses different physics than our models. Our job is to solve OUR model exactly, then improve the model based on experimental feedback.',
        styles
    ))

    elements.append(PageBreak())
    return elements

def create_section_5(styles):
    """3. Exact Algorithms section."""
    elements = []

    elements.append(Paragraph("3. Exact Algorithms", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("3.1 Branch-and-Bound", styles['Op_H2']))
    elements.append(Paragraph("The core algorithm for finding certified optimal conformations.", styles['Op_Body']))

    elements.append(Paragraph("Key Object: Region R", styles['Op_H3']))
    region_code = """Region R subset Omega(S) described by:
  - Bounded internal coordinate ranges (phi, psi, chi angles)
  - Discrete rotamer sets (if used)
  - Additional constraint propagation results

Required primitives (declared once):
  - Bound(R): returns (LB(R), UB(R)) such that
      LB(R) <= inf_{X in R} E(X) <= UB(R)
  - Feasible(R): returns whether R contains any admissible X
      (or gives UNSAT witness if impossible inside R)"""
    elements.append(create_code_block(region_code, styles))

    elements.append(Paragraph("Branch-and-Bound Algorithm", styles['Op_H3']))
    bb_code = """def branch_and_bound(S, L, E, epsilon, max_nodes):
    # Initialize with full conformation space
    R0 = initial_region(S)
    queue = PriorityQueue()  # by LB
    queue.push((Bound(R0).LB, R0))

    X_best = None
    UB_best = infinity
    nodes_expanded = 0

    while not queue.empty():
        LB_R, R = queue.pop()
        nodes_expanded += 1

        # Pruning: if this region can't improve, skip
        if LB_R >= UB_best - epsilon:
            continue

        # Check feasibility
        feas = Feasible(R, L)
        if feas == UNSAT:
            continue  # Prune infeasible region

        # Try to find a feasible point in R
        X_sample = sample_feasible(R, L)
        if X_sample is not None:
            E_sample = E(X_sample)
            if E_sample < UB_best:
                X_best = X_sample
                UB_best = E_sample"""
    elements.append(create_code_block(bb_code, styles, keep_together=False))

    elements.append(PageBreak())

    bb_code2 = """        # Split region by tau* rule
        tau_star = select_split_variable(R)
        R1, R2 = split_region(R, tau_star)

        for Ri in [R1, R2]:
            LB_i, UB_i = Bound(Ri)
            if LB_i < UB_best - epsilon:
                queue.push((LB_i, Ri))

        # Check node budget
        if nodes_expanded >= max_nodes:
            return OMEGA_GAP(
                frontier=queue.top_regions(k=10),
                best_LB=queue.min_LB(),
                best_UB=UB_best,
                best_X=X_best,
                tau_star=select_next_split(queue)
            )

    # Queue exhausted
    if X_best is None:
        return UNSAT(reason="No feasible conformation found")

    # Check for degeneracy (multiple optimal basins)
    optimal_basins = find_basins_within_epsilon(X_best, epsilon)
    if len(optimal_basins) > 1:
        return OMEGA_MULTI(basins=optimal_basins, energy=UB_best)

    return UNIQUE(
        structure=X_best,
        energy=UB_best,
        certificate=(global_LB, UB_best),
        receipt=SHA256(canonical(X_best, UB_best))
    )"""
    elements.append(create_code_block(bb_code2, styles, keep_together=False))

    # 3.2 Feasibility Oracle
    elements.append(Paragraph("3.2 Feasibility Oracle (UNSAT Certificates)", styles['Op_H2']))
    elements.append(Paragraph("When ledger constraints are inconsistent with hard chemistry:", styles['Op_Body']))

    feas_code = """def feasibility_oracle(R, L, S):
    # Check geometry constraints
    geom_result = check_geometry_feasibility(R, S)
    if geom_result == UNSAT:
        return UNSAT(witness=geom_result.witness)

    # Check steric constraints
    steric_result = check_steric_feasibility(R)
    if steric_result == UNSAT:
        return UNSAT(witness=steric_result.witness)

    # Check ledger constraints
    for constraint in L:
        if not satisfiable_in_region(R, constraint):
            return UNSAT(witness=("LEDGER", constraint))

    return FEASIBLE"""
    elements.append(create_code_block(feas_code, styles))

    elements.append(Paragraph("Examples of UNSAT certificates:", styles['Op_Body']))
    elements.append(Paragraph("• Impossible distance restraints (mutually exclusive)", styles['Op_Bullet']))
    elements.append(Paragraph("• Steric impossibility (atoms forced to overlap)", styles['Op_Bullet']))
    elements.append(Paragraph("• Contradictory geometry constraints", styles['Op_Bullet']))

    elements.append(PageBreak())

    # 3.3 Algorithm Theorems
    elements.append(Paragraph("3.3 Algorithm Theorems", styles['Op_H2']))

    elements.append(create_theorem_box(
        "Theorem 5.1: Branch-and-Bound Soundness",
        "If B&B returns UNIQUE(X*, E*), then X* is a verified optimum within epsilon. Proof: UNIQUE is only returned when: (1) verify(X*) = PASS, (2) E(X*) = E*, (3) global_LB >= E* - epsilon (certified closure). By (3), no conformation can have energy more than epsilon better than X*. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Theorem 5.2: Branch-and-Bound Completeness",
        "If a feasible conformation exists, B&B will find it (given sufficient budget). Proof: The initial region R0 contains all admissible conformations. Splitting is complete (every point is in some leaf region). Feasibility oracle is sound. Therefore, if X exists in Omega(S) ∩ L, it will be sampled. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Theorem 5.3: Optimality Certificate",
        "When B&B returns UNIQUE with certificate (LB, UB), the solution is epsilon-optimal. Proof: The certificate states LB <= optimal_energy <= UB. Since UB = E(X_best) and LB >= UB - epsilon (closure condition), we have optimal_energy >= E(X_best) - epsilon. Therefore X_best is within epsilon of optimal. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Theorem 5.4: Branch-and-Bound Termination",
        "Given finite resolution delta on coordinates, B&B always terminates. Proof: Each split reduces region volume by at least factor 1/2. With minimum coordinate resolution delta, maximum tree depth is O(n * log(diam/delta)) where n is dimension and diam is initial diameter. Finite depth implies finite tree. QED.",
        styles
    ))

    elements.append(PageBreak())
    return elements

def create_section_6(styles):
    """4. Failure Modes and Bounded Outputs section."""
    elements = []

    elements.append(Paragraph("4. Failure Modes and Bounded Outputs", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("Omega represents what we do not know, structured precisely.", styles['Op_Body']))

    # UNSAT
    elements.append(Paragraph("UNSAT (Proven Infeasible)", styles['Op_H2']))
    elements.append(create_definition_box(
        "UNSAT Certificate",
        "No admissible conformation satisfies hard constraints + ledger.<br/>Certificate must be explicit:<br/>- Geometry violation: impossible bond/angle combination<br/>- Steric impossibility: forced overlap<br/>- Ledger contradiction: mutually exclusive constraints",
        styles
    ))

    unsat_code = """UNSAT = {
    status: "UNSAT",
    certificate: {
        type: "LEDGER_CONTRADICTION",
        constraints: [c1, c2, c3],
        reason: "Distance d(A,B) < 3A and d(A,B) > 5A cannot both hold"
    }
}"""
    elements.append(create_code_block(unsat_code, styles))

    # OMEGA_MULTI
    elements.append(Paragraph("OMEGA_MULTI (Degeneracy)", styles['Op_H2']))
    elements.append(create_definition_box(
        "OMEGA_MULTI",
        'Multiple optimal basins survive even with closed certificates.<br/>This is not undecided; it is the correct "many answers" truth.<br/>Returns all epsilon-optimal basins with their structures and energies.',
        styles
    ))

    multi_code = """OMEGA_MULTI = {
    status: "OMEGA_MULTI",
    basins: [
        {structure: X1, energy: E1, basin_id: "fold_A"},
        {structure: X2, energy: E2, basin_id: "fold_B"}
    ],
    certificate: "All basins within epsilon of global minimum",
    receipt: SHA256(canonical(basins))
}"""
    elements.append(create_code_block(multi_code, styles))

    # OMEGA_GAP
    elements.append(Paragraph("OMEGA_GAP (Undecided Under Budget)", styles['Op_H2']))
    elements.append(create_definition_box(
        "OMEGA_GAP",
        "Budget/time/memory limit hit before certificates close.<br/>Returns structured frontier: remaining regions, bounds, next split.",
        styles
    ))

    gap_code = """OMEGA_GAP = {
    status: "OMEGA_GAP",
    frontier: [
        {region: R1, LB: lb1, UB: ub1},
        {region: R2, LB: lb2, UB: ub2}
    ],
    global_LB: best_lower_bound,
    global_UB: best_upper_bound,
    best_X: current_best_structure,
    tau_star: {variable: "phi_42", split_point: -60.0}
}"""
    elements.append(create_code_block(gap_code, styles))

    elements.append(PageBreak())
    return elements

def create_section_7(styles):
    """5. Correctness and Guarantees section."""
    elements = []

    elements.append(Paragraph("5. Correctness and Guarantees", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("Summary of Guarantees", styles['Op_H2']))

    guarantee_data = [
        [Paragraph('<font color="#1BCDFF"><b>Property</b></font>', styles['Op_Body']),
         Paragraph('<font color="#1BCDFF"><b>Guarantee</b></font>', styles['Op_Body'])],
        [Paragraph('Soundness', styles['Op_Body']),
         Paragraph('Only verifier-pass structures returned', styles['Op_Body'])],
        [Paragraph('Completeness', styles['Op_Body']),
         Paragraph('All admissible solutions reachable', styles['Op_Body'])],
        [Paragraph('Optimality', styles['Op_Body']),
         Paragraph('Certified epsilon-optimality via LB/UB closure', styles['Op_Body'])],
        [Paragraph('Honest Omega', styles['Op_Body']),
         Paragraph('UNSAT / OMEGA_MULTI / OMEGA_GAP never conflated', styles['Op_Body'])],
        [Paragraph('Verifiable', styles['Op_Body']),
         Paragraph('Any output replayable with receipts', styles['Op_Body'])],
        [Paragraph('Deterministic', styles['Op_Body']),
         Paragraph('Same inputs always produce same outputs', styles['Op_Body'])],
    ]
    guarantee_table = Table(guarantee_data, colWidths=[4*cm, 11*cm])
    guarantee_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), BRAND['code_bg']),
        ('BACKGROUND', (0, 1), (-1, -1), colors.Color(0.05, 0.05, 0.05)),
        ('GRID', (0, 0), (-1, -1), 0.5, BRAND['border']),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('LEFTPADDING', (0, 0), (-1, -1), 10),
        ('RIGHTPADDING', (0, 0), (-1, -1), 10),
        ('TOPPADDING', (0, 0), (-1, -1), 8),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 8),
    ]))
    elements.append(guarantee_table)
    elements.append(Spacer(1, 0.5*cm))

    elements.append(Paragraph("Formal Properties", styles['Op_H2']))

    elements.append(create_theorem_box(
        "Property: No False Positives",
        "The system never claims a structure is optimal when it is not. Proof: UNIQUE requires certified closure (LB >= UB - epsilon). Until closure is achieved, the system returns OMEGA_GAP. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Property: No False Negatives",
        "The system never claims UNSAT when a feasible solution exists. Proof: UNSAT requires explicit certificate (geometry/steric/ledger impossibility). Without such proof, the system returns OMEGA_GAP. QED.",
        styles
    ))

    elements.append(create_theorem_box(
        "Property: Honest Degeneracy",
        "When multiple equally-good solutions exist, the system reports all of them. Proof: OMEGA_MULTI is returned when multiple basins survive within epsilon. The system does not arbitrarily pick one. QED.",
        styles
    ))

    elements.append(create_insight_box(
        "Key Point",
        "These guarantees hold because the verifier is the source of truth, not any particular solver. The solver proposes, the verifier disposes. Certificates prove claims; absence of certificate means OMEGA.",
        styles
    ))

    elements.append(PageBreak())
    return elements

def create_section_8(styles):
    """6. Implementation Playbook section."""
    elements = []

    elements.append(Paragraph("6. Implementation Playbook", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("Follow these steps in order for a correct implementation.", styles['Op_Body']))

    elements.append(Paragraph("Step 0: Pin the Contract", styles['Op_H3']))
    elements.append(Paragraph("• Version the energy function E(X) with exact specification", styles['Op_Bullet']))
    elements.append(Paragraph("• Version all tolerances (bond lengths, angles, steric radii)", styles['Op_Bullet']))
    elements.append(Paragraph("• Version the ledger schema (restraint types, tolerance semantics)", styles['Op_Bullet']))
    elements.append(Paragraph("• Document coordinate system and units (Angstroms, degrees)", styles['Op_Bullet']))

    elements.append(Paragraph("Step 1: Implement the Verifier First", styles['Op_H3']))
    elements.append(Paragraph("• GEOM check: bond lengths, angles, chirality, planarity", styles['Op_Bullet']))
    elements.append(Paragraph("• STERIC check: van der Waals exclusion", styles['Op_Bullet']))
    elements.append(Paragraph("• LEDGER check: evidence constraint satisfaction", styles['Op_Bullet']))
    elements.append(Paragraph("• Return minimal witness on FAIL", styles['Op_Bullet']))
    elements.append(Paragraph("• Test verifier independently before any solver code", styles['Op_Bullet']))

    elements.append(Paragraph("Step 2: Implement Canonicalization (Pi)", styles['Op_H3']))
    elements.append(Paragraph("• Normalize rigid motions: center at centroid, align principal axes", styles['Op_Bullet']))
    elements.append(Paragraph("• Canonicalize internal coordinate representation", styles['Op_Bullet']))
    elements.append(Paragraph("• Canonicalize evidence ordering and units", styles['Op_Bullet']))
    elements.append(Paragraph("• Ensure deterministic output for equivalent inputs", styles['Op_Bullet']))

    elements.append(Paragraph("Step 3: Implement Bound(R)", styles['Op_H3']))
    elements.append(Paragraph("• Interval bounds on internal coordinates", styles['Op_Bullet']))
    elements.append(Paragraph("• Convex relaxations for energy terms", styles['Op_Bullet']))
    elements.append(Paragraph("• Certified steric pruning", styles['Op_Bullet']))
    elements.append(Paragraph("• Must guarantee: LB(R) <= inf_{X in R} E(X) <= UB(R)", styles['Op_Bullet']))

    elements.append(Paragraph("Step 4: Implement Region Splitting tau*", styles['Op_H3']))
    elements.append(Paragraph("• Deterministic choice of variable + split point", styles['Op_Bullet']))
    elements.append(Paragraph("• Maximize expected bound tightening", styles['Op_Bullet']))
    elements.append(Paragraph("• Pi-invariant tie-break (canonical ordering)", styles['Op_Bullet']))
    elements.append(Paragraph("• Document splitting strategy precisely", styles['Op_Bullet']))

    elements.append(PageBreak())

    elements.append(Paragraph("Step 5: Implement Branch-and-Bound Engine", styles['Op_H3']))
    elements.append(Paragraph("• Priority queue ordered by lower bound", styles['Op_Bullet']))
    elements.append(Paragraph("• Prune regions where LB >= UB_best - epsilon", styles['Op_Bullet']))
    elements.append(Paragraph("• Track best feasible point found", styles['Op_Bullet']))
    elements.append(Paragraph("• Terminate on closure or budget exhaustion", styles['Op_Bullet']))

    elements.append(Paragraph("Step 6: Implement Omega Objects", styles['Op_H3']))
    elements.append(Paragraph("• UNSAT: explicit certificate of infeasibility", styles['Op_Bullet']))
    elements.append(Paragraph("• OMEGA_MULTI: list of epsilon-optimal basins", styles['Op_Bullet']))
    elements.append(Paragraph("• OMEGA_GAP: frontier regions + bounds + tau*", styles['Op_Bullet']))
    elements.append(Paragraph('• Never return bare "failed" without explanation', styles['Op_Bullet']))

    elements.append(Paragraph("Step 7: Implement Receipts", styles['Op_H3']))
    elements.append(Paragraph("Every solution generates a deterministic, implementation-independent receipt:", styles['Op_Body']))

    receipt_code = """def generate_receipt(structure, energy, sequence):
    witness = {
        "sequence": sequence,
        "coordinates": canonicalize(structure),
        "energy": energy,
        "verifier": "PASS"
    }
    # NO timestamps in hashed payload!
    canonical = json.dumps(witness, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()"""
    elements.append(create_code_block(receipt_code, styles))

    elements.append(Paragraph("Step 8: Self-Improvement (Lemmas)", styles['Op_H3']))
    elements.append(Paragraph("• Store pruning lemmas: (motif pattern, steric certificate)", styles['Op_Bullet']))
    elements.append(Paragraph("• Store constraint implications discovered during search", styles['Op_Bullet']))
    elements.append(Paragraph("• Store basin signatures for common folds", styles['Op_Bullet']))
    elements.append(Paragraph("• Reapply as derived tests in future runs", styles['Op_Bullet']))
    elements.append(Paragraph("• System gets faster over time on similar sequences", styles['Op_Bullet']))

    elements.append(PageBreak())
    return elements

def create_section_9(styles):
    """7. Optional Extensions section."""
    elements = []

    elements.append(Paragraph("7. Optional Extensions", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("Complexes and Multimers", styles['Op_H2']))
    elements.append(Paragraph("Extend to multi-chain systems:", styles['Op_Body']))
    elements.append(Paragraph("• Model as coupled conformation spaces: X = (X_A, X_B, ...)", styles['Op_Bullet']))
    elements.append(Paragraph("• Add inter-chain steric constraints", styles['Op_Bullet']))
    elements.append(Paragraph("• Add interface constraints from evidence (cross-links, contacts)", styles['Op_Bullet']))
    elements.append(Paragraph("• Same B&B framework with expanded constraint set", styles['Op_Bullet']))

    elements.append(Paragraph("Kinetics and Folding Pathways", styles['Op_H2']))
    elements.append(Paragraph("Extend to path-space inference:", styles['Op_Body']))
    elements.append(Paragraph("• Model as sequence of conformations: P = (X_0, X_1, ..., X_T)", styles['Op_Bullet']))
    elements.append(Paragraph("• Add transition constraints (local moves, energy barriers)", styles['Op_Bullet']))
    elements.append(Paragraph("• Add barrier certificates (saddle point bounds)", styles['Op_Bullet']))
    elements.append(Paragraph("• Enables folding pathway inference, not just endpoint", styles['Op_Bullet']))

    elements.append(Paragraph("Active Learning for Experimental Design", styles['Op_H2']))
    elements.append(Paragraph("Use tau* to guide experiments:", styles['Op_Body']))
    elements.append(Paragraph("• tau* identifies the test that would most reduce uncertainty", styles['Op_Bullet']))
    elements.append(Paragraph("• Translate to experimental measurement (NMR restraint, cross-link)", styles['Op_Bullet']))
    elements.append(Paragraph("• Enables optimal experimental design under budget", styles['Op_Bullet']))
    elements.append(Paragraph("• Each experiment collapses Omega frontier maximally", styles['Op_Bullet']))

    elements.append(Paragraph("Cross-Check Solvers", styles['Op_H2']))
    elements.append(Paragraph("Optional independent verification:", styles['Op_Body']))
    elements.append(Paragraph("• Discretized coarse-grain ILP/CP-SAT model", styles['Op_Bullet']))
    elements.append(Paragraph("• Lattice protein models for sanity checks", styles['Op_Bullet']))
    elements.append(Paragraph("• Multiple energy function implementations", styles['Op_Bullet']))
    elements.append(Paragraph("• Cross-validation strengthens receipts", styles['Op_Bullet']))

    elements.append(PageBreak())
    return elements

def create_appendix_a(styles):
    """Appendix A: Sample Verification Output."""
    elements = []

    elements.append(Paragraph("Appendix A: Sample Verification Output", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("PASS Example", styles['Op_H2']))
    pass_code = """Input: Structure X for sequence "ACDEFGHIK"

Verification Results:
  V1 Geometry: PASS
    - All 8 peptide bonds within 0.02A of 1.33A
    - All 16 backbone angles within 3 degrees of ideal
    - All chirality centers correct (L-amino acids)

  V2 Sterics: PASS
    - No atom pairs closer than sum of vdW radii - 0.4A
    - Checked 435 non-bonded pairs

  V3 Ledger: PASS
    - NOE restraint d(Ala1.CA, Lys9.CA) = 8.2A (limit: <10A)
    - All 12 distance restraints satisfied

  V4 Objective: E(X) = -142.3 kcal/mol

  V5 Certificate: PASS
    - Global LB = -145.1 kcal/mol
    - Gap = 2.8 kcal/mol < epsilon = 5.0 kcal/mol

Final: PASS
Receipt: a3f8c2d1e9b7..."""
    elements.append(create_code_block(pass_code, styles, keep_together=False))

    elements.append(Paragraph("FAIL Example", styles['Op_H2']))
    fail_code = """Input: Structure X for sequence "ACDEFGHIK"

Verification Results:
  V1 Geometry: PASS
  V2 Sterics: FAIL
    - CLASH detected: Phe5.CE1 -- Ile8.CD1
    - Distance: 2.1A
    - Minimum allowed: 3.0A (vdW sum - tolerance)

Final: FAIL
Witness: ("CLASH", "Phe5.CE1", "Ile8.CD1", 2.1, 3.0)"""
    elements.append(create_code_block(fail_code, styles))

    elements.append(PageBreak())
    return elements

def create_appendix_b(styles):
    """Appendix B: Reference Implementation Skeleton."""
    elements = []

    elements.append(Paragraph("Appendix B: Reference Implementation Skeleton", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    impl_code1 = """# alphaprotein_kernel.py - Reference Implementation Skeleton

import hashlib
import json
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum

class Status(Enum):
    UNIQUE = "UNIQUE"
    OMEGA_MULTI = "OMEGA_MULTI"
    OMEGA_GAP = "OMEGA_GAP"
    UNSAT = "UNSAT"

@dataclass
class Conformation:
    \"\"\"3D structure representation.\"\"\"
    sequence: str
    coordinates: List[Tuple[float, float, float]]  # Per-atom coords

    def canonicalize(self) -> 'Conformation':
        \"\"\"Center at centroid, align principal axes.\"\"\"
        # Implementation: SVD alignment
        pass

@dataclass
class Region:
    \"\"\"Bounded region in conformation space.\"\"\"
    phi_bounds: List[Tuple[float, float]]
    psi_bounds: List[Tuple[float, float]]
    chi_bounds: List[List[Tuple[float, float]]]

    def split(self, variable: str, point: float) -> Tuple['Region', 'Region']:
        \"\"\"Split region at given variable and point.\"\"\"
        pass

@dataclass
class VerifierResult:
    status: str  # "PASS" or "FAIL"
    witness: Optional[Any] = None
    energy: Optional[float] = None"""
    elements.append(create_code_block(impl_code1, styles, keep_together=False))

    elements.append(PageBreak())

    impl_code2 = """class Verifier:
    \"\"\"Truth gate - source of all validity.\"\"\"

    def __init__(self, tolerances: Dict[str, float]):
        self.tol = tolerances

    def verify(self, X: Conformation, ledger: List[Any]) -> VerifierResult:
        # V1: Geometry
        geom = self._check_geometry(X)
        if geom.status == "FAIL":
            return geom

        # V2: Sterics
        steric = self._check_sterics(X)
        if steric.status == "FAIL":
            return steric

        # V3: Ledger
        ledger_result = self._check_ledger(X, ledger)
        if ledger_result.status == "FAIL":
            return ledger_result

        # V4: Objective
        energy = self._compute_energy(X)

        return VerifierResult(status="PASS", energy=energy)


class BranchAndBound:
    \"\"\"Certified global optimizer.\"\"\"

    def solve(self, sequence: str, ledger: List, epsilon: float,
              max_nodes: int) -> Dict:
        R0 = self._initial_region(sequence)
        queue = [(self._bound(R0)[0], R0)]
        X_best, UB_best = None, float('inf')
        nodes = 0

        while queue and nodes < max_nodes:
            LB_R, R = heapq.heappop(queue)
            nodes += 1

            if LB_R >= UB_best - epsilon:
                continue

            # ... (rest of B&B logic)

        # Return appropriate status
        pass"""
    elements.append(create_code_block(impl_code2, styles, keep_together=False))

    elements.append(PageBreak())
    return elements

def create_appendix_c(styles):
    """Appendix C: Production Checklist."""
    elements = []

    elements.append(Paragraph("Appendix C: Production Checklist", styles['Op_H1']))
    elements.append(HRFlowable(width="100%", thickness=2, color=BRAND['cyan'], spaceAfter=15))

    elements.append(Paragraph("Pre-Deployment Checklist", styles['Op_H2']))
    elements.append(Paragraph("• Energy function E(X) pinned with version hash", styles['Op_Bullet']))
    elements.append(Paragraph("• All tolerances documented and versioned", styles['Op_Bullet']))
    elements.append(Paragraph("• Verifier passes independent test suite", styles['Op_Bullet']))
    elements.append(Paragraph("• Canonicalization produces deterministic output", styles['Op_Bullet']))
    elements.append(Paragraph("• Bound(R) provably correct (LB <= true <= UB)", styles['Op_Bullet']))
    elements.append(Paragraph("• Receipt generation is timestamp-free", styles['Op_Bullet']))
    elements.append(Paragraph("• All four output states implemented (UNIQUE/OMEGA_MULTI/OMEGA_GAP/UNSAT)", styles['Op_Bullet']))

    elements.append(Paragraph("Monitoring Checklist", styles['Op_H2']))
    elements.append(Paragraph("• Track solve time distribution", styles['Op_Bullet']))
    elements.append(Paragraph("• Track node count distribution", styles['Op_Bullet']))
    elements.append(Paragraph("• Track certificate gap at termination", styles['Op_Bullet']))
    elements.append(Paragraph("• Track OMEGA_GAP frequency (indicates budget issues)", styles['Op_Bullet']))
    elements.append(Paragraph("• Track OMEGA_MULTI frequency (indicates model degeneracy)", styles['Op_Bullet']))
    elements.append(Paragraph("• Log all receipts for audit trail", styles['Op_Bullet']))

    elements.append(Paragraph("Safety Checklist", styles['Op_H2']))
    elements.append(Paragraph("• Never claim UNIQUE without certificate closure", styles['Op_Bullet']))
    elements.append(Paragraph("• Never claim UNSAT without explicit proof", styles['Op_Bullet']))
    elements.append(Paragraph("• Always return structured OMEGA on budget exhaustion", styles['Op_Bullet']))
    elements.append(Paragraph("• Validate all inputs before processing", styles['Op_Bullet']))
    elements.append(Paragraph("• Bound memory usage to prevent OOM", styles['Op_Bullet']))
    elements.append(Paragraph("• Implement graceful timeout handling", styles['Op_Bullet']))

    elements.append(Paragraph("Test Suite Requirements", styles['Op_H2']))
    test_data = [
        [Paragraph('<font color="#1BCDFF"><b>Test</b></font>', styles['Op_Body']),
         Paragraph('<font color="#1BCDFF"><b>Description</b></font>', styles['Op_Body']),
         Paragraph('<font color="#1BCDFF"><b>Expected</b></font>', styles['Op_Body'])],
        [Paragraph('Small peptide', styles['Op_Body']),
         Paragraph('Ala-Gly-Ala trivial', styles['Op_Body']),
         Paragraph('UNIQUE', styles['Op_Body'])],
        [Paragraph('Constrained', styles['Op_Body']),
         Paragraph('With NOE restraints', styles['Op_Body']),
         Paragraph('UNIQUE', styles['Op_Body'])],
        [Paragraph('Degenerate', styles['Op_Body']),
         Paragraph('Symmetric sequence', styles['Op_Body']),
         Paragraph('OMEGA_MULTI', styles['Op_Body'])],
        [Paragraph('Infeasible', styles['Op_Body']),
         Paragraph('Contradictory restraints', styles['Op_Body']),
         Paragraph('UNSAT', styles['Op_Body'])],
        [Paragraph('Large', styles['Op_Body']),
         Paragraph('Budget exhaustion', styles['Op_Body']),
         Paragraph('OMEGA_GAP', styles['Op_Body'])],
    ]
    test_table = Table(test_data, colWidths=[4*cm, 6*cm, 5*cm])
    test_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), BRAND['code_bg']),
        ('BACKGROUND', (0, 1), (-1, -1), colors.Color(0.05, 0.05, 0.05)),
        ('GRID', (0, 0), (-1, -1), 0.5, BRAND['border']),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
    ]))
    elements.append(test_table)

    elements.append(PageBreak())
    return elements

def create_back_page(styles):
    """Create back/closing page."""
    elements = []

    elements.append(Spacer(1, 6*cm))

    elements.append(Paragraph(
        "ALPHAPROTEIN KERNEL SPEC",
        ParagraphStyle('back_title', parent=styles['Title_Custom'], fontSize=24)
    ))
    elements.append(Spacer(1, 0.5*cm))
    elements.append(Paragraph(
        "Protein Folding as Quotient-Collapse",
        ParagraphStyle('back_subtitle', parent=styles['Op_Body'], fontSize=14, textColor=BRAND['text_60'], alignment=TA_CENTER)
    ))
    elements.append(Spacer(1, 1*cm))
    elements.append(Paragraph(
        '"If I speak, I have proof. If I cannot prove, I return the exact boundary."',
        ParagraphStyle('back_quote', parent=styles['Op_Body'], fontSize=11, textColor=BRAND['text_60'], alignment=TA_CENTER, fontName='Helvetica-Oblique')
    ))
    elements.append(Spacer(1, 1.5*cm))
    elements.append(Paragraph(
        "This specification resolves protein structure inference for the pinned model.",
        ParagraphStyle('back_desc', parent=styles['Op_Body'], fontSize=11, textColor=BRAND['text_60'], alignment=TA_CENTER, fontName='Helvetica-Oblique')
    ))
    elements.append(Paragraph(
        "Sound. Complete. Certified. Honest.",
        ParagraphStyle('back_props', parent=styles['Op_Body'], fontSize=11, textColor=BRAND['text_60'], alignment=TA_CENTER, fontName='Helvetica-Oblique')
    ))
    elements.append(Spacer(1, 3*cm))
    elements.append(Paragraph("OPOCH", ParagraphStyle('opoch_back', parent=styles['Title_Custom'], fontSize=20)))
    elements.append(Paragraph("www.opoch.com", ParagraphStyle('url_back', parent=styles['Op_Muted'])))

    return elements

def generate_pdf():
    """Generate the complete AlphaProtein Kernel Spec PDF."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(script_dir, "ALPHAPROTEIN_KERNEL_SPEC_v1.pdf")

    doc = SimpleDocTemplate(
        output_file,
        pagesize=A4,
        leftMargin=2*cm,
        rightMargin=2*cm,
        topMargin=2*cm,
        bottomMargin=2*cm,
    )

    styles = create_styles()
    elements = []

    # Cover page (no header)
    elements.extend(create_cover_page(styles))

    # Index
    elements.extend(create_index(styles))

    # Executive Summary (new section 0)
    elements.extend(create_executive_summary(styles))

    # Main sections
    elements.extend(create_section_1(styles))
    elements.extend(create_section_2(styles))
    elements.extend(create_section_3(styles))
    elements.extend(create_section_4(styles))
    elements.extend(create_section_5(styles))
    elements.extend(create_section_6(styles))
    elements.extend(create_section_7(styles))
    elements.extend(create_section_8(styles))
    elements.extend(create_section_9(styles))

    # Appendices
    elements.extend(create_appendix_a(styles))
    elements.extend(create_appendix_b(styles))
    elements.extend(create_appendix_c(styles))

    # Build PDF - use header_footer for ALL pages (including cover) like MAPF
    doc.build(elements, onFirstPage=header_footer, onLaterPages=header_footer)
    print(f"PDF generated: {output_file}")

if __name__ == "__main__":
    generate_pdf()
