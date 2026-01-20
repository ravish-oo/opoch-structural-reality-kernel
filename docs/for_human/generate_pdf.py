#!/usr/bin/env python3
"""
Generate styled PDF from restructured MAPF Kernel Spec markdown.
Uses WeasyPrint for markdown-to-PDF with CSS styling.

Install dependencies:
    pip install weasyprint markdown pymdown-extensions

Usage:
    python generate_pdf.py
"""

import markdown
from weasyprint import HTML, CSS
import os

# Brand colors extracted from original PDF
BRAND_COLORS = {
    'background': '#000000',
    'header_cyan': '#1BCDFF',
    'definition_yellow': '#FFD700',
    'theorem_purple': '#B19CD9',
    'text_white': '#FFFFFF',
    'text_muted': '#AAAAAA',
    'code_bg': '#0A1929',
    'border': '#1A3A4A',
}

CSS_STYLES = f"""
@page {{
    size: A4;
    margin: 2cm;
    background-color: {BRAND_COLORS['background']};
    @top-left {{
        content: "OPOCH";
        color: {BRAND_COLORS['header_cyan']};
        font-family: 'Helvetica Neue', Arial, sans-serif;
        font-weight: bold;
        font-size: 10pt;
    }}
    @top-right {{
        content: "Page " counter(page);
        color: {BRAND_COLORS['text_muted']};
        font-size: 9pt;
    }}
    @bottom-center {{
        content: "Confidential - Opoch Research";
        color: {BRAND_COLORS['text_muted']};
        font-size: 8pt;
        font-style: italic;
    }}
}}

body {{
    font-family: 'Helvetica Neue', Arial, sans-serif;
    font-size: 10pt;
    line-height: 1.5;
    color: {BRAND_COLORS['text_white']};
    background-color: {BRAND_COLORS['background']};
}}

/* Headers */
h1 {{
    color: {BRAND_COLORS['header_cyan']};
    font-size: 24pt;
    font-weight: bold;
    margin-top: 30pt;
    margin-bottom: 15pt;
    padding-bottom: 8pt;
    border-bottom: 2px solid {BRAND_COLORS['header_cyan']};
    page-break-after: avoid;
}}

h2 {{
    color: {BRAND_COLORS['header_cyan']};
    font-size: 18pt;
    font-weight: bold;
    margin-top: 25pt;
    margin-bottom: 12pt;
    page-break-after: avoid;
}}

h3 {{
    color: {BRAND_COLORS['header_cyan']};
    font-size: 14pt;
    font-weight: bold;
    margin-top: 20pt;
    margin-bottom: 10pt;
    page-break-after: avoid;
}}

h4 {{
    color: {BRAND_COLORS['header_cyan']};
    font-size: 12pt;
    font-weight: bold;
    margin-top: 15pt;
    margin-bottom: 8pt;
}}

/* Paragraphs and text */
p {{
    margin-bottom: 10pt;
    text-align: justify;
}}

/* Lists */
ul, ol {{
    margin-left: 20pt;
    margin-bottom: 10pt;
}}

li {{
    margin-bottom: 5pt;
}}

/* Code blocks */
code {{
    font-family: 'Courier New', Consolas, monospace;
    font-size: 9pt;
    background-color: {BRAND_COLORS['code_bg']};
    padding: 2pt 4pt;
    border-radius: 3pt;
    color: {BRAND_COLORS['header_cyan']};
}}

pre {{
    background-color: {BRAND_COLORS['code_bg']};
    padding: 15pt;
    border-radius: 5pt;
    border-left: 3px solid {BRAND_COLORS['header_cyan']};
    margin: 15pt 0;
    overflow-x: auto;
    page-break-inside: avoid;
}}

pre code {{
    background-color: transparent;
    padding: 0;
    font-size: 8pt;
    line-height: 1.4;
    color: {BRAND_COLORS['header_cyan']};
}}

/* Tables */
table {{
    width: 100%;
    border-collapse: collapse;
    margin: 15pt 0;
    page-break-inside: avoid;
}}

th {{
    background-color: {BRAND_COLORS['code_bg']};
    color: {BRAND_COLORS['header_cyan']};
    font-weight: bold;
    padding: 10pt;
    border: 1px solid {BRAND_COLORS['border']};
    text-align: left;
}}

td {{
    padding: 8pt 10pt;
    border: 1px solid {BRAND_COLORS['border']};
    color: {BRAND_COLORS['text_white']};
}}

tr:nth-child(even) {{
    background-color: rgba(26, 58, 74, 0.3);
}}

/* Blockquotes - used for definitions and theorems */
blockquote {{
    background-color: {BRAND_COLORS['code_bg']};
    border-left: 4px solid {BRAND_COLORS['definition_yellow']};
    margin: 15pt 0;
    padding: 15pt 20pt;
    border-radius: 0 5pt 5pt 0;
    page-break-inside: avoid;
}}

blockquote p {{
    margin: 0;
    color: {BRAND_COLORS['text_white']};
}}

/* Strong text - for definitions */
strong {{
    color: {BRAND_COLORS['definition_yellow']};
    font-weight: bold;
}}

/* Emphasis */
em {{
    color: {BRAND_COLORS['theorem_purple']};
    font-style: italic;
}}

/* Horizontal rules */
hr {{
    border: none;
    border-top: 1px solid {BRAND_COLORS['border']};
    margin: 30pt 0;
}}

/* Links */
a {{
    color: {BRAND_COLORS['header_cyan']};
    text-decoration: none;
}}

/* Cover page styling */
.cover {{
    text-align: center;
    padding-top: 150pt;
}}

.cover h1 {{
    font-size: 36pt;
    border: none;
    margin-bottom: 10pt;
}}

.cover h2 {{
    color: {BRAND_COLORS['header_cyan']};
    font-size: 20pt;
    font-weight: normal;
}}

.cover .subtitle {{
    color: {BRAND_COLORS['text_muted']};
    font-size: 12pt;
    margin-top: 20pt;
}}

/* Contract and verified boxes */
.contract-box {{
    background-color: {BRAND_COLORS['code_bg']};
    border: 1px solid {BRAND_COLORS['border']};
    padding: 20pt;
    margin: 20pt 0;
    border-radius: 5pt;
}}

.contract-box h4 {{
    color: {BRAND_COLORS['definition_yellow']};
    margin-top: 0;
}}

/* Key insight boxes */
.insight-box {{
    background-color: rgba(27, 205, 255, 0.1);
    border: 1px solid {BRAND_COLORS['header_cyan']};
    padding: 15pt;
    margin: 15pt 0;
    border-radius: 5pt;
}}

/* Theorem styling */
.theorem {{
    background-color: {BRAND_COLORS['code_bg']};
    border-left: 4px solid {BRAND_COLORS['theorem_purple']};
    padding: 15pt 20pt;
    margin: 15pt 0;
    border-radius: 0 5pt 5pt 0;
}}

.theorem-title {{
    color: {BRAND_COLORS['theorem_purple']};
    font-weight: bold;
    font-style: italic;
}}

/* Page breaks */
.page-break {{
    page-break-before: always;
}}

/* Executive summary special styling */
.executive-summary {{
    background-color: rgba(27, 205, 255, 0.05);
    padding: 20pt;
    border-radius: 10pt;
    margin: 20pt 0;
}}
"""

def read_markdown_file(filepath):
    """Read markdown content from file."""
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()

def convert_markdown_to_html(md_content):
    """Convert markdown to HTML with extensions."""
    extensions = [
        'tables',
        'fenced_code',
        'codehilite',
        'toc',
        'nl2br',
    ]
    html_content = markdown.markdown(md_content, extensions=extensions)

    # Wrap in HTML document structure
    html_doc = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="UTF-8">
        <title>MAPF Kernel Spec v3</title>
    </head>
    <body>
        {html_content}
    </body>
    </html>
    """
    return html_doc

def generate_pdf(input_md, output_pdf):
    """Generate styled PDF from markdown file."""
    print(f"Reading markdown from: {input_md}")
    md_content = read_markdown_file(input_md)

    print("Converting markdown to HTML...")
    html_content = convert_markdown_to_html(md_content)

    print(f"Generating PDF: {output_pdf}")
    html = HTML(string=html_content)
    css = CSS(string=CSS_STYLES)

    html.write_pdf(output_pdf, stylesheets=[css])
    print(f"PDF generated successfully: {output_pdf}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, "MAPF_KERNEL_SPEC_v3_RESTRUCTURED.md")
    output_file = os.path.join(script_dir, "MAPF_KERNEL_SPEC_v3_RESTRUCTURED.pdf")

    if not os.path.exists(input_file):
        print(f"Error: Input file not found: {input_file}")
        return

    generate_pdf(input_file, output_file)

if __name__ == "__main__":
    main()
