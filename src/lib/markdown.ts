import DOMPurify from 'isomorphic-dompurify'
import { marked } from 'marked'

// Configure marked options for security and consistency
marked.setOptions({
  gfm: true,
  breaks: true,
  // sanitize option removed in newer versions - we use DOMPurify instead
})

// Configure DOMPurify for safe HTML rendering
const purifyConfig = {
  ALLOWED_TAGS: [
    'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
    'p', 'br', 'hr',
    'ul', 'ol', 'li',
    'strong', 'em', 'b', 'i', 'u',
    'a', 'code', 'pre', 'blockquote',
    'table', 'thead', 'tbody', 'tr', 'th', 'td',
    'img', 'figure', 'figcaption'
  ],
  ALLOWED_ATTR: [
    'href', 'target', 'rel', 'title',
    'src', 'alt', 'width', 'height',
    'class', 'id'
  ],
  ALLOWED_URI_REGEXP: /^(?:(?:https?|mailto|tel):|[^a-z]|[a-z+.\-]+(?:[^a-z+.\-:]|$))/i,
  ADD_ATTR: ['target', 'rel'],
  ADD_TAGS: ['iframe'],
  FORBID_TAGS: ['script', 'style'],
  FORBID_ATTR: ['onerror', 'onload', 'onclick']
}

export interface MarkdownOptions {
  /**
   * Whether to allow HTML in markdown
   */
  allowHtml?: boolean
  /**
   * Custom link renderer
   */
  linkRenderer?: (href: string, text: string) => string
  /**
   * CSS classes to add to rendered elements
   */
  classes?: {
    h1?: string
    h2?: string
    h3?: string
    p?: string
    a?: string
    ul?: string
    ol?: string
    blockquote?: string
    code?: string
    pre?: string
  }
}

/**
 * Safely parse and render markdown to HTML
 */
export function parseMarkdown(content: string, options: MarkdownOptions = {}): string {
  const { allowHtml = false, linkRenderer, classes } = options

  // Parse markdown to HTML
  let html = marked.parse(content, { async: false }) as string

  // Apply custom link rendering
  if (linkRenderer) {
    html = html.replace(/<a\s+(?:[^>]*?\s+)?href="([^"]*)"[^>]*>([^<]*)<\/a>/g, 
      (_match, href, text) => linkRenderer(href, text))
  }

  // Apply CSS classes if provided
  if (classes) {
    Object.entries(classes).forEach(([tag, className]) => {
      if (className) {
        const regex = new RegExp(`<${tag}(?![a-z])([^>]*)>`, 'gi')
        html = html.replace(regex, (match, attrs) => {
          if (attrs.includes('class=')) {
            return match.replace(/class="([^"]*)"/, `class="$1 ${className}"`)
          }
          return `<${tag}${attrs} class="${className}">`
        })
      }
    })
  }

  // Sanitize HTML
  const config = allowHtml ? purifyConfig : { ...purifyConfig, ALLOWED_TAGS: purifyConfig.ALLOWED_TAGS.filter(tag => tag !== 'iframe') }
  return DOMPurify.sanitize(html, config)
}

/**
 * Extract plain text from markdown
 */
export function markdownToPlainText(markdown: string): string {
  // Remove markdown syntax
  return markdown
    // Remove headers
    .replace(/^#{1,6}\s+/gm, '')
    // Remove emphasis
    .replace(/(\*\*|__)(.*?)\1/g, '$2')
    .replace(/(\*|_)(.*?)\1/g, '$2')
    // Remove links
    .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1')
    // Remove images
    .replace(/!\[([^\]]*)\]\([^)]+\)/g, '')
    // Remove code blocks
    .replace(/```[\s\S]*?```/g, '')
    .replace(/`([^`]+)`/g, '$1')
    // Remove blockquotes
    .replace(/^\s*>\s+/gm, '')
    // Remove lists
    .replace(/^\s*[-*+]\s+/gm, '')
    .replace(/^\s*\d+\.\s+/gm, '')
    // Clean up extra whitespace
    .replace(/\n{3,}/g, '\n\n')
    .trim()
}

/**
 * Get estimated reading time for markdown content
 */
export function getReadingTime(markdown: string, wordsPerMinute = 200): number {
  const plainText = markdownToPlainText(markdown)
  const wordCount = plainText.split(/\s+/).filter(word => word.length > 0).length
  return Math.ceil(wordCount / wordsPerMinute)
}