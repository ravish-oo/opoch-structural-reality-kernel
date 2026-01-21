import { useMemo, memo } from 'react'
import { parseMarkdown, type MarkdownOptions } from '../lib/markdown'

interface MDXContentProps {
  content: string
  className?: string
  options?: MarkdownOptions
}

const MDXContent = memo(function MDXContent({ 
  content, 
  className = '',
  options = {}
}: MDXContentProps) {
  // Safely parse markdown with custom styling
  const html = useMemo(() => {
    const defaultClasses = {
      h1: 'text-3xl font-bold mb-6 text-white',
      h2: 'text-2xl font-semibold mb-4 mt-8 text-white', 
      h3: 'text-xl font-semibold mb-3 mt-6 text-white',
      p: 'mb-4 text-white/80 leading-relaxed',
      ul: 'mb-4 ml-6 list-disc text-white/80',
      ol: 'mb-4 ml-6 list-decimal text-white/80',
      a: 'text-cyan-400 hover:text-cyan-300 underline transition-colors',
      blockquote: 'border-l-4 border-cyan-500/50 pl-4 my-4 text-white/70 italic',
      code: 'bg-white/10 px-1.5 py-0.5 rounded text-sm font-mono',
      pre: 'bg-white/5 border border-white/10 rounded-lg p-4 overflow-x-auto my-4'
    }
    
    return parseMarkdown(content, {
      ...options,
      classes: { ...defaultClasses, ...options.classes },
      linkRenderer: options.linkRenderer || ((href, text) => 
        `<a href="${href}" target="_blank" rel="noopener noreferrer" class="${defaultClasses.a}">${text}</a>`
      )
    })
  }, [content, options])

  return (
    <div 
      className={`mdx-content prose prose-invert prose-lg max-w-none ${className}`}
      dangerouslySetInnerHTML={{ __html: html }}
      role="article"
      aria-label="Article content"
    />
  )
})

export default MDXContent