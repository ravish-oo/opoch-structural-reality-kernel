import { useMemo } from 'react'

interface MDXContentProps {
  content: string
}

export default function MDXContent({ content }: MDXContentProps) {
  // For now, we'll render markdown as HTML
  // In a full implementation, we'd use mdx-bundler to compile MDX
  const Component = useMemo(() => {
    // Simple markdown to HTML conversion for demo
    const html = content
      .replace(/^### (.*$)/gim, '<h3>$1</h3>')
      .replace(/^## (.*$)/gim, '<h2>$1</h2>')
      .replace(/^# (.*$)/gim, '<h1>$1</h1>')
      .replace(/^\* (.*$)/gim, '<li>$1</li>')
      .replace(/\*\*(.*)\*\*/g, '<strong>$1</strong>')
      .replace(/\*(.*)\*/g, '<em>$1</em>')
      .replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2">$1</a>')
      .replace(/\n\n/g, '</p><p>')
      .replace(/^/, '<p>')
      .replace(/$/, '</p>')
      .replace(/<li>(.*)<\/li>/g, '<ul><li>$1</li></ul>')
      .replace(/<\/ul>\s*<ul>/g, '')
    
    return () => <div dangerouslySetInnerHTML={{ __html: html }} />
  }, [content])

  return (
    <div className="mdx-content prose prose-invert prose-lg max-w-none">
      <Component />
      <style>{`
        .mdx-content h1 {
          @apply text-3xl font-bold mb-6 text-white;
        }
        .mdx-content h2 {
          @apply text-2xl font-semibold mb-4 mt-8 text-white;
        }
        .mdx-content h3 {
          @apply text-xl font-semibold mb-3 mt-6 text-white;
        }
        .mdx-content p {
          @apply mb-4 text-white/80 leading-relaxed;
        }
        .mdx-content ul {
          @apply mb-4 ml-6 list-disc text-white/80;
        }
        .mdx-content li {
          @apply mb-2;
        }
        .mdx-content a {
          @apply text-cyan-400 hover:text-cyan-300 underline;
        }
        .mdx-content strong {
          @apply font-semibold text-white;
        }
        .mdx-content em {
          @apply italic;
        }
      `}</style>
    </div>
  )
}