import { memo } from 'react'
import { format } from 'date-fns'
import { Calendar, Tag } from 'lucide-react'
import type { Update } from '../types'

interface UpdateCardProps extends Partial<Update> {
  title: string
  excerpt: string
  slug: string
  publishedAt: string
  tags: string[]
  featured?: boolean
  onClick?: () => void
}

const UpdateCard = memo(function UpdateCard({
  title,
  excerpt,
  slug,
  publishedAt,
  tags,
  featured,
  onClick
}: UpdateCardProps) {
  return (
    <article className={`group relative overflow-hidden rounded-2xl border ${
      featured ? 'border-cyan-500/50 bg-gradient-to-br from-cyan-500/10 to-transparent' : 'border-white/10'
    } bg-white/5 p-6 transition-all duration-300 hover:border-white/20 hover:bg-white/10`}>
      {featured && (
        <div className="absolute -right-8 top-4 rotate-45 bg-cyan-500 px-8 py-1 text-xs font-medium text-black">
          Featured
        </div>
      )}
      
      <div className="flex items-start justify-between gap-4">
        <div className="flex-1">
          <h3 className="mb-2 text-lg font-semibold text-white group-hover:text-cyan-400 transition-colors">
            <a 
              href={`/updates/${slug}`} 
              className="after:absolute after:inset-0"
              onClick={onClick}
              aria-label={`Read more about ${title}`}
            >
              {title}
            </a>
          </h3>
          <p className="mb-4 text-sm text-white/60 line-clamp-2">{excerpt}</p>
          
          <div className="flex items-center gap-4 text-xs text-white/40">
            <div className="flex items-center gap-1">
              <Calendar size={14} aria-hidden="true" />
              <time dateTime={publishedAt}>{format(new Date(publishedAt), 'MMM d, yyyy')}</time>
            </div>
            {tags.length > 0 && (
              <div className="flex items-center gap-1">
                <Tag size={14} aria-hidden="true" />
                <span aria-label={`Tags: ${tags.join(', ')}`}>
                  {tags[0]}
                  {tags.length > 1 && <span className="text-white/30">+{tags.length - 1}</span>}
                </span>
              </div>
            )}
          </div>
        </div>
      </div>
    </article>
  )
})

export default UpdateCard