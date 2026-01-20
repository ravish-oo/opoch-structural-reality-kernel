import { useState, useEffect } from 'react'
import { useParams, Link } from 'react-router-dom'
import { supabase, isSupabaseConfigured } from '../lib/supabase'
import { format } from 'date-fns'
import { ArrowLeft, Calendar, Tag, User } from 'lucide-react'
import MDXContent from '../components/MDXContent'

interface Update {
  id: string
  slug: string
  title: string
  excerpt: string
  content: string
  published_at: string
  author: string
  tags: string[]
  featured: boolean
}

export default function UpdateDetailPage() {
  const { slug } = useParams<{ slug: string }>()
  const [update, setUpdate] = useState<Update | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    if (slug) {
      loadUpdate(slug)
    }
  }, [slug])

  async function loadUpdate(slug: string) {
    if (!isSupabaseConfigured()) {
      // Load mock data
      if (slug === 'introducing-opoch-moonshots') {
        setUpdate({
          id: '1',
          slug: 'introducing-opoch-moonshots',
          title: 'Introducing Opoch Moonshots',
          excerpt: 'We\'re excited to launch our moonshots program - tackling the hardest technical challenges in AI, data, and engineering.',
          content: `
# Introducing Opoch Moonshots

We're excited to announce the launch of our Moonshots program - a collection of ambitious technical challenges that push the boundaries of what's possible in AI, data science, and engineering.

## What are Moonshots?

Moonshots are problems that:
- Have no clear solution path
- Require breakthrough thinking
- Can create 10x impact when solved
- Challenge conventional approaches

## Our First Batch

We're launching with 15 moonshots across domains:

### AI & Machine Learning
- Real-time hallucination detection
- Self-healing ML pipelines
- Adversarial robustness at scale

### Data Engineering
- Instant data lake queries
- Zero-downtime schema evolution
- Real-time data quality scoring

### Systems & Infrastructure
- Cost-optimal autoscaling
- Quantum-resistant encryption migration
- Edge computing orchestration

## How It Works

1. **Browse Moonshots**: Explore our catalog of technical challenges
2. **Apply with Your Challenge**: Share your specific problem and constraints
3. **Deep Dive Session**: We map your problem space and identify solutions
4. **Implementation Support**: Get hands-on help building the solution

## Why Moonshots?

Traditional consulting focuses on best practices. We focus on breakthroughs. When you're facing a problem that has no playbook, that's where Opoch thrives.

Ready to tackle your moonshot? [Apply now](/apply).
`,
          published_at: new Date().toISOString(),
          author: 'Opoch Team',
          tags: ['announcement', 'moonshots'],
          featured: true
        })
      }
      setLoading(false)
      return
    }

    try {
      const { data, error } = await supabase
        .from('updates')
        .select('*')
        .eq('slug', slug)
        .eq('status', 'published')
        .single()

      if (!error && data) {
        setUpdate(data)
      }
    } catch (err) {
      console.error('Failed to load update:', err)
    } finally {
      setLoading(false)
    }
  }

  if (loading) {
    return (
      <div className="min-h-screen bg-[#0B0F1A] flex items-center justify-center">
        <p className="text-white/60">Loading...</p>
      </div>
    )
  }

  if (!update) {
    return (
      <div className="min-h-screen bg-[#0B0F1A] text-white">
        <div className="mx-auto max-w-4xl px-6 py-20">
          <Link to="/updates" className="inline-flex items-center gap-2 text-sm text-white/60 hover:text-white mb-8">
            <ArrowLeft size={16} />
            Back to updates
          </Link>
          <h1 className="mb-4 text-3xl font-bold">Update not found</h1>
          <p className="text-white/60">This update may have been moved or deleted.</p>
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen bg-[#0B0F1A] text-white">
      <div className="mx-auto max-w-4xl px-6 py-20">
        <Link to="/updates" className="inline-flex items-center gap-2 text-sm text-white/60 hover:text-white mb-8">
          <ArrowLeft size={16} />
          Back to updates
        </Link>

        <article>
          <header className="mb-8 border-b border-white/10 pb-8">
            <h1 className="mb-4 text-4xl font-bold">{update.title}</h1>
            <p className="mb-6 text-lg text-white/60">{update.excerpt}</p>
            
            <div className="flex flex-wrap items-center gap-6 text-sm text-white/40">
              <div className="flex items-center gap-2">
                <User size={16} />
                <span>{update.author}</span>
              </div>
              <div className="flex items-center gap-2">
                <Calendar size={16} />
                <time dateTime={update.published_at}>
                  {format(new Date(update.published_at), 'MMMM d, yyyy')}
                </time>
              </div>
              {update.tags.length > 0 && (
                <div className="flex items-center gap-2">
                  <Tag size={16} />
                  <div className="flex gap-2">
                    {update.tags.map((tag) => (
                      <span key={tag} className="rounded-full bg-white/10 px-2 py-1 text-xs">
                        {tag}
                      </span>
                    ))}
                  </div>
                </div>
              )}
            </div>
          </header>

          <div className="prose prose-invert max-w-none">
            <MDXContent content={update.content} />
          </div>
        </article>
      </div>
    </div>
  )
}