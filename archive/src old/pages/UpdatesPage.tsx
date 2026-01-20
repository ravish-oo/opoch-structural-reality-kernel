import { useState, useEffect } from 'react'
import { supabase, isSupabaseConfigured } from '../lib/supabase'
import UpdateCard from '../components/UpdateCard'
import { ArrowLeft, Mail } from 'lucide-react'
import { Link } from 'react-router-dom'
import { Button } from '../components/ui/button'
import SEOHead from '../components/SEOHead'
import { getPageMetadata } from '../lib/metadata'

interface Update {
  id: string
  slug: string
  title: string
  excerpt: string
  published_at: string
  tags: string[]
  featured: boolean
}

export default function UpdatesPage() {
  const [updates, setUpdates] = useState<Update[]>([])
  const [loading, setLoading] = useState(true)
  const [email, setEmail] = useState('')
  const [subscribing, setSubscribing] = useState(false)
  const [subscribed, setSubscribed] = useState(false)

  useEffect(() => {
    loadUpdates()
  }, [])

  async function loadUpdates() {
    if (!isSupabaseConfigured()) {
      // Load mock data
      setUpdates([
        {
          id: '1',
          slug: 'introducing-opoch-moonshots',
          title: 'Introducing Opoch Moonshots',
          excerpt: 'We\'re excited to launch our moonshots program - tackling the hardest technical challenges in AI, data, and engineering.',
          published_at: new Date().toISOString(),
          tags: ['announcement', 'moonshots'],
          featured: true
        },
        {
          id: '2',
          slug: 'new-sf-office',
          title: 'New San Francisco Office',
          excerpt: 'We\'ve opened our new office in San Francisco. Drop by for a coffee and chat about your technical challenges.',
          published_at: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000).toISOString(),
          tags: ['office', 'news'],
          featured: false
        }
      ])
      setLoading(false)
      return
    }

    try {
      const { data, error } = await supabase
        .from('updates')
        .select('*')
        .eq('status', 'published')
        .lte('published_at', new Date().toISOString())
        .order('published_at', { ascending: false })

      if (!error && data) {
        setUpdates(data)
      }
    } catch (err) {
      console.error('Failed to load updates:', err)
    } finally {
      setLoading(false)
    }
  }

  async function handleSubscribe(e: React.FormEvent) {
    e.preventDefault()
    if (!email) return

    setSubscribing(true)
    
    try {
      if (isSupabaseConfigured()) {
        const { error } = await supabase
          .from('newsletter_subscribers')
          .insert({ email, source: 'website' })
        
        if (!error) {
          setSubscribed(true)
          setEmail('')
        }
      } else {
        // Just show success
        setSubscribed(true)
        setEmail('')
      }
    } catch (err) {
      console.error('Failed to subscribe:', err)
    } finally {
      setSubscribing(false)
    }
  }

  const updatesMetadata = getPageMetadata('updates')

  return (
    <>
      <SEOHead metadata={updatesMetadata} />
      <div className="min-h-screen bg-[#0B0F1A] text-white">
      <div className="relative mx-auto max-w-6xl px-6 py-20">
        <Link to="/" className="inline-flex items-center gap-2 text-sm text-white/60 hover:text-white mb-8">
          <ArrowLeft size={16} />
          Back to home
        </Link>

        <div className="mb-16">
          <h1 className="mb-4 text-4xl font-bold">Updates</h1>
          <p className="text-lg text-white/60">
            Latest news, insights, and technical deep-dives from Opoch.
          </p>
        </div>

        {/* Newsletter Signup */}
        <div className="mb-16 rounded-2xl border border-white/10 bg-white/5 p-8">
          <div className="mb-6 flex items-center gap-3">
            <div className="rounded-full bg-cyan-500/20 p-2">
              <Mail size={20} className="text-cyan-500" />
            </div>
            <div>
              <h3 className="text-lg font-semibold">Subscribe to updates</h3>
              <p className="text-sm text-white/60">Get notified when we publish new insights.</p>
            </div>
          </div>
          
          {subscribed ? (
            <p className="text-cyan-400">✓ Thanks for subscribing!</p>
          ) : (
            <form onSubmit={handleSubscribe} className="flex gap-3">
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="Enter your email"
                className="flex-1 rounded-xl border border-white/10 bg-white/5 px-4 py-2 text-white placeholder:text-white/50 focus:border-cyan-500/50 focus:outline-none"
                required
              />
              <Button
                type="submit"
                disabled={subscribing}
                className="rounded-xl bg-cyan-500 text-black hover:bg-cyan-400 disabled:opacity-50"
              >
                {subscribing ? 'Subscribing...' : 'Subscribe'}
              </Button>
            </form>
          )}
        </div>

        {/* Updates Grid */}
        {loading ? (
          <div className="text-center text-white/60">Loading updates...</div>
        ) : updates.length === 0 ? (
          <div className="text-center">
            <p className="mb-2 text-lg text-white/60">No updates yet.</p>
            <p className="text-sm text-white/40">Check back soon for our latest insights.</p>
          </div>
        ) : (
          <div className="grid gap-6 md:grid-cols-2">
            {updates.map((update) => (
              <UpdateCard key={update.id} {...update} publishedAt={update.published_at} />
            ))}
          </div>
        )}
      </div>
      </div>
    </>
  )
}