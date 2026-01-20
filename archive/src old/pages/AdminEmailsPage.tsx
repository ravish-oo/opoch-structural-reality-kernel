import { useState, useEffect } from 'react'
import { supabase, isSupabaseConfigured } from '../lib/supabase'
import { format } from 'date-fns'
import { ArrowLeft, CheckCircle, XCircle, Clock, Mail } from 'lucide-react'
import { Link } from 'react-router-dom'
import SEOHead from '../components/SEOHead'
import { generateAdminMetadata } from '../lib/metadata'

interface EmailSent {
  id: string
  created_at: string
  to_email: string
  template_name: string
  subject: string
  status: string
  provider: string
  provider_id: string
  metadata: any
  lead_id: string
  error: string | null
}

export default function AdminEmailsPage() {
  const [emails, setEmails] = useState<EmailSent[]>([])
  const [loading, setLoading] = useState(true)
  const [filter, setFilter] = useState<'all' | 'sent' | 'failed'>('all')

  useEffect(() => {
    loadEmails()
  }, [])

  async function loadEmails() {
    if (!isSupabaseConfigured()) {
      // Load mock data
      setEmails([
        {
          id: '1',
          created_at: new Date().toISOString(),
          to_email: 'john@example.com',
          template_name: 'welcome',
          subject: 'Welcome to Opoch - Your journey begins',
          status: 'sent',
          provider: 'resend',
          provider_id: 're_123456',
          metadata: { variables: { name: 'John' } },
          lead_id: 'lead_1',
          error: null
        },
        {
          id: '2',
          created_at: new Date(Date.now() - 3600000).toISOString(),
          to_email: 'jane@example.com',
          template_name: 'apply-received',
          subject: 'Application received - Opoch',
          status: 'sent',
          provider: 'resend',
          provider_id: 're_789012',
          metadata: { 
            variables: { 
              name: 'Jane', 
              research_query: 'How to build a distributed ML training system?' 
            } 
          },
          lead_id: 'lead_2',
          error: null
        }
      ])
      setLoading(false)
      return
    }

    try {
      const { data, error } = await supabase
        .from('emails_sent')
        .select('*')
        .order('created_at', { ascending: false })

      if (!error && data) {
        setEmails(data)
      }
    } catch (err) {
      console.error('Failed to load emails:', err)
    } finally {
      setLoading(false)
    }
  }

  const filteredEmails = emails.filter(email => {
    if (filter === 'all') return true
    return email.status === filter
  })

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'sent':
        return <CheckCircle size={16} className="text-green-400" />
      case 'failed':
        return <XCircle size={16} className="text-red-400" />
      default:
        return <Clock size={16} className="text-yellow-400" />
    }
  }

  const adminMetadata = generateAdminMetadata('emails')

  return (
    <>
      <SEOHead metadata={adminMetadata} />
      <div className="min-h-screen bg-[#0B0F1A] text-white">
      <div className="mx-auto max-w-7xl px-6 py-20">
        <Link to="/" className="inline-flex items-center gap-2 text-sm text-white/60 hover:text-white mb-8">
          <ArrowLeft size={16} />
          Back to home
        </Link>

        <div className="mb-8">
          <h1 className="mb-4 text-4xl font-bold">Email Dashboard</h1>
          <p className="text-lg text-white/60">Monitor all emails sent from the platform.</p>
        </div>

        {/* Stats */}
        <div className="mb-8 grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className="rounded-xl border border-white/10 bg-white/5 p-6">
            <div className="flex items-center gap-3 mb-2">
              <Mail size={20} className="text-cyan-400" />
              <h3 className="font-medium">Total Emails</h3>
            </div>
            <p className="text-2xl font-bold">{emails.length}</p>
          </div>
          <div className="rounded-xl border border-white/10 bg-white/5 p-6">
            <div className="flex items-center gap-3 mb-2">
              <CheckCircle size={20} className="text-green-400" />
              <h3 className="font-medium">Sent</h3>
            </div>
            <p className="text-2xl font-bold">{emails.filter(e => e.status === 'sent').length}</p>
          </div>
          <div className="rounded-xl border border-white/10 bg-white/5 p-6">
            <div className="flex items-center gap-3 mb-2">
              <XCircle size={20} className="text-red-400" />
              <h3 className="font-medium">Failed</h3>
            </div>
            <p className="text-2xl font-bold">{emails.filter(e => e.status === 'failed').length}</p>
          </div>
        </div>

        {/* Filters */}
        <div className="mb-6 flex gap-2">
          <button
            onClick={() => setFilter('all')}
            className={`px-4 py-2 rounded-lg transition-colors ${
              filter === 'all'
                ? 'bg-cyan-500 text-black'
                : 'bg-white/10 hover:bg-white/20'
            }`}
          >
            All
          </button>
          <button
            onClick={() => setFilter('sent')}
            className={`px-4 py-2 rounded-lg transition-colors ${
              filter === 'sent'
                ? 'bg-green-500 text-black'
                : 'bg-white/10 hover:bg-white/20'
            }`}
          >
            Sent
          </button>
          <button
            onClick={() => setFilter('failed')}
            className={`px-4 py-2 rounded-lg transition-colors ${
              filter === 'failed'
                ? 'bg-red-500 text-black'
                : 'bg-white/10 hover:bg-white/20'
            }`}
          >
            Failed
          </button>
        </div>

        {/* Email List */}
        {loading ? (
          <div className="text-center text-white/60">Loading emails...</div>
        ) : filteredEmails.length === 0 ? (
          <div className="text-center text-white/60">No emails found.</div>
        ) : (
          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="border-b border-white/10 text-left text-sm text-white/60">
                  <th className="pb-3 pr-4">Status</th>
                  <th className="pb-3 pr-4">To</th>
                  <th className="pb-3 pr-4">Template</th>
                  <th className="pb-3 pr-4">Subject</th>
                  <th className="pb-3 pr-4">Sent</th>
                  <th className="pb-3">Provider ID</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-white/10">
                {filteredEmails.map((email) => (
                  <tr key={email.id} className="text-sm">
                    <td className="py-3 pr-4">
                      <div className="flex items-center gap-2">
                        {getStatusIcon(email.status)}
                        <span className="capitalize">{email.status}</span>
                      </div>
                    </td>
                    <td className="py-3 pr-4 text-white/80">{email.to_email}</td>
                    <td className="py-3 pr-4">
                      <span className="rounded-full bg-white/10 px-2 py-1 text-xs">
                        {email.template_name}
                      </span>
                    </td>
                    <td className="py-3 pr-4 text-white/60 max-w-xs truncate">{email.subject}</td>
                    <td className="py-3 pr-4 text-white/60">
                      {format(new Date(email.created_at), 'MMM d, h:mm a')}
                    </td>
                    <td className="py-3 text-white/40 font-mono text-xs">
                      {email.provider_id || '—'}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </div>
      </div>
    </>
  )
}