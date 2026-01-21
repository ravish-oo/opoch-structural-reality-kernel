import { useState, useEffect } from 'react'
import { supabase, isSupabaseConfigured } from '../lib/supabase'
import { format } from 'date-fns'
import { ArrowLeft, User, Building, Search, Download } from 'lucide-react'
import { Link } from 'react-router-dom'
import SEOHead from '../components/SEOHead'
import { generateAdminMetadata } from '../lib/metadata'
import SupabaseTestPanel from '../components/SupabaseTestPanel'

interface Lead {
  id: string
  created_at: string
  name: string
  email: string
  phone: string | null
  designation: string
  organization: string
  reason: string
  research_query: string | null
  source: string
  status: string
  utm: any
}

export default function AdminLeadsPage() {
  const [leads, setLeads] = useState<Lead[]>([])
  const [loading, setLoading] = useState(true)
  const [searchTerm, setSearchTerm] = useState('')
  const [statusFilter, setStatusFilter] = useState<string>('all')

  useEffect(() => {
    loadLeads()
  }, [])

  async function loadLeads() {
    if (!isSupabaseConfigured()) {
      // Load mock data
      setLeads([
        {
          id: '1',
          created_at: new Date().toISOString(),
          name: 'John Doe',
          email: 'john@example.com',
          phone: '+1234567890',
          designation: 'CTO',
          organization: 'TechCorp',
          reason: 'We need help scaling our ML pipeline to handle 10x traffic',
          research_query: 'How to optimize PyTorch inference for production?',
          source: 'moonshots',
          status: 'new',
          utm: {}
        },
        {
          id: '2',
          created_at: new Date(Date.now() - 86400000).toISOString(),
          name: 'Jane Smith',
          email: 'jane@startup.com',
          phone: null,
          designation: 'Founder',
          organization: 'AI Startup',
          reason: 'Looking for help with real-time data processing architecture',
          research_query: null,
          source: 'hero',
          status: 'contacted',
          utm: {}
        }
      ])
      setLoading(false)
      return
    }

    try {
      const { data, error } = await supabase
        .from('leads')
        .select('*')
        .order('created_at', { ascending: false })

      if (!error && data) {
        setLeads(data)
      }
    } catch (err) {
      console.error('Failed to load leads:', err)
    } finally {
      setLoading(false)
    }
  }

  async function updateLeadStatus(leadId: string, newStatus: string) {
    if (!isSupabaseConfigured()) {
      // Update mock data
      setLeads(leads.map(lead => 
        lead.id === leadId ? { ...lead, status: newStatus } : lead
      ))
      return
    }

    try {
      const { error } = await supabase
        .from('leads')
        .update({ status: newStatus })
        .eq('id', leadId)

      if (!error) {
        setLeads(leads.map(lead => 
          lead.id === leadId ? { ...lead, status: newStatus } : lead
        ))
      }
    } catch (err) {
      console.error('Failed to update lead status:', err)
    }
  }

  function exportLeadsAsCSV() {
    const headers = ['Date', 'Name', 'Email', 'Phone', 'Designation', 'Organization', 'Reason', 'Research Query', 'Source', 'Status']
    const rows = filteredLeads.map(lead => [
      format(new Date(lead.created_at), 'yyyy-MM-dd'),
      lead.name,
      lead.email,
      lead.phone || '',
      lead.designation,
      lead.organization,
      lead.reason,
      lead.research_query || '',
      lead.source,
      lead.status
    ])

    const csv = [headers, ...rows]
      .map(row => row.map(cell => `"${cell}"`).join(','))
      .join('\n')

    const blob = new Blob([csv], { type: 'text/csv' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `opoch-leads-${format(new Date(), 'yyyy-MM-dd')}.csv`
    a.click()
    URL.revokeObjectURL(url)
  }

  const filteredLeads = leads.filter(lead => {
    const matchesSearch = searchTerm === '' || 
      lead.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
      lead.email.toLowerCase().includes(searchTerm.toLowerCase()) ||
      lead.organization.toLowerCase().includes(searchTerm.toLowerCase())
    
    const matchesStatus = statusFilter === 'all' || lead.status === statusFilter
    
    return matchesSearch && matchesStatus
  })

  const statusOptions = ['new', 'contacted', 'qualified', 'converted', 'dropped']
  
  const adminMetadata = generateAdminMetadata('leads')

  return (
    <>
      <SEOHead metadata={adminMetadata} />
      <div className="min-h-screen bg-[#0B0F1A] text-white">
      <div className="mx-auto max-w-7xl px-6 py-20">
        <Link to="/" className="inline-flex items-center gap-2 text-sm text-white/60 hover:text-white mb-8">
          <ArrowLeft size={16} />
          Back to home
        </Link>

        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="mb-4 text-4xl font-bold">Leads Management</h1>
            <p className="text-lg text-white/60">View and manage all leads</p>
          </div>
          <button
            onClick={exportLeadsAsCSV}
            className="flex items-center gap-2 rounded-xl bg-cyan-500 px-4 py-2 text-black hover:bg-cyan-400"
          >
            <Download size={16} />
            Export CSV
          </button>
        </div>

        {/* Supabase Connection Test */}
        {!isSupabaseConfigured() && (
          <div className="mb-8">
            <SupabaseTestPanel />
          </div>
        )}

        {/* Stats */}
        <div className="mb-8 grid grid-cols-1 md:grid-cols-4 gap-4">
          <div className="rounded-xl border border-white/10 bg-white/5 p-6">
            <div className="flex items-center gap-3 mb-2">
              <User size={20} className="text-cyan-400" />
              <h3 className="font-medium">Total Leads</h3>
            </div>
            <p className="text-2xl font-bold">{leads.length}</p>
          </div>
          {statusOptions.slice(0, 3).map(status => (
            <div key={status} className="rounded-xl border border-white/10 bg-white/5 p-6">
              <h3 className="font-medium mb-2 capitalize">{status}</h3>
              <p className="text-2xl font-bold">{leads.filter(l => l.status === status).length}</p>
            </div>
          ))}
        </div>

        {/* Filters */}
        <div className="mb-6 flex gap-4">
          <div className="relative flex-1 max-w-md">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 text-white/40" size={20} />
            <input
              type="text"
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              placeholder="Search by name, email, or organization..."
              className="w-full rounded-xl border border-white/10 bg-white/5 pl-10 pr-4 py-2 text-white placeholder:text-white/40 focus:border-cyan-500/50 focus:outline-none"
            />
          </div>
          <select
            value={statusFilter}
            onChange={(e) => setStatusFilter(e.target.value)}
            className="rounded-xl border border-white/10 bg-white/5 px-4 py-2 text-white focus:border-cyan-500/50 focus:outline-none"
          >
            <option value="all">All Status</option>
            {statusOptions.map(status => (
              <option key={status} value={status}>{status}</option>
            ))}
          </select>
        </div>

        {/* Leads Table */}
        {loading ? (
          <div className="text-center text-white/60">Loading leads...</div>
        ) : filteredLeads.length === 0 ? (
          <div className="text-center text-white/60">No leads found.</div>
        ) : (
          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="border-b border-white/10 text-left text-sm text-white/60">
                  <th className="pb-3 pr-4">Date</th>
                  <th className="pb-3 pr-4">Name</th>
                  <th className="pb-3 pr-4">Organization</th>
                  <th className="pb-3 pr-4">Email</th>
                  <th className="pb-3 pr-4">Source</th>
                  <th className="pb-3 pr-4">Status</th>
                  <th className="pb-3">Actions</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-white/10">
                {filteredLeads.map((lead) => (
                  <tr key={lead.id} className="text-sm group">
                    <td className="py-4 pr-4 text-white/60">
                      {format(new Date(lead.created_at), 'MMM d')}
                    </td>
                    <td className="py-4 pr-4">
                      <div>
                        <p className="font-medium text-white">{lead.name}</p>
                        <p className="text-xs text-white/60">{lead.designation}</p>
                      </div>
                    </td>
                    <td className="py-4 pr-4">
                      <div className="flex items-center gap-1">
                        <Building size={14} className="text-white/40" />
                        <span className="text-white/80">{lead.organization}</span>
                      </div>
                    </td>
                    <td className="py-4 pr-4 text-white/60">{lead.email}</td>
                    <td className="py-4 pr-4">
                      <span className="rounded-full bg-white/10 px-2 py-1 text-xs">
                        {lead.source}
                      </span>
                    </td>
                    <td className="py-4 pr-4">
                      <select
                        value={lead.status}
                        onChange={(e) => updateLeadStatus(lead.id, e.target.value)}
                        className="rounded-lg bg-white/10 px-2 py-1 text-xs capitalize focus:outline-none"
                      >
                        {statusOptions.map(status => (
                          <option key={status} value={status}>{status}</option>
                        ))}
                      </select>
                    </td>
                    <td className="py-4">
                      <button
                        onClick={() => {
                          const subject = `Re: Your Opoch application`
                          const body = `Hi ${lead.name},\n\n`
                          window.location.href = `mailto:${lead.email}?subject=${encodeURIComponent(subject)}&body=${encodeURIComponent(body)}`
                        }}
                        className="text-cyan-400 hover:text-cyan-300 text-xs"
                      >
                        Email
                      </button>
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