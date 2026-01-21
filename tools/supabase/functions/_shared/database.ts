import { createClient, SupabaseClient } from 'https://esm.sh/@supabase/supabase-js@2.38.4'
import { SupabaseOptions } from './types.ts'

// Database helper functions
export class DatabaseClient {
  private supabase: SupabaseClient

  constructor(url?: string, key?: string, options?: SupabaseOptions) {
    const supabaseUrl = url || Deno.env.get('SUPABASE_URL')!
    const supabaseKey = key || Deno.env.get('SUPABASE_SERVICE_ROLE_KEY')!
    
    this.supabase = createClient(supabaseUrl, supabaseKey, options || {
      auth: {
        persistSession: false,
        autoRefreshToken: false,
      }
    })
  }

  // Get the raw Supabase client
  getClient(): SupabaseClient {
    return this.supabase
  }

  // Lead operations
  async createLead(data: {
    name: string
    email: string
    phone?: string
    designation?: string
    organization?: string
    reason?: string
    research_query?: string
    source?: string
    utm?: Record<string, any>
    user_id?: string
  }) {
    return await this.supabase
      .from('leads')
      .insert({
        ...data,
        status: 'new'
      })
      .select()
      .single()
  }

  async getLeads(filters?: {
    email?: string
    status?: string
    user_id?: string
  }, options?: {
    limit?: number
    offset?: number
  }) {
    let query = this.supabase.from('leads').select('*')
    
    if (filters?.email) {
      query = query.eq('email', filters.email)
    }
    if (filters?.status) {
      query = query.eq('status', filters.status)
    }
    if (filters?.user_id) {
      query = query.eq('user_id', filters.user_id)
    }
    if (options?.limit) {
      query = query.limit(options.limit)
    }
    if (options?.offset) {
      query = query.range(options.offset, options.offset + (options.limit || 10) - 1)
    }
    
    return await query.order('created_at', { ascending: false })
  }

  async updateLeadStatus(id: string, status: string) {
    return await this.supabase
      .from('leads')
      .update({ status })
      .eq('id', id)
      .select()
      .single()
  }

  // Query operations
  async createQuery(data: {
    text: string
    ip?: string
    user_agent?: string
    lead_id?: string
    context?: Record<string, any>
  }) {
    return await this.supabase
      .from('queries')
      .insert(data)
      .select()
      .single()
  }

  async getQueries(filters?: {
    lead_id?: string
  }, options?: {
    limit?: number
  }) {
    let query = this.supabase.from('queries').select('*')
    
    if (filters?.lead_id) {
      query = query.eq('lead_id', filters.lead_id)
    }
    if (options?.limit) {
      query = query.limit(options.limit)
    }
    
    return await query.order('created_at', { ascending: false })
  }

  // Email template operations
  async getEmailTemplate(name: string) {
    return await this.supabase
      .from('email_templates')
      .select('*')
      .eq('name', name)
      .single()
  }

  async upsertEmailTemplate(data: {
    name: string
    subject: string
    html_body?: string
    text_body?: string
  }) {
    return await this.supabase
      .from('email_templates')
      .upsert(data, { onConflict: 'name' })
      .select()
      .single()
  }

  // Email tracking operations
  async recordEmailSent(data: {
    to_email: string
    from_email: string
    subject: string
    body?: string
    html_body?: string
    template_name?: string
    metadata?: Record<string, any>
  }) {
    return await this.supabase
      .from('emails_sent')
      .insert({
        ...data,
        status: 'sent',
        sent_at: new Date().toISOString()
      })
      .select()
      .single()
  }

  async updateEmailStatus(id: string, updates: {
    status?: string
    error_message?: string
    opened_at?: string
    clicked_at?: string
  }) {
    return await this.supabase
      .from('emails_sent')
      .update(updates)
      .eq('id', id)
      .select()
      .single()
  }

  // Profile operations
  async getProfile(userId: string) {
    return await this.supabase
      .from('profiles')
      .select('*')
      .eq('id', userId)
      .single()
  }

  async updateProfile(userId: string, data: {
    full_name?: string
    phone?: string
    designation?: string
    organization?: string
    reason?: string
    has_completed_profile?: boolean
    welcomed_at?: string
  }) {
    return await this.supabase
      .from('profiles')
      .update(data)
      .eq('id', userId)
      .select()
      .single()
  }

  // Utility operations
  async executeRawQuery(query: string, params?: any[]) {
    // Note: Direct SQL execution requires setting up RPC functions
    // This is a placeholder for when RPC functions are configured
    throw new Error('Direct SQL execution requires RPC function setup')
  }

  async checkTableExists(tableName: string) {
    const { data, error } = await this.supabase
      .from('information_schema.tables')
      .select('table_name')
      .eq('table_schema', 'public')
      .eq('table_name', tableName)
      .single()
    
    return !error && data !== null
  }

  async getTableSchema(tableName: string) {
    const { data, error } = await this.supabase
      .from('information_schema.columns')
      .select('column_name, data_type, is_nullable, column_default')
      .eq('table_schema', 'public')
      .eq('table_name', tableName)
      .order('ordinal_position')
    
    if (error) throw error
    return data
  }
}

// Export a singleton instance for convenience
export const db = new DatabaseClient()