import { serve } from "https://deno.land/std@0.168.0/http/server.ts"
import { createClient } from 'https://esm.sh/@supabase/supabase-js@2.38.4'

// Database operation types
type DbOperation = 
  | 'createLead'
  | 'createQuery'
  | 'updateLeadStatus'
  | 'getLeads'
  | 'getQueries'
  | 'getEmailTemplate'
  | 'recordEmailSent'
  | 'updateProfile'
  | 'getProfile'

interface DbRequest {
  operation: DbOperation
  data?: any
  filters?: any
  options?: any
}

serve(async (req) => {
  try {
    // Get authorization header
    const authHeader = req.headers.get('Authorization')
    if (!authHeader) {
      return new Response(
        JSON.stringify({ error: 'Missing authorization header' }),
        { status: 401, headers: { 'Content-Type': 'application/json' } }
      )
    }

    // Create Supabase client
    const supabaseUrl = Deno.env.get('SUPABASE_URL')!
    const supabaseServiceKey = Deno.env.get('SUPABASE_SERVICE_ROLE_KEY')!
    
    const supabase = createClient(supabaseUrl, supabaseServiceKey, {
      auth: {
        persistSession: false,
        autoRefreshToken: false,
      }
    })

    // Parse request
    const { operation, data, filters, options }: DbRequest = await req.json()

    let result: any

    switch (operation) {
      case 'createLead':
        result = await supabase
          .from('leads')
          .insert({
            name: data.name,
            email: data.email,
            phone: data.phone,
            designation: data.designation,
            organization: data.organization,
            reason: data.reason,
            research_query: data.research_query,
            source: data.source || 'api',
            utm: data.utm || {},
            status: 'new',
            user_id: data.user_id
          })
          .select()
          .single()
        break

      case 'createQuery':
        result = await supabase
          .from('queries')
          .insert({
            text: data.text,
            ip: data.ip,
            user_agent: data.user_agent,
            lead_id: data.lead_id,
            context: data.context || {}
          })
          .select()
          .single()
        break

      case 'updateLeadStatus':
        result = await supabase
          .from('leads')
          .update({ status: data.status })
          .eq('id', data.id)
          .select()
          .single()
        break

      case 'getLeads':
        let leadsQuery = supabase.from('leads').select('*')
        
        if (filters?.email) {
          leadsQuery = leadsQuery.eq('email', filters.email)
        }
        if (filters?.status) {
          leadsQuery = leadsQuery.eq('status', filters.status)
        }
        if (filters?.user_id) {
          leadsQuery = leadsQuery.eq('user_id', filters.user_id)
        }
        if (options?.limit) {
          leadsQuery = leadsQuery.limit(options.limit)
        }
        if (options?.offset) {
          leadsQuery = leadsQuery.range(options.offset, options.offset + (options.limit || 10) - 1)
        }
        
        leadsQuery = leadsQuery.order('created_at', { ascending: false })
        result = await leadsQuery
        break

      case 'getQueries':
        let queriesQuery = supabase.from('queries').select('*')
        
        if (filters?.lead_id) {
          queriesQuery = queriesQuery.eq('lead_id', filters.lead_id)
        }
        if (options?.limit) {
          queriesQuery = queriesQuery.limit(options.limit)
        }
        
        queriesQuery = queriesQuery.order('created_at', { ascending: false })
        result = await queriesQuery
        break

      case 'getEmailTemplate':
        result = await supabase
          .from('email_templates')
          .select('*')
          .eq('name', data.name)
          .single()
        break

      case 'recordEmailSent':
        result = await supabase
          .from('emails_sent')
          .insert({
            to_email: data.to_email,
            from_email: data.from_email,
            subject: data.subject,
            body: data.body,
            html_body: data.html_body,
            template_name: data.template_name,
            metadata: data.metadata || {},
            status: 'sent',
            sent_at: new Date().toISOString()
          })
          .select()
          .single()
        break

      case 'updateProfile':
        result = await supabase
          .from('profiles')
          .update({
            full_name: data.full_name,
            phone: data.phone,
            designation: data.designation,
            organization: data.organization,
            reason: data.reason,
            has_completed_profile: data.has_completed_profile,
            welcomed_at: data.welcomed_at
          })
          .eq('id', data.id)
          .select()
          .single()
        break

      case 'getProfile':
        result = await supabase
          .from('profiles')
          .select('*')
          .eq('id', data.id)
          .single()
        break

      default:
        return new Response(
          JSON.stringify({ error: `Unknown operation: ${operation}` }),
          { status: 400, headers: { 'Content-Type': 'application/json' } }
        )
    }

    if (result.error) {
      throw result.error
    }

    return new Response(
      JSON.stringify({ 
        success: true,
        data: result.data 
      }),
      { status: 200, headers: { 'Content-Type': 'application/json' } }
    )

  } catch (error) {
    return new Response(
      JSON.stringify({ 
        success: false,
        error: error.message 
      }),
      { status: 500, headers: { 'Content-Type': 'application/json' } }
    )
  }
})