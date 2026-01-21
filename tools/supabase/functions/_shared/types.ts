// Database Types
export interface Lead {
  id: string
  created_at: string
  name: string
  email: string
  phone?: string
  designation?: string
  organization?: string
  reason?: string
  research_query?: string
  source?: string
  utm?: Record<string, any>
  status: 'new' | 'triage' | 'contacted' | 'closed'
  user_id?: string
}

export interface Query {
  id: string
  created_at: string
  text: string
  ip?: string
  user_agent?: string
  lead_id?: string
  context?: Record<string, any>
}

export interface EmailTemplate {
  id: string
  name: string
  subject: string
  html_body?: string
  text_body?: string
  created_at: string
  updated_at: string
}

export interface EmailSent {
  id: string
  created_at: string
  to_email: string
  from_email: string
  subject: string
  body?: string
  html_body?: string
  template_name?: string
  metadata?: Record<string, any>
  status: 'sent' | 'failed' | 'bounced' | 'delivered'
  error_message?: string
  sent_at?: string
  opened_at?: string
  clicked_at?: string
}

export interface Profile {
  id: string
  email?: string
  full_name?: string
  phone?: string
  designation?: string
  organization?: string
  reason?: string
  has_completed_profile: boolean
  welcomed_at?: string
  created_at: string
  updated_at: string
}

// API Response Types
export interface ApiResponse<T = any> {
  success: boolean
  data?: T
  error?: string
  message?: string
}

export interface PaginationOptions {
  limit?: number
  offset?: number
  orderBy?: string
  ascending?: boolean
}

// Supabase Client Options
export interface SupabaseOptions {
  auth: {
    persistSession: boolean
    autoRefreshToken: boolean
  }
  db?: {
    schema: string
  }
}

// Edge Function Utilities
export const corsHeaders = {
  'Access-Control-Allow-Origin': '*',
  'Access-Control-Allow-Headers': 'authorization, x-client-info, apikey, content-type',
}

export const jsonResponse = (data: any, status = 200) => {
  return new Response(
    JSON.stringify(data),
    { 
      status, 
      headers: { 
        'Content-Type': 'application/json',
        ...corsHeaders
      } 
    }
  )
}

export const errorResponse = (error: string, status = 500) => {
  return jsonResponse({ success: false, error }, status)
}

export const successResponse = (data: any) => {
  return jsonResponse({ success: true, data })
}