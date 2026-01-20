import type { VercelRequest, VercelResponse } from '@vercel/node'
import { createClient } from '@supabase/supabase-js'

// Create Supabase client with service role for server-side operations
const serviceRoleKey = process.env.SUPABASE_SERVICE_ROLE_KEY || process.env.supabase_service_role_key

if (!serviceRoleKey) {
  console.error('Missing service role key - checked SUPABASE_SERVICE_ROLE_KEY and supabase_service_role_key')
}

const supabase = createClient(
  process.env.VITE_SUPABASE_URL!,
  serviceRoleKey!
)

interface SubmitLeadRequest {
  name: string
  email: string
  phone?: string
  designation: string
  organization: string
  reason: string
  research_query?: string
  source?: string
  userId?: string
  hasCompletedProfile?: boolean
}

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  // Set CORS headers
  const origin = req.headers.origin
  const allowedOrigins = ['https://opoch.com', 'https://www.opoch.com', 'http://localhost:5173', 'http://localhost:3000']
  
  if (origin && allowedOrigins.includes(origin)) {
    res.setHeader('Access-Control-Allow-Origin', origin)
  } else if (process.env.ALLOWED_ORIGIN) {
    res.setHeader('Access-Control-Allow-Origin', process.env.ALLOWED_ORIGIN)
  } else {
    res.setHeader('Access-Control-Allow-Origin', 'https://opoch.com')
  }
  
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS')
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type')
  res.setHeader('Access-Control-Max-Age', '86400')

  // Handle preflight
  if (req.method === 'OPTIONS') {
    return res.status(200).end()
  }

  // Only allow POST
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' })
  }

  try {
    const body = req.body as SubmitLeadRequest

    // Validate required fields
    if (!body.name || !body.email || !body.designation || !body.organization || !body.reason) {
      return res.status(400).json({ error: 'Missing required fields' })
    }

    // Insert lead using service role (bypasses RLS)
    const { data, error } = await supabase
      .from('leads')
      .insert({
        name: body.name,
        email: body.email,
        phone: body.phone || null,
        designation: body.designation,
        organization: body.organization,
        reason: body.reason,
        research_query: body.research_query || null,
        source: body.source || 'apply-form',
        utm: {}
      })
      .select('id, created_at')
      .single()

    if (error) {
      console.error('Database error:', error)
      return res.status(500).json({ 
        error: 'Failed to submit application',
        details: error.message 
      })
    }

    // Update user profile if userId provided
    if (body.userId && body.hasCompletedProfile) {
      try {
        await supabase
          .from('profiles')
          .upsert({
            id: body.userId,
            email: body.email,
            full_name: body.name,
            phone: body.phone,
            designation: body.designation,
            organization: body.organization,
            reason: body.reason,
            has_completed_profile: true,
            updated_at: new Date().toISOString()
          }, {
            onConflict: 'id'
          })
      } catch (profileError) {
        console.error('Profile update error (non-blocking):', profileError)
      }
    }

    // Send confirmation email (fire and forget)
    if (data) {
      try {
        const emailRes = await fetch(`${origin}/api/send-email`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            templateName: 'apply-received',
            to: body.email,
            variables: {
              name: body.name,
              research_query: body.research_query || 'Your technical challenge',
              lead_id: data.id
            },
            leadId: data.id
          })
        })
        
        if (!emailRes.ok) {
          console.error('Email send failed:', await emailRes.text())
        }
      } catch (emailError) {
        console.error('Email error (non-blocking):', emailError)
      }
    }

    return res.status(200).json({ 
      success: true, 
      data: { id: data.id, created_at: data.created_at } 
    })
  } catch (err) {
    console.error('Unexpected error:', err)
    return res.status(500).json({ error: 'An unexpected error occurred' })
  }
}