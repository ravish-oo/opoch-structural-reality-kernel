import type { VercelRequest, VercelResponse } from '@vercel/node'
import { createClient } from '@supabase/supabase-js'

// Create Supabase client with service role
const serviceRoleKey = process.env.SUPABASE_SERVICE_ROLE_KEY || process.env.supabase_service_role_key

if (!serviceRoleKey) {
  console.error('Missing service role key - checked SUPABASE_SERVICE_ROLE_KEY and supabase_service_role_key')
}

const supabase = createClient(
  process.env.VITE_SUPABASE_URL!,
  serviceRoleKey!
)

interface SubmitQueryRequest {
  text: string
  context?: Record<string, any>
  lead_id?: string
  userId?: string
  userEmail?: string
  userName?: string
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
    const body = req.body as SubmitQueryRequest

    // Validate
    if (!body.text || !body.text.trim()) {
      return res.status(400).json({ error: 'Query text is required' })
    }

    // Insert query using service role
    const { data, error } = await supabase
      .from('queries')
      .insert({
        text: body.text,
        user_id: body.userId || null,
        context: body.context || { source: 'explore-with-opoch' },
        lead_id: body.lead_id || null
      })
      .select('id, created_at')
      .single()

    if (error) {
      console.error('Database error:', error)
      return res.status(500).json({ 
        error: 'Failed to submit query',
        details: error.message 
      })
    }

    // Send notification email if user info provided
    if (data && body.userEmail) {
      try {
        const origin = req.headers.origin || 'https://opoch.com'
        const emailRes = await fetch(`${origin}/api/send-email`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            templateName: 'query_confirmation',
            to: body.userEmail,
            variables: {
              user_name: body.userName || 'there',
              query_content: body.text
            }
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