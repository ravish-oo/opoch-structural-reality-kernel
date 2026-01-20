import type { VercelRequest, VercelResponse } from '@vercel/node'
import { createClient } from '@supabase/supabase-js'

const serviceRoleKey = process.env.SUPABASE_SERVICE_ROLE_KEY || process.env.supabase_service_role_key

const supabase = createClient(
  process.env.VITE_SUPABASE_URL!,
  serviceRoleKey!
)

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  // CORS headers
  const origin = req.headers.origin
  const allowedOrigins = ['https://opoch.com', 'https://www.opoch.com', 'http://localhost:5173', 'http://localhost:3000']
  
  if (origin && allowedOrigins.includes(origin)) {
    res.setHeader('Access-Control-Allow-Origin', origin)
  } else {
    res.setHeader('Access-Control-Allow-Origin', 'https://opoch.com')
  }
  
  res.setHeader('Access-Control-Allow-Methods', 'GET, OPTIONS')
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization')
  
  if (req.method === 'OPTIONS') {
    return res.status(200).end()
  }

  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' })
  }

  try {
    // Get user ID from query params
    const { userId } = req.query
    
    if (!userId || typeof userId !== 'string') {
      return res.status(400).json({ error: 'User ID required' })
    }

    // Get user profile
    const { data: profile, error } = await supabase
      .from('profiles')
      .select('*')
      .eq('id', userId)
      .single()

    if (error && error.code !== 'PGRST116') { // PGRST116 = no rows returned
      console.error('Profile fetch error:', error)
      return res.status(500).json({ error: 'Failed to fetch profile' })
    }

    return res.status(200).json({ 
      success: true, 
      profile: profile || null 
    })
  } catch (err) {
    console.error('Unexpected error:', err)
    return res.status(500).json({ error: 'An unexpected error occurred' })
  }
}