import type { VercelRequest, VercelResponse } from '@vercel/node'

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  // Only allow GET
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' })
  }

  try {
    const url = process.env.VITE_SUPABASE_URL
    const anonKey = process.env.VITE_SUPABASE_ANON_KEY
    
    if (!url || !anonKey) {
      return res.status(500).json({ 
        ok: false, 
        error: 'Missing Supabase configuration',
        hasUrl: !!url,
        hasAnonKey: !!anonKey 
      })
    }

    // Test the REST API endpoint
    const response = await fetch(`${url}/rest/v1/`, {
      headers: {
        'apikey': anonKey,
        'Authorization': `Bearer ${anonKey}`
      },
      method: 'GET'
    })

    const responseText = await response.text()
    
    return res.status(200).json({ 
      ok: response.ok,
      status: response.status,
      statusText: response.statusText,
      headers: Object.fromEntries(response.headers.entries()),
      preview: responseText.slice(0, 200) + (responseText.length > 200 ? '...' : ''),
      timestamp: new Date().toISOString()
    })
  } catch (error) {
    console.error('Ping error:', error)
    return res.status(500).json({ 
      ok: false, 
      error: error instanceof Error ? error.message : 'Unknown error',
      timestamp: new Date().toISOString()
    })
  }
}