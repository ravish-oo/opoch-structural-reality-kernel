import dotenv from 'dotenv'
import { fileURLToPath } from 'url'
import { dirname, join } from 'path'

const __filename = fileURLToPath(import.meta.url)
const __dirname = dirname(__filename)

// Load environment variables
dotenv.config({ path: join(__dirname, '..', '.env.local') })

const supabaseUrl = process.env.VITE_SUPABASE_URL
const supabaseAnonKey = process.env.VITE_SUPABASE_ANON_KEY

console.log('🔧 Testing Supabase API directly...\n')

async function testAPI() {
  try {
    // Test direct API call
    const response = await fetch(`${supabaseUrl}/rest/v1/leads?select=count`, {
      headers: {
        'apikey': supabaseAnonKey,
        'Authorization': `Bearer ${supabaseAnonKey}`,
        'Content-Type': 'application/json',
        'Prefer': 'count=exact'
      }
    })
    
    console.log('Response status:', response.status)
    console.log('Response headers:', Object.fromEntries(response.headers.entries()))
    
    const text = await response.text()
    console.log('\nResponse body:', text)
    
    if (!response.ok) {
      console.log('\n❌ API call failed')
    } else {
      console.log('\n✅ API call successful')
      
      // Try to parse count
      const count = response.headers.get('content-range')
      if (count) {
        console.log('Row count from header:', count)
      }
    }
    
  } catch (error) {
    console.error('❌ Network error:', error.message)
  }
}

testAPI()