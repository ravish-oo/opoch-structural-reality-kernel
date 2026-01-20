import { createClient } from '@supabase/supabase-js'
import dotenv from 'dotenv'
import { fileURLToPath } from 'url'
import { dirname, join } from 'path'

const __filename = fileURLToPath(import.meta.url)
const __dirname = dirname(__filename)

// Load environment variables
dotenv.config({ path: join(__dirname, '..', '.env.local') })

const supabaseUrl = process.env.VITE_SUPABASE_URL
const supabaseAnonKey = process.env.VITE_SUPABASE_ANON_KEY

console.log('🔍 Debugging Supabase Connection...\n')
console.log('URL:', supabaseUrl)
console.log('Key exists:', !!supabaseAnonKey)
console.log('Key length:', supabaseAnonKey ? supabaseAnonKey.length : 0)

if (!supabaseUrl || !supabaseAnonKey) {
  console.error('❌ Missing environment variables!')
  process.exit(1)
}

const supabase = createClient(supabaseUrl, supabaseAnonKey)

async function debugConnection() {
  console.log('\n📊 Testing database connection...')
  
  try {
    // Test 1: Basic query
    const { data, error, count } = await supabase
      .from('leads')
      .select('*', { count: 'exact', head: true })
    
    if (error) {
      console.log('\n❌ Query failed:')
      console.log('Error message:', error.message)
      console.log('Error code:', error.code)
      console.log('Error details:', error.details)
      console.log('Error hint:', error.hint)
      return
    }
    
    console.log('✅ Query successful!')
    console.log('Row count:', count)
    
    // Test 2: Check tables exist
    console.log('\n📋 Checking tables...')
    const tables = ['leads', 'queries', 'email_templates', 'emails_sent']
    
    for (const table of tables) {
      try {
        const { error } = await supabase.from(table).select('count', { count: 'exact', head: true })
        if (error) {
          console.log(`❌ ${table}: Not found`)
        } else {
          console.log(`✅ ${table}: Exists`)
        }
      } catch (e) {
        console.log(`❌ ${table}: Error - ${e.message}`)
      }
    }
    
  } catch (err) {
    console.error('\n❌ Connection failed!')
    console.error('Error:', err.message)
  }
}

debugConnection()