#!/usr/bin/env node
/**
 * Automated Supabase Database Setup
 * Runs all SQL schemas in the correct order
 */

import { createClient } from '@supabase/supabase-js'
import { readFileSync, existsSync } from 'fs'
import { join, dirname } from 'path'
import { fileURLToPath } from 'url'
import dotenv from 'dotenv'

const __filename = fileURLToPath(import.meta.url)
const __dirname = dirname(__filename)
const projectRoot = join(__dirname, '..')

// Load environment variables
const envPath = join(projectRoot, '.env.local')
if (existsSync(envPath)) {
  dotenv.config({ path: envPath })
}

console.log('🚀 Automated Supabase Database Setup\n')

// Check for required environment variables
const supabaseUrl = process.env.VITE_SUPABASE_URL
const serviceRoleKey = process.env.SUPABASE_SERVICE_ROLE_KEY

if (!supabaseUrl || !serviceRoleKey) {
  console.log('❌ Missing required environment variables:')
  console.log(`   VITE_SUPABASE_URL: ${supabaseUrl ? '✅' : '❌'}`)
  console.log(`   SUPABASE_SERVICE_ROLE_KEY: ${serviceRoleKey ? '✅' : '❌'}`)
  console.log('\n📋 To fix this:')
  console.log('1. Go to https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/settings/api')
  console.log('2. Copy the "service_role" key (NOT the anon key)')
  console.log('3. Add it to your .env.local file as SUPABASE_SERVICE_ROLE_KEY')
  console.log('4. Run this script again')
  process.exit(1)
}

// Create Supabase client with service role (admin privileges)
const supabase = createClient(supabaseUrl, serviceRoleKey)

function showSQLStep(stepNumber, description, sqlContent) {
  console.log(`\n📋 Step ${stepNumber}: ${description}`)
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
  console.log('🔗 Go to: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/sql/new')
  console.log('📝 Copy and paste this SQL:')
  console.log('')
  console.log('```sql')
  console.log(sqlContent)
  console.log('```')
  console.log('')
  console.log('⚡ Then click "Run" in the SQL Editor')
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
}

async function testConnection() {
  console.log('🔍 Testing database connection...')
  
  try {
    // Test with anon key (what the frontend will use)
    const anonKey = process.env.VITE_SUPABASE_ANON_KEY
    if (!anonKey) {
      console.log('   ⚠️  VITE_SUPABASE_ANON_KEY not found, skipping connection test')
      return false
    }
    
    const clientSupabase = createClient(supabaseUrl, anonKey)
    
    // Test basic connectivity
    const { data, error } = await clientSupabase
      .from('leads')
      .select('count', { count: 'exact', head: true })

    if (error) {
      console.log(`   ❌ Connection test failed: ${error.message}`)
      return false
    } else {
      console.log('   ✅ Database connection successful!')
      return true
    }
  } catch (err) {
    console.log(`   ❌ Connection error: ${err.message}`)
    return false
  }
}

async function setupDatabase() {
  console.log('This script will show you exactly what SQL to run in your Supabase dashboard.')
  console.log('Each step shows the SQL code and tells you where to paste it.\n')
  
  // Step 1: Base schema
  const baseSchema = readFileSync(join(projectRoot, 'supabase/schema.sql'), 'utf8')
  showSQLStep(1, 'Create base tables (leads, queries)', baseSchema)
  
  // Step 2: Email system schema  
  const emailSchema = readFileSync(join(projectRoot, 'supabase/schema_phase3.sql'), 'utf8')
  showSQLStep(2, 'Create email system tables', emailSchema)
  
  // Step 3: Seed email templates
  const emailSeeds = readFileSync(join(projectRoot, 'supabase/seed_email_templates.sql'), 'utf8')
  showSQLStep(3, 'Add default email templates', emailSeeds)
  
  console.log('\n🎯 After running all 3 SQL scripts above:')
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
  
  // Test connection
  const connected = await testConnection()
  
  if (connected) {
    console.log('\n🎉 Setup complete! Your database is ready.')
    console.log('\n📋 Next steps:')
    console.log('1. Start dev server: npm run dev')
    console.log('2. Visit: http://localhost:3001/admin/leads')
    console.log('3. Test the Apply form at: http://localhost:3001')
    console.log('4. Check submissions appear in your Supabase dashboard')
  } else {
    console.log('\n⚠️  Connection test failed. This means either:')
    console.log('   • You haven\'t run the SQL scripts yet')
    console.log('   • There\'s an issue with your .env.local file')
    console.log('   • The SQL scripts had errors')
    console.log('\n💡 Run this script again after completing the SQL steps above.')
  }
}

// Run the setup
setupDatabase().catch(err => {
  console.log(`\n❌ Setup failed: ${err.message}`)
  console.log('💡 Check your .env.local file and try again.')
  process.exit(1)
})
