#!/usr/bin/env node

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

console.log('🔧 Testing Supabase Connection...\n')

if (!supabaseUrl || !supabaseAnonKey) {
  console.error('❌ Missing environment variables!')
  process.exit(1)
}

const supabase = createClient(supabaseUrl, supabaseAnonKey)

async function testConnection() {
  try {
    // Test 1: Check if we can connect
    console.log('📡 Testing basic connectivity...')
    const { data, error } = await supabase
      .from('leads')
      .select('count', { count: 'exact', head: true })
    
    if (error) {
      if (error.message.includes('relation "public.leads" does not exist')) {
        console.log('❌ Tables not created yet!')
        console.log('📋 Please run the SQL schemas in Supabase first')
        console.log('   Go to: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/sql/new')
        return
      }
      throw error
    }
    
    console.log('✅ Connected successfully!')
    console.log(`   Found ${data || 0} leads in database`)
    
    // Test 2: Try to insert a test lead
    console.log('\n📝 Testing insert capability...')
    const testLead = {
      name: 'Test User',
      email: `test-${Date.now()}@example.com`,
      designation: 'Developer',
      organization: 'Test Organization',
      reason: 'Testing Supabase connection',
      source: 'test-script'
    }
    
    const { data: insertData, error: insertError } = await supabase
      .from('leads')
      .insert(testLead)
      .select()
    
    if (insertError) {
      throw insertError
    }
    
    console.log('✅ Insert test passed!')
    console.log('   Created lead with ID:', insertData[0].id)
    
    // Test 3: Clean up test data
    console.log('\n🧹 Cleaning up test data...')
    const { error: deleteError } = await supabase
      .from('leads')
      .delete()
      .eq('id', insertData[0].id)
    
    if (deleteError) {
      console.log('⚠️  Could not clean up test data:', deleteError.message)
    } else {
      console.log('✅ Test data cleaned up')
    }
    
    console.log('\n🎉 All tests passed! Supabase is working correctly.')
    
  } catch (error) {
    console.error('\n❌ Connection test failed!')
    console.error('Error:', error.message)
    console.log('\n📋 Troubleshooting:')
    console.log('1. Make sure you ran all SQL schemas in Supabase')
    console.log('2. Check that your keys are correct')
    console.log('3. Ensure your Supabase project is not paused')
  }
}

testConnection()