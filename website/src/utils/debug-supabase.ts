import { supabase } from '../lib/supabase';

/**
 * Debug function to check Supabase connection and role
 * Call this from browser console: debugSupabase()
 */
export async function debugSupabase() {
  console.log('🔍 Debugging Supabase Connection...');
  
  // Check if Supabase is configured
  const url = import.meta.env.VITE_SUPABASE_URL;
  const anonKey = import.meta.env.VITE_SUPABASE_ANON_KEY;
  
  console.log('📍 Supabase URL:', url);
  console.log('🔑 Anon Key:', anonKey ? `${anonKey.substring(0, 20)}...` : 'NOT SET');
  
  try {
    // Check current session
    const { data: { session }, error: sessionError } = await supabase.auth.getSession();
    
    if (sessionError) {
      console.error('❌ Session Error:', sessionError);
    } else {
      console.log('👤 Current Session:', session ? 'Authenticated' : 'Anonymous');
      if (session) {
        console.log('📧 User Email:', session.user.email);
      }
    }
    
    // Test database connection with a simple query
    console.log('\n🔍 Testing Database Connection...');
    const { count, error: countError } = await supabase
      .from('leads')
      .select('*', { count: 'exact', head: true });
    
    if (countError) {
      console.error('❌ Database Error:', countError);
      console.error('Error Code:', countError.code);
      console.error('Error Details:', countError.details);
      console.error('Error Hint:', countError.hint);
      console.error('Error Message:', countError.message);
    } else {
      console.log('✅ Database Connection: SUCCESS');
      console.log('📊 Leads Table Count:', count);
    }
    
    // Test insert permission
    console.log('\n🔍 Testing Insert Permission...');
    const testData = {
      name: 'Debug Test',
      email: 'debug@test.com',
      designation: 'Test',
      organization: 'Test Org',
      reason: 'Testing RLS policies',
      source: 'debug'
    };
    
    const { error: insertError } = await supabase
      .from('leads')
      .insert(testData)
      .select()
      .single();
    
    if (insertError) {
      console.error('❌ Insert Error:', insertError);
      if (insertError.code === '42501') {
        console.error('🔒 This is a PERMISSION error - RLS policies are blocking the insert');
      }
    } else {
      console.log('✅ Insert Test: SUCCESS - RLS policies are working!');
    }
    
  } catch (error) {
    console.error('❌ Unexpected Error:', error);
  }
  
  console.log('\n📋 Diagnostic Complete!');
}

// Make it available globally for console access
if (typeof window !== 'undefined') {
  (window as any).debugSupabase = debugSupabase;
}