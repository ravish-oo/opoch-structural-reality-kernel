import { createClient } from '@supabase/supabase-js'

const supabaseUrl = import.meta.env.VITE_SUPABASE_URL || ''
const supabaseAnonKey = import.meta.env.VITE_SUPABASE_ANON_KEY || ''

// Debug logging for development
if (import.meta.env.DEV) {
  console.log('🔧 Supabase Configuration Check:');
  console.log('URL:', supabaseUrl ? '✅ Set' : '❌ Missing');
  console.log('Anon Key:', supabaseAnonKey ? '✅ Set' : '❌ Missing');
  if (supabaseUrl) {
    console.log('Project ID:', supabaseUrl.match(/https:\/\/([^.]+)\.supabase\.co/)?.[1] || 'Unknown');
  }
}

// Create a dummy client if environment variables are missing
export const supabase = supabaseUrl && supabaseAnonKey
  ? createClient(supabaseUrl, supabaseAnonKey, {
      auth: {
        autoRefreshToken: true,
        persistSession: true,
        detectSessionInUrl: true,
        flowType: 'pkce'
      },
      global: {
        headers: {
          'X-Client-Info': 'opoch-web@1.0.0'
        }
      }
    })
  : null as any

// Helper to check if Supabase is configured
export const isSupabaseConfigured = () => Boolean(supabaseUrl && supabaseAnonKey)

// Test connection function for debugging
export const testSupabaseConnection = async () => {
  if (!isSupabaseConfigured()) {
    return { success: false, error: 'Supabase not configured' }
  }

  try {
    // Test basic connectivity
    const { data, error } = await supabase
      .from('leads')
      .select('count', { count: 'exact', head: true })

    if (error) {
      return { success: false, error: error.message }
    }

    return { 
      success: true, 
      message: 'Connection successful',
      rowCount: data || 0
    }
  } catch (err) {
    return { 
      success: false, 
      error: err instanceof Error ? err.message : 'Unknown error' 
    }
  }
}

export type Database = {
  public: {
    Tables: {
      [key: string]: any
    }
    Views: {
      [key: string]: any
    }
    Functions: {
      [key: string]: any
    }
    Enums: {
      [key: string]: any
    }
    CompositeTypes: {
      [key: string]: any
    }
  }
}