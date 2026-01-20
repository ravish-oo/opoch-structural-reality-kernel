import { createBrowserClient } from '@supabase/ssr'

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

// Create a browser client with automatic cookie chunking and subdomain sharing
// @supabase/ssr handles large sessions by splitting them across multiple cookies
// Cookie domain is environment-aware:
// - Development: 'localhost' (shares cookies between localhost:5173 and localhost:3000)
// - Production: '.opoch.com' (shares cookies between www.opoch.com and chat.opoch.com)
export const supabase = supabaseUrl && supabaseAnonKey
  ? createBrowserClient(supabaseUrl, supabaseAnonKey, {
      cookieOptions: {
        domain: import.meta.env.DEV ? 'localhost' : '.opoch.com',
        path: '/',
        sameSite: 'lax',
        secure: import.meta.env.PROD, // http in dev, https in prod
        maxAge: 31536000, // 1 year
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