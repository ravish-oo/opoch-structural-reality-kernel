import { createContext, useContext, useEffect, useState } from 'react'
import { User, Session } from '@supabase/supabase-js'
import { supabase, isSupabaseConfigured } from '../lib/supabase'
import { sendWelcomeEmail } from '../lib/email'
import { handleSupabaseError } from '../lib/error-handler'

interface AuthContextType {
  user: User | null
  session: Session | null
  loading: boolean
  signInWithGoogle: () => Promise<void>
  signInWithGitHub: () => Promise<void>
  signInWithEmail: (email: string, password: string) => Promise<void>
  signUpWithEmail: (email: string, password: string) => Promise<void>
  signOut: () => Promise<void>
  isAdmin: boolean
}

const AuthContext = createContext<AuthContextType | undefined>(undefined)

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null)
  const [session, setSession] = useState<Session | null>(null)
  const [loading, setLoading] = useState(true)
  const [isAdmin, setIsAdmin] = useState(false)

  // TEMPORARILY DISABLED - for use with focus listener if re-enabled
  // Force refresh function
  // const refreshAuth = async () => {
  //   const { data: { session } } = await supabase.auth.getSession()
  //   setSession(session)
  //   setUser(session?.user ?? null)
  //   checkAdminStatus(session?.user ?? null)
  // }

  useEffect(() => {
    if (!isSupabaseConfigured()) {
      console.log('⚠️ Supabase not configured')
      setLoading(false)
      return
    }

    console.log('🔧 AuthProvider initializing...')

    // Get initial session
    supabase.auth.getSession().then(({ data: { session } }: { data: { session: Session | null } }) => {
      console.log('📊 Initial session check:', session ? `User: ${session.user?.email}` : 'No session')
      console.log('🍪 Cookies in browser:', document.cookie.split(';').filter(c => c.includes('sb-')).length)
      setSession(session)
      setUser(session?.user ?? null)
      checkAdminStatus(session?.user ?? null)
      setLoading(false)
    })

    // Listen for auth changes
    const { data: { subscription } } = supabase.auth.onAuthStateChange(async (event: string, session: Session | null) => {
      console.log('🔔 Auth state changed:', event, session ? `User: ${session.user?.email}` : 'No session')
      setSession(session)
      setUser(session?.user ?? null)
      checkAdminStatus(session?.user ?? null)
      
      // Send welcome email on first sign in only
      if (event === 'SIGNED_IN' && session?.user) {
        // Check profile to see if user has been welcomed
        const { data: profile } = await supabase
          .from('profiles')
          .select('welcomed_at')
          .eq('id', session.user.id)
          .single()
        
        if (!profile?.welcomed_at) {
          const name = session.user.user_metadata?.full_name || 
                       session.user.user_metadata?.name || 
                       session.user.email?.split('@')[0] || 
                       'there'
          
          // Send welcome email
          await sendWelcomeEmail(name, session.user.email || '')
            .catch(error => console.error('Failed to send welcome email:', error))
          
          // Update profile to mark as welcomed
          await supabase
            .from('profiles')
            .update({ welcomed_at: new Date().toISOString() })
            .eq('id', session.user.id)
            .catch((error: any) => console.error('Failed to update welcomed_at:', error))
        }
      }
    })

    return () => subscription.unsubscribe()
  }, [])

  async function checkAdminStatus(user: User | null) {
    if (!user || !isSupabaseConfigured()) {
      setIsAdmin(false)
      return
    }

    try {
      const { data: profile } = await supabase
        .from('profiles')
        .select('role')
        .eq('id', user.id)
        .single()

      setIsAdmin(profile?.role === 'admin')
    } catch (error) {
      console.error('Error checking admin status:', error)
      setIsAdmin(false)
    }
  }

  async function signInWithGoogle() {
    if (!isSupabaseConfigured()) {
      console.log('Auth not configured')
      return
    }

    const { error } = await supabase.auth.signInWithOAuth({
      provider: 'google',
      options: {
        redirectTo: `${window.location.origin}/auth/callback`
      }
    })

    if (error) {
      handleSupabaseError(error)
    }
  }

  async function signInWithGitHub() {
    if (!isSupabaseConfigured()) {
      console.log('Auth not configured')
      return
    }

    const { error } = await supabase.auth.signInWithOAuth({
      provider: 'github',
      options: {
        redirectTo: `${window.location.origin}/auth/callback`
      }
    })

    if (error) {
      handleSupabaseError(error)
    }
  }

  async function signInWithEmail(email: string, password: string) {
    if (!isSupabaseConfigured()) {
      console.log('Auth not configured')
      return
    }

    const { error } = await supabase.auth.signInWithPassword({
      email,
      password
    })

    if (error) {
      handleSupabaseError(error)
      throw error
    }
  }

  async function signUpWithEmail(email: string, password: string) {
    if (!isSupabaseConfigured()) {
      console.log('Auth not configured')
      return
    }

    const { data, error } = await supabase.auth.signUp({
      email,
      password,
      options: {
        emailRedirectTo: `${window.location.origin}/auth/callback`
      }
    })

    if (error) {
      handleSupabaseError(error)
      throw error
    }

    // If email confirmation is disabled, user is signed in immediately
    // If enabled, user needs to confirm email first
    if (data.user && !data.user.confirmed_at) {
      console.log('Please check your email to confirm your account')
    }
  }

  async function signOut() {
    if (!isSupabaseConfigured()) {
      console.log('Auth not configured')
      return
    }

    try {
      // Clear local state first
      setUser(null)
      setSession(null)
      setIsAdmin(false)

      // Clear any saved drafts
      localStorage.removeItem('opoch.ask.draft')

      // Sign out from Supabase - this clears server-side session
      const { error } = await supabase.auth.signOut()

      if (error) {
        console.error('Sign out error:', error)
        handleSupabaseError(error)
      }

      // Manually clear all Supabase cookies for the domain
      // This is needed because @supabase/ssr doesn't clear cookies with custom domains properly
      const domain = import.meta.env.DEV ? 'localhost' : '.opoch.com'
      const cookiesToClear = document.cookie.split(';').map(c => c.trim())

      cookiesToClear.forEach(cookie => {
        const cookieName = cookie.split('=')[0]
        // Clear Supabase auth cookies (they start with sb-)
        if (cookieName.startsWith('sb-')) {
          // Clear for current domain
          document.cookie = `${cookieName}=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/; domain=${domain}`
          // Also clear without domain (for localhost)
          document.cookie = `${cookieName}=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/;`
        }
      })

      // Force a page reload to ensure clean state
      // Use replace to prevent back button issues
      window.location.replace('/')
    } catch (err) {
      console.error('Sign out failed:', err)
      // Still attempt to redirect even on error
      window.location.replace('/')
    }
  }

  // TEMPORARILY DISABLED - may contribute to rate limit issues
  // Add a window focus listener to refresh auth state
  // useEffect(() => {
  //   const handleFocus = () => {
  //     refreshAuth()
  //   }
  //
  //   window.addEventListener('focus', handleFocus)
  //   return () => window.removeEventListener('focus', handleFocus)
  // }, [])

  return (
    <AuthContext.Provider
      value={{
        user,
        session,
        loading,
        signInWithGoogle,
        signInWithGitHub,
        signInWithEmail,
        signUpWithEmail,
        signOut,
        isAdmin
      }}
    >
      {children}
    </AuthContext.Provider>
  )
}

export function useAuth() {
  const context = useContext(AuthContext)
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider')
  }
  return context
}