import { useEffect, useState, useRef } from 'react'
import { useNavigate, useSearchParams } from 'react-router-dom'
import { Loader2 } from 'lucide-react'
import { createBrowserClient } from '@supabase/ssr'

export default function AuthCallbackPage() {
  const navigate = useNavigate()
  const [searchParams] = useSearchParams()
  const [error, setError] = useState<string | null>(null)
  const hasExchanged = useRef(false)

  useEffect(() => {
    if (hasExchanged.current) return
    hasExchanged.current = true

    console.log('🔐 Auth callback page mounted')

    const code = searchParams.get('code')
    if (!code) {
      console.error('❌ No code in URL')
      setError('No authorization code found')
      setTimeout(() => navigate('/'), 2000)
      return
    }

    // Create a NEW browser client with URL detection DISABLED for this callback
    const callbackClient = createBrowserClient(
      import.meta.env.VITE_SUPABASE_URL!,
      import.meta.env.VITE_SUPABASE_ANON_KEY!,
      {
        auth: {
          detectSessionInUrl: false,  // CRITICAL: Disable auto-detection
          persistSession: true,
          autoRefreshToken: true,
        },
        cookieOptions: {
          domain: import.meta.env.DEV ? 'localhost' : '.opoch.com',
          path: '/',
          sameSite: 'lax',
          secure: import.meta.env.PROD,
          maxAge: 31536000,
        }
      }
    )

    console.log('📝 Manually exchanging code for session...')

    const exchangeCode = async () => {
      try {
        const { data, error: exchangeError } = await callbackClient.auth.exchangeCodeForSession(code)

        if (exchangeError) {
          console.error('❌ Exchange failed:', exchangeError.message)
          setError('Authentication failed. Please try again.')
          setTimeout(() => navigate('/'), 3000)
          return
        }

        if (!data?.session) {
          console.error('❌ No session returned')
          setError('Authentication failed. No session created.')
          setTimeout(() => navigate('/'), 3000)
          return
        }

        console.log('✅ Authentication successful for:', data.session.user?.email)
        console.log('🍪 Cookies set via manual exchange')

        // Redirect to chat
        const chatUrl = import.meta.env.DEV ? 'http://localhost:3000' : 'https://chat.opoch.com'
        setTimeout(() => {
          console.log('🚀 Redirecting to:', chatUrl)
          window.location.href = chatUrl
        }, 1000)
      } catch (err) {
        console.error('💥 Unexpected error:', err)
        setError('An unexpected error occurred.')
        setTimeout(() => navigate('/'), 3000)
      }
    }

    exchangeCode()
  }, [navigate, searchParams])

  return (
    <div className="min-h-screen bg-[#0B0F1A] flex items-center justify-center">
      <div className="text-center">
        {error ? (
          <>
            <p className="text-red-400 mb-2">{error}</p>
            <p className="text-white/60 text-sm">Redirecting...</p>
          </>
        ) : (
          <>
            <Loader2 className="h-8 w-8 animate-spin text-cyan-500 mx-auto mb-4" />
            <p className="text-white/60">Signing you in...</p>
          </>
        )}
      </div>
    </div>
  )
}