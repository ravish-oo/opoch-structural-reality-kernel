import { useState, useCallback, memo } from 'react'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from './ui/dialog'
import { Button } from './ui/button'
import { Input } from './ui/input'
import { Mail, Loader2, AlertCircle } from 'lucide-react'
import { useAuth } from '../contexts/AuthContext'
import type { ApiError } from '../types'

interface AuthModalProps {
  open: boolean
  onOpenChange: (open: boolean) => void
  title?: string
  description?: string
  onSuccess?: () => void
  onError?: (error: ApiError) => void
}

const AuthModal = memo(function AuthModal({
  open,
  onOpenChange,
  title = "Sign in to Opoch",
  description = "Choose your preferred sign-in method",
  onSuccess,
  onError
}: AuthModalProps) {
  const { signInWithGoogle, signInWithEmail, signUpWithEmail } = useAuth()
  const [loading, setLoading] = useState<'google' | 'email' | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [mode, setMode] = useState<'signin' | 'signup'>('signin')
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')

  const isDevAuthEnabled = import.meta.env.VITE_ENABLE_DEV_AUTH === 'true'

  const handleGoogleSignIn = useCallback(async () => {
    setLoading('google')
    setError(null)
    try {
      await signInWithGoogle()
      onSuccess?.()
    } catch (err) {
      const error = err as Error
      const apiError: ApiError = {
        message: error.message || 'Failed to sign in with Google',
        code: 'GOOGLE_AUTH_ERROR'
      }
      setError(apiError.message)
      onError?.(apiError)
    } finally {
      setLoading(null)
    }
  }, [signInWithGoogle, onSuccess, onError])

  const handleEmailAuth = useCallback(async (e: React.FormEvent) => {
    e.preventDefault()
    setLoading('email')
    setError(null)

    try {
      if (mode === 'signin') {
        await signInWithEmail(email, password)
      } else {
        await signUpWithEmail(email, password)
      }
      onSuccess?.()
      onOpenChange(false)
    } catch (err) {
      const error = err as Error
      const apiError: ApiError = {
        message: error.message || `Failed to ${mode === 'signin' ? 'sign in' : 'sign up'}`,
        code: 'EMAIL_AUTH_ERROR'
      }
      setError(apiError.message)
      onError?.(apiError)
    } finally {
      setLoading(null)
    }
  }, [mode, email, password, signInWithEmail, signUpWithEmail, onSuccess, onError, onOpenChange])

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-md rounded-2xl border-white/10 bg-[#0B0F1A] text-white" aria-describedby="auth-description">
        <DialogHeader>
          <DialogTitle className="text-xl font-semibold">{title}</DialogTitle>
          <p id="auth-description" className="sr-only">{description}</p>
        </DialogHeader>
        
        <div className="mt-4">
          <p className="text-sm text-white/60 mb-6">{description}</p>
          
          {error && (
            <div className="mb-4 flex items-center gap-2 rounded-lg bg-red-500/10 border border-red-500/20 p-3">
              <AlertCircle className="h-4 w-4 text-red-400 flex-shrink-0" />
              <p className="text-sm text-red-400">{error}</p>
            </div>
          )}

          {isDevAuthEnabled && (
            <>
              <form onSubmit={handleEmailAuth} className="space-y-4 mb-6">
                <div>
                  <label htmlFor="email" className="block text-sm text-white/80 mb-1">Email</label>
                  <Input
                    id="email"
                    type="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    placeholder="you@example.com"
                    required
                    disabled={loading !== null}
                    className="w-full bg-white/5 border-white/10 text-white placeholder:text-white/40"
                  />
                </div>

                <div>
                  <label htmlFor="password" className="block text-sm text-white/80 mb-1">Password</label>
                  <Input
                    id="password"
                    type="password"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    placeholder="••••••••"
                    required
                    minLength={6}
                    disabled={loading !== null}
                    className="w-full bg-white/5 border-white/10 text-white placeholder:text-white/40"
                  />
                </div>

                <div className="flex gap-2">
                  <Button
                    type="button"
                    onClick={() => setMode('signin')}
                    disabled={loading !== null}
                    className={`flex-1 rounded-xl ${mode === 'signin' ? 'bg-[#00B0FF] text-white hover:bg-[#00B0FF]/90' : 'bg-white/10 text-white hover:bg-white/20'} disabled:opacity-50`}
                  >
                    {mode === 'signin' ? 'Sign In' : 'Sign In'}
                  </Button>
                  <Button
                    type="button"
                    onClick={() => setMode('signup')}
                    disabled={loading !== null}
                    className={`flex-1 rounded-xl ${mode === 'signup' ? 'bg-[#00B0FF] text-white hover:bg-[#00B0FF]/90' : 'bg-white/10 text-white hover:bg-white/20'} disabled:opacity-50`}
                  >
                    {mode === 'signup' ? 'Sign Up' : 'Sign Up'}
                  </Button>
                </div>

                <Button
                  type="submit"
                  disabled={loading !== null}
                  className="w-full rounded-xl bg-gradient-to-r from-[#00B0FF] to-[#0080FF] text-white hover:opacity-90 disabled:opacity-50"
                >
                  {loading === 'email' ? (
                    <>
                      <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                      {mode === 'signin' ? 'Signing In...' : 'Signing Up...'}
                    </>
                  ) : (
                    <>Continue with Email</>
                  )}
                </Button>
              </form>

              <div className="relative mb-6">
                <div className="absolute inset-0 flex items-center">
                  <div className="w-full border-t border-white/10"></div>
                </div>
                <div className="relative flex justify-center text-xs">
                  <span className="bg-[#0B0F1A] px-2 text-white/40">or continue with</span>
                </div>
              </div>
            </>
          )}

          <div className="space-y-3">
            <Button
              onClick={handleGoogleSignIn}
              disabled={loading !== null}
              className="w-full rounded-xl bg-white text-black hover:bg-white/90 disabled:opacity-50"
            >
              {loading === 'google' ? (
                <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              ) : (
                <Mail className="mr-2 h-4 w-4" />
              )}
              Continue with Google
            </Button>
          </div>
          
          <p className="mt-6 text-xs text-white/40 text-center">
            By signing in, you agree to our Terms of Service and Privacy Policy.
          </p>
        </div>
      </DialogContent>
    </Dialog>
  )
})

export default AuthModal