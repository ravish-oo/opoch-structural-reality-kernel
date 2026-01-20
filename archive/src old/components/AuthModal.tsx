import { useState } from 'react'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from './ui/dialog'
import { Button } from './ui/button'
import { Github, Mail, Loader2 } from 'lucide-react'
import { useAuth } from '../contexts/AuthContext'

interface AuthModalProps {
  open: boolean
  onOpenChange: (open: boolean) => void
  title?: string
  description?: string
}

export default function AuthModal({ 
  open, 
  onOpenChange, 
  title = "Sign in to Opoch",
  description = "Choose your preferred sign-in method"
}: AuthModalProps) {
  const { signInWithGoogle, signInWithGitHub } = useAuth()
  const [loading, setLoading] = useState<'google' | 'github' | null>(null)

  async function handleGoogleSignIn() {
    setLoading('google')
    try {
      await signInWithGoogle()
    } finally {
      setLoading(null)
    }
  }

  async function handleGitHubSignIn() {
    setLoading('github')
    try {
      await signInWithGitHub()
    } finally {
      setLoading(null)
    }
  }

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-md rounded-2xl border-white/10 bg-[#0B0F1A] text-white">
        <DialogHeader>
          <DialogTitle className="text-xl font-semibold">{title}</DialogTitle>
        </DialogHeader>
        
        <div className="mt-4">
          <p className="text-sm text-white/60 mb-6">{description}</p>
          
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
            
            <Button
              onClick={handleGitHubSignIn}
              disabled={loading !== null}
              className="w-full rounded-xl bg-white/10 text-white hover:bg-white/20 disabled:opacity-50"
            >
              {loading === 'github' ? (
                <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              ) : (
                <Github className="mr-2 h-4 w-4" />
              )}
              Continue with GitHub
            </Button>
          </div>
          
          <p className="mt-6 text-xs text-white/40 text-center">
            By signing in, you agree to our Terms of Service and Privacy Policy.
          </p>
        </div>
      </DialogContent>
    </Dialog>
  )
}