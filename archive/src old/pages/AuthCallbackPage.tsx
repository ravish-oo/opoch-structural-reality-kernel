import { useEffect } from 'react'
import { useNavigate } from 'react-router-dom'
import { Loader2 } from 'lucide-react'

export default function AuthCallbackPage() {
  const navigate = useNavigate()

  useEffect(() => {
    // Handle the auth callback and redirect
    const timer = setTimeout(() => {
      navigate('/')
    }, 1000)

    return () => clearTimeout(timer)
  }, [navigate])

  return (
    <div className="min-h-screen bg-[#0B0F1A] flex items-center justify-center">
      <div className="text-center">
        <Loader2 className="h-8 w-8 animate-spin text-cyan-500 mx-auto mb-4" />
        <p className="text-white/60">Signing you in...</p>
      </div>
    </div>
  )
}