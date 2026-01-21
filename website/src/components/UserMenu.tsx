import { useState, useRef, useEffect, useCallback, memo } from 'react'
import { useAuth } from '../contexts/AuthContext'
import { LogOut, Mail, Shield, MessageSquare } from 'lucide-react'
import { Link } from 'react-router-dom'

interface UserMenuProps {
  className?: string
}

const UserMenu = memo(function UserMenu({ className }: UserMenuProps = {}) {
  const { user, signOut, isAdmin } = useAuth()
  const [isOpen, setIsOpen] = useState(false)
  const menuRef = useRef<HTMLDivElement>(null)

  const handleClickOutside = useCallback((event: MouseEvent) => {
    if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
      setIsOpen(false)
    }
  }, [])

  useEffect(() => {
    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside)
      return () => document.removeEventListener('mousedown', handleClickOutside)
    }
  }, [isOpen, handleClickOutside])

  if (!user) return null

  return (
    <div className={`relative ${className || ''}`} ref={menuRef}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center gap-2 rounded-full bg-white/10 px-2 py-1.5 sm:px-3 text-sm hover:bg-white/20 transition-colors"
        aria-label="User menu"
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <img
          src={user.user_metadata?.avatar_url || `https://ui-avatars.com/api/?name=${encodeURIComponent(user.email || '')}&background=00B0FF&color=000`}
          alt="Avatar"
          className="h-6 w-6 rounded-full"
        />
        <span className="text-white/80 hidden md:inline">{user.email?.split('@')[0]}</span>
      </button>

      {isOpen && (
        <div 
          className="absolute right-0 mt-2 w-56 rounded-xl border border-white/10 bg-[#0B0F1A] shadow-xl"
          role="menu"
          aria-orientation="vertical"
          aria-labelledby="user-menu-button"
        >
          <div className="p-4 border-b border-white/10 space-y-1">
            <p className="text-sm font-medium text-white truncate">{user.user_metadata?.full_name || user.email?.split('@')[0]}</p>
            <p className="text-xs text-white/60 truncate">{user.email}</p>
          </div>

          <div className="p-2">
            <a
              href={import.meta.env.DEV ? 'http://localhost:3000' : 'https://chat.opoch.com'}
              className="flex items-center gap-2 rounded-lg px-3 py-2 text-sm text-white/80 hover:bg-white/10"
              role="menuitem"
            >
              <MessageSquare size={16} />
              Chat with UNIVERSE-TECH
            </a>

            {isAdmin && (
              <>
                <div className="my-1 border-t border-white/10" />
                <Link
                  to="/admin/emails"
                  className="flex items-center gap-2 rounded-lg px-3 py-2 text-sm text-white/80 hover:bg-white/10"
                  onClick={() => setIsOpen(false)}
                  role="menuitem"
                >
                  <Mail size={16} />
                  Email Dashboard
                </Link>
                <Link
                  to="/admin/leads"
                  className="flex items-center gap-2 rounded-lg px-3 py-2 text-sm text-white/80 hover:bg-white/10"
                  onClick={() => setIsOpen(false)}
                >
                  <Shield size={16} />
                  Manage Leads
                </Link>
              </>
            )}

            <div className="my-1 border-t border-white/10" />
            <button
              onClick={async () => {
                try {
                  await signOut()
                  setIsOpen(false)
                } catch (error) {
                  console.error('Sign out error:', error)
                }
              }}
              className="flex w-full items-center gap-2 rounded-lg px-3 py-2 text-sm text-white/80 hover:bg-white/10"
            >
              <LogOut size={16} />
              Sign out
            </button>
          </div>
        </div>
      )}
    </div>
  )
})

export default UserMenu