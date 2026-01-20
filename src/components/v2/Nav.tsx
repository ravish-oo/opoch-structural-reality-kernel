import React, { useCallback, useState, lazy, Suspense } from "react"
import { Link, useLocation } from "react-router-dom"
import { Sparkles, Menu, X } from "lucide-react"
import OpochLogo from "../OpochLogo"
import Tag from "../Tag"
import UserMenu from "../UserMenu"
import { useAuth } from "../../contexts/AuthContext"
import { NAV_LINKS } from "../../sections/v2/constants"
import { trackNavLinkClicked, trackNavSignInClicked, trackNavMobileMenuOpened, trackNavMobileMenuClosed } from "../../lib/analytics"

// Lazy load AuthModal
const AuthModal = lazy(() => import("../AuthModal"))

const Nav = React.memo(() => {
  const { user } = useAuth()
  const location = useLocation()
  const [authModalOpen, setAuthModalOpen] = useState(false)
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false)

  const handleSignInClick = useCallback((isMobile: boolean = false) => {
    trackNavSignInClicked(location.pathname, isMobile)
    // Redirect to chat - let chat.opoch.com handle authentication
    const chatUrl = import.meta.env.DEV ? "http://localhost:3000" : "https://chat.opoch.com"
    window.location.href = chatUrl
  }, [location.pathname])

  const handleAuthSuccess = useCallback(() => {
    setAuthModalOpen(false)
  }, [])

  const handleNavLinkClick = useCallback((linkName: string, isMobile: boolean = false) => {
    trackNavLinkClicked(linkName, location.pathname, isMobile)
    if (isMobile) {
      setMobileMenuOpen(false)
    }
  }, [location.pathname])

  const toggleMobileMenu = useCallback(() => {
    if (!mobileMenuOpen) {
      trackNavMobileMenuOpened(location.pathname)
    } else {
      trackNavMobileMenuClosed(location.pathname)
    }
    setMobileMenuOpen(!mobileMenuOpen)
  }, [mobileMenuOpen, location.pathname])

  return (
    <>
      <nav className="fixed left-0 right-0 top-0 z-50 border-b border-white/10 bg-black/50 backdrop-blur-md">
        <div className="mx-auto max-w-7xl px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between py-4">
            {/* Logo + Age of Truth Tag */}
            <div className="flex items-center gap-3">
              <Link
                to="/"
                className="transition-colors duration-200 hover:opacity-80"
              >
                <OpochLogo width={100} height={32} />
              </Link>
              <Tag>
                <Sparkles className="h-3 w-3" /> Age of Truth
              </Tag>
            </div>

            {/* Desktop Navigation Links + User Menu / Sign In */}
            <div className="hidden items-center gap-8 md:flex">
              <Link
                to={NAV_LINKS.toe.url}
                onClick={() => handleNavLinkClick('ToE')}
                className="text-sm font-medium text-white/60 transition-colors duration-200 hover:text-white"
              >
                {NAV_LINKS.toe.label}
              </Link>
              <a
                href={NAV_LINKS.docs.url}
                target="_blank"
                rel="noopener noreferrer"
                onClick={() => handleNavLinkClick('Docs')}
                className="text-sm font-medium text-white/60 transition-colors duration-200 hover:text-white"
              >
                {NAV_LINKS.docs.label}
              </a>
              <Link
                to={NAV_LINKS.simpleQuestions.url}
                onClick={() => handleNavLinkClick('Simple Questions')}
                className="text-sm font-medium text-white/60 transition-colors duration-200 hover:text-white"
              >
                {NAV_LINKS.simpleQuestions.label}
              </Link>
              <Link
                to={NAV_LINKS.consulting.url}
                onClick={() => handleNavLinkClick('Consulting')}
                className="text-sm font-medium text-white/60 transition-colors duration-200 hover:text-white"
              >
                {NAV_LINKS.consulting.label}
              </Link>
              {user ? (
                <UserMenu />
              ) : (
                <button
                  onClick={() => handleSignInClick(false)}
                  className="rounded-2xl border border-white/20 bg-white/5 px-4 py-2 text-sm font-medium text-white transition-all hover:border-white/40 hover:bg-white/10"
                >
                  Sign in
                </button>
              )}
            </div>

            {/* Mobile Menu Button */}
            <button
              onClick={toggleMobileMenu}
              className="flex items-center justify-center text-white md:hidden"
              aria-label="Toggle menu"
            >
              {mobileMenuOpen ? (
                <X className="h-6 w-6" />
              ) : (
                <Menu className="h-6 w-6" />
              )}
            </button>
          </div>
        </div>

        {/* Mobile Menu */}
        {mobileMenuOpen && (
          <div className="border-t border-white/10 bg-black/95 md:hidden">
            <div className="mx-auto max-w-7xl space-y-1 px-4 py-4 sm:px-6 lg:px-8">
              <Link
                to={NAV_LINKS.toe.url}
                onClick={() => handleNavLinkClick('ToE', true)}
                className="block rounded-lg px-4 py-3 text-base font-medium text-white/60 transition-colors hover:bg-white/5 hover:text-white"
              >
                {NAV_LINKS.toe.label}
              </Link>
              <a
                href={NAV_LINKS.docs.url}
                target="_blank"
                rel="noopener noreferrer"
                onClick={() => handleNavLinkClick('Docs', true)}
                className="block rounded-lg px-4 py-3 text-base font-medium text-white/60 transition-colors hover:bg-white/5 hover:text-white"
              >
                {NAV_LINKS.docs.label}
              </a>
              <Link
                to={NAV_LINKS.simpleQuestions.url}
                onClick={() => handleNavLinkClick('Simple Questions', true)}
                className="block rounded-lg px-4 py-3 text-base font-medium text-white/60 transition-colors hover:bg-white/5 hover:text-white"
              >
                {NAV_LINKS.simpleQuestions.label}
              </Link>
              <Link
                to={NAV_LINKS.consulting.url}
                onClick={() => handleNavLinkClick('Consulting', true)}
                className="block rounded-lg px-4 py-3 text-base font-medium text-white/60 transition-colors hover:bg-white/5 hover:text-white"
              >
                {NAV_LINKS.consulting.label}
              </Link>

              {/* Mobile Sign In / User Menu */}
              {user ? (
                <div className="px-4 py-3">
                  <UserMenu />
                </div>
              ) : (
                <button
                  onClick={() => handleSignInClick(true)}
                  className="mx-4 mt-2 w-[calc(100%-2rem)] rounded-2xl border border-white/20 bg-white/5 px-4 py-3 text-base font-medium text-white transition-all hover:border-white/40 hover:bg-white/10"
                >
                  Sign in
                </button>
              )}
            </div>
          </div>
        )}
      </nav>

      {/* Auth Modal (lazy loaded) */}
      {authModalOpen && (
        <Suspense fallback={<div />}>
          <AuthModal
            open={authModalOpen}
            onOpenChange={setAuthModalOpen}
            title="Sign in to Opoch"
            description="Access the chat and explore solutions to frontier problems"
            onSuccess={handleAuthSuccess}
          />
        </Suspense>
      )}
    </>
  )
})

Nav.displayName = "Nav"

export default Nav
