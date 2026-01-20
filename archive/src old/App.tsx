import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import { AuthProvider } from './contexts/AuthContext'
import { HelmetProvider } from 'react-helmet-async'
import { lazy, Suspense } from 'react'
import ProtectedRoute from './components/ProtectedRoute'
import { Loader2 } from 'lucide-react'

// Lazy load pages for better performance
const OpochLanding = lazy(() => import('./OpochLanding'))
const MoonshotsPage = lazy(() => import('./pages/MoonshotsPage'))
const MoonshotDetailPage = lazy(() => import('./pages/MoonshotDetailPage'))
const UpdatesPage = lazy(() => import('./pages/UpdatesPage'))
const UpdateDetailPage = lazy(() => import('./pages/UpdateDetailPage'))
const AdminEmailsPage = lazy(() => import('./pages/AdminEmailsPage'))
const AdminLeadsPage = lazy(() => import('./pages/AdminLeadsPage'))
const AuthCallbackPage = lazy(() => import('./pages/AuthCallbackPage'))

// Loading component
function PageLoader() {
  return (
    <div className="min-h-screen bg-[#0B0F1A] flex items-center justify-center">
      <Loader2 className="h-8 w-8 animate-spin text-cyan-500" />
    </div>
  )
}

export default function App() {
  return (
    <HelmetProvider>
      <AuthProvider>
        <Router>
          <Suspense fallback={<PageLoader />}>
            <Routes>
              <Route path="/" element={<OpochLanding />} />
          <Route path="/auth/callback" element={<AuthCallbackPage />} />
          <Route path="/moonshots" element={<MoonshotsPage />} />
          <Route path="/moonshots/:slug" element={<MoonshotDetailPage />} />
          <Route path="/updates" element={<UpdatesPage />} />
          <Route path="/updates/:slug" element={<UpdateDetailPage />} />
          <Route path="/admin/emails" element={
            <ProtectedRoute requireAdmin>
              <AdminEmailsPage />
            </ProtectedRoute>
          } />
          <Route path="/admin/leads" element={
            <ProtectedRoute requireAdmin>
              <AdminLeadsPage />
            </ProtectedRoute>
          } />
            </Routes>
          </Suspense>
        </Router>
      </AuthProvider>
    </HelmetProvider>
  )
}