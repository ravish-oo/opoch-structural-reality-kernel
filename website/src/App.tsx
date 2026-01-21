import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import { AuthProvider } from './contexts/AuthContext'
import { HelmetProvider } from 'react-helmet-async'
import { lazy, Suspense } from 'react'
import ProtectedRoute from './components/ProtectedRoute'
import { Loader2 } from 'lucide-react'

// Lazy load pages for better performance
const OpochLanding = lazy(() => import('./OpochLanding'))
const HomeV2 = lazy(() => import('./pages/HomeV2'))
const ToePage = lazy(() => import('./pages/ToePage'))
const FalsifyPage = lazy(() => import('./pages/FalsifyPage'))
const VerifyPage = lazy(() => import('./pages/VerifyPage'))
const SimpleQuestionsPage = lazy(() => import('./pages/SimpleQuestionsPage'))
const PaperPage = lazy(() => import('./pages/PaperPage'))
const MoonshotsPage = lazy(() => import('./pages/MoonshotsPage'))
const MoonshotDetailPage = lazy(() => import('./pages/MoonshotDetailPage'))
const ReasoningSolvedPage = lazy(() => import('./pages/ReasoningSolvedPage'))
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
      <Router>
        <Suspense fallback={<PageLoader />}>
          <Routes>
            {/* Auth callback route - OUTSIDE AuthProvider to prevent auto-detection conflict */}
            <Route path="/auth/callback" element={<AuthCallbackPage />} />

            {/* All other routes wrapped in AuthProvider */}
            <Route path="*" element={
              <AuthProvider>
                <Routes>
                  <Route path="/" element={<HomeV2 />} />
                  <Route path="/toe" element={<ToePage />} />
                  <Route path="/consulting" element={<OpochLanding />} />
                  <Route path="/falsify" element={<FalsifyPage />} />
                  <Route path="/verify-independently" element={<VerifyPage />} />
                  <Route path="/simple-questions" element={<SimpleQuestionsPage />} />
                  <Route path="/paper" element={<PaperPage />} />
                  <Route path="/moonshots" element={<MoonshotsPage />} />
                  <Route path="/moonshots/:slug" element={<MoonshotDetailPage />} />
                  <Route path="/reasoning-solved" element={<ReasoningSolvedPage />} />
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
              </AuthProvider>
            } />
          </Routes>
        </Suspense>
      </Router>
    </HelmetProvider>
  )
}