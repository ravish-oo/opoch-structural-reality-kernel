import React from 'react'
import ReactDOM from 'react-dom/client'
import { Analytics } from '@vercel/analytics/react'
import { SpeedInsights } from '@vercel/speed-insights/react'
import App from './App'
import './index.css'
import { initPerformance } from './lib/performance'

// Initialize performance monitoring
initPerformance()

// Import debug utility in development
if (import.meta.env.DEV || window.location.hostname === 'localhost') {
  import('./utils/debug-supabase').then(() => {
    console.log('🛠️ Debug mode enabled. Run debugSupabase() in console to diagnose issues.');
  });
}

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <App />
    <Analytics />
    <SpeedInsights />
  </React.StrictMode>,
)