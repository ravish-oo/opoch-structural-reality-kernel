# Production Issues & Fixes

## Issue 1: "Auth not configured" - Environment Variables Not Loading

**Problem**: The Supabase environment variables are not being loaded in production.

**Solution**: Add environment variables to Vercel

1. Go to: https://vercel.com/dvcoolster/opoch-website/settings/environment-variables
2. Add these variables (click "Add New" for each):
   ```
   VITE_SUPABASE_URL = https://qaoodcvaismvqcydcudx.supabase.co
   VITE_SUPABASE_ANON_KEY = eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFhb29kY3ZhaXNtdnFjeWRjdWR4Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTYzNTc2NTksImV4cCI6MjA3MTkzMzY1OX0.voju8EcUkJpg3IFNHVzwZsc7TxnazeIpJXpfgN-EV50
   SUPABASE_SERVICE_ROLE_KEY = eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFhb29kY3ZhaXNtdnFjeWRjdWR4Iiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTc1NjM1NzY1OSwiZXhwIjoyMDcxOTMzNjU5fQ.C8VjDHAkLc9gXe_6EOVYrT1OVaZOT0mePxj1FaKU3_0
   RESEND_API_KEY = re_vFEY2SSc_AtCNERcW84ybu4rg29cKLPcB
   ```
3. Make sure to check all three boxes: Production, Preview, Development
4. Click "Save"
5. **IMPORTANT**: Redeploy the site after adding variables:
   - Go to the Deployments tab
   - Click the three dots on the latest deployment
   - Select "Redeploy"

## Issue 2: CSP Blocking Google Services

**Problem**: Content Security Policy is blocking Google Tag Manager and other external services.

**Fix**: We need to update the CSP in index.html to allow Google services.

## Issue 3: Missing PWA Icon

**Problem**: The manifest.json references an icon that doesn't exist (icon-192.png).

**Fix**: Either create the icon or update manifest.json to use existing icons.

## Issue 4: Service Worker Registration Failed

**Problem**: The service worker path is incorrect.

**Fix**: Update the service worker registration path.