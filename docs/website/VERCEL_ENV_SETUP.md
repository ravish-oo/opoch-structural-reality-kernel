# Vercel Environment Variables Setup

## Required Environment Variables for Production

Add these to your Vercel project settings at:
https://vercel.com/[your-org]/opoch/settings/environment-variables

### 1. Supabase Configuration
```
VITE_SUPABASE_URL=https://qaoodcvaismvqcydcudx.supabase.co
VITE_SUPABASE_ANON_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFhb29kY3ZhaXNtdnFjeWRjdWR4Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTYzNTc2NTksImV4cCI6MjA3MTkzMzY1OX0.voju8EcUkJpg3IFNHVzwZsc7TxnazeIpJXpfgN-EV50
```

### 2. Email Service (Resend)
```
RESEND_API_KEY=re_vFEY2SSc_AtCNERcW84ybu4rg29cKLPcB
```

### 3. Supabase Service Role (for server-side operations)
```
SUPABASE_SERVICE_ROLE_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFhb29kY3ZhaXNtdnFjeWRjdWR4Iiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTc1NjM1NzY1OSwiZXhwIjoyMDcxOTMzNjU5fQ.C8VjDHAkLc9gXe_6EOVYrT1OVaZOT0mePxj1FaKU3_0
```

## How to Add to Vercel

1. Go to your Vercel dashboard
2. Select your project
3. Go to Settings → Environment Variables
4. Add each variable with its value
5. Make sure they're available for Production, Preview, and Development
6. Save changes

## OAuth Setup (Google)

Since you've enabled Google OAuth in Supabase, you need to:

1. **In Google Cloud Console**:
   - Go to https://console.cloud.google.com/
   - Create or select a project
   - Enable Google+ API
   - Create OAuth 2.0 credentials
   - Add authorized redirect URIs:
     - `https://qaoodcvaismvqcydcudx.supabase.co/auth/v1/callback`
     - `http://localhost:3000/auth/callback` (for local development)
   
2. **In Supabase Dashboard**:
   - Go to Authentication → Providers
   - Enable Google
   - Add your Google Client ID and Secret
   - Save changes

3. **Update Site URL in Supabase**:
   - Go to Authentication → URL Configuration
   - Set Site URL to: `https://opoch.com`
   - Add redirect URLs:
     - `https://opoch.com/auth/callback`
     - `http://localhost:3000/auth/callback`

## Testing OAuth

After setup, the "Continue with Google" button should work at:
- https://opoch.com/admin/leads
- https://opoch.com/admin/emails

## Security Notes

- Never commit these values to Git
- Use different API keys for development vs production
- Rotate keys periodically
- Monitor usage in both Supabase and Resend dashboards