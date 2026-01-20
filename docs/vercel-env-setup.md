# Vercel Environment Variables Setup

## Required Environment Variables

Add these environment variables in your Vercel project settings (Settings → Environment Variables):

### 1. Supabase Configuration
```
VITE_SUPABASE_URL=https://qaoodcvaismvqcydcudx.supabase.co
VITE_SUPABASE_ANON_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFhb29kY3ZhaXNtdnFjeWRjdWR4Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTYzNTc2NTksImV4cCI6MjA3MTkzMzY1OX0.voju8EcUkJpg3IFNHVzwZsc7TxnazeIpJXpfgN-EV50
SUPABASE_SERVICE_ROLE_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFhb29kY3ZhaXNtdnFjeWRjdWR4Iiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTc1NjM1NzY1OSwiZXhwIjoyMDcxOTMzNjU5fQ.C8VjDHAkLc9gXe_6EOVYrT1OVaZOT0mePxj1FaKU3_0
```

### 2. Email Service (Resend)
```
RESEND_API_KEY=re_vFEY2SSc_AtCNERcW84ybu4rg29cKLPcB
```

### 3. Security Configuration (Optional)
```
ALLOWED_ORIGIN=https://opoch.com
```
Note: If not set, the API will automatically allow requests from opoch.com, www.opoch.com, and localhost for development.

## ⚠️ Critical for Form Submission

**The following environment variables MUST be added in Vercel for the Apply form to work:**
- `SUPABASE_SERVICE_ROLE_KEY` - Required for API to access Supabase
- `RESEND_API_KEY` - Required for sending emails

Without these, the form will get stuck in "Submitting..." state.

## How to Add in Vercel

1. Go to your Vercel project dashboard
2. Click on "Settings" in the top navigation
3. Select "Environment Variables" from the left sidebar
4. Add each variable one by one:
   - Enter the key (e.g., `VITE_SUPABASE_URL`)
   - Enter the value
   - Select which environments to apply to (Production, Preview, Development)
   - Click "Save"

## Important Notes

- The `VITE_` prefix makes variables available to the client-side code
- `SUPABASE_SERVICE_ROLE_KEY` should only be used in server-side functions (like API routes)
- Never commit these values to your repository
- After adding variables, you need to redeploy for changes to take effect

## Testing

After setting up the variables and redeploying:

1. Test form submission on `/` (Apply form)
2. Test authentication (Google/GitHub sign in)
3. Test query submission in RBT section after signing in
4. Check browser console for any errors

## Troubleshooting

If emails are not being sent:
1. Check the Vercel function logs for `/api/send-email`
2. Verify Resend API key is valid
3. Ensure email templates exist in Supabase
4. Check CORS settings match your domain