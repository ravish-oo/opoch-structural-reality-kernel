# Supabase Setup Guide

## Prerequisites
- Supabase account (create at https://supabase.com)
- Your Supabase project URL and anon key (already in .env.local)

## Step 1: Run Database Schema

1. Go to your Supabase dashboard
2. Navigate to SQL Editor (in the left sidebar)
3. Create a new query

### Run schemas in this order:

#### 1. Base Schema (Phase 1)
```sql
-- Copy and paste contents of supabase/schema.sql
```

#### 2. Email System Schema (Phase 3)
```sql
-- Copy and paste contents of supabase/schema_phase3.sql
```

#### 3. Email Templates Seed Data
```sql
-- Copy and paste contents of supabase/seed_email_templates.sql
```

## Step 2: Enable Authentication

1. Go to Authentication → Providers
2. Enable Email authentication
3. Enable Google OAuth (optional)
4. Enable GitHub OAuth (optional)

### Configure OAuth providers:

#### Google OAuth
1. Go to https://console.cloud.google.com/
2. Create a new project or select existing
3. Enable Google+ API
4. Create OAuth 2.0 credentials
5. Add authorized redirect URI: `https://[YOUR_PROJECT_ID].supabase.co/auth/v1/callback`
6. Copy Client ID and Secret to Supabase

#### GitHub OAuth
1. Go to https://github.com/settings/developers
2. Create a new OAuth App
3. Set Authorization callback URL: `https://[YOUR_PROJECT_ID].supabase.co/auth/v1/callback`
4. Copy Client ID and Secret to Supabase

## Step 3: Configure Email Settings

1. Go to Authentication → Email Templates
2. Customize email templates for:
   - Confirm signup
   - Reset password
   - Magic link

## Step 4: Set up Storage (Optional)

1. Go to Storage
2. Create a bucket named `avatars` (for user profile pictures)
3. Set bucket to public
4. Add RLS policies for authenticated users

## Step 5: Environment Variables

Ensure these are set in your Vercel project:
```
VITE_SUPABASE_URL=your_supabase_url
VITE_SUPABASE_ANON_KEY=your_supabase_anon_key
```

For server-side operations (future):
```
SUPABASE_SERVICE_ROLE_KEY=your_service_role_key
```

## Step 6: Test the Connection

1. Visit your local development site
2. Try submitting the Apply form
3. Check Supabase Table Editor → `leads` table
4. Try the Ask Box
5. Check Supabase Table Editor → `queries` table

## Troubleshooting

### RLS Policies Not Working
- Ensure RLS is enabled on tables
- Check policy definitions allow anonymous inserts
- Test with service role key (bypasses RLS)

### Forms Not Submitting
- Check browser console for errors
- Verify environment variables are loaded
- Ensure Supabase project is not paused

### Email Not Sending
- Email sending requires a server-side implementation
- We'll add this in Phase 3 with Resend/Postmark integration

## Security Best Practices

1. **Never expose service role key** to client
2. **Use RLS** for all tables
3. **Validate all inputs** before inserting
4. **Rate limit** form submissions
5. **Monitor** for suspicious activity

## Step 7: Set Up Email Sending (Resend)

1. Create a Resend account at https://resend.com
2. Get your API key from the dashboard
3. Add to Vercel environment variables:
```
RESEND_API_KEY=re_xxxxxxxxxxxxx
SUPABASE_SERVICE_ROLE_KEY=eyJhbGci...
```

4. Configure domain for sending:
   - Add DNS records as shown in Resend dashboard
   - Verify domain ownership
   - Update "from" email in `/api/send-email.ts`

## Testing Email Integration

1. Submit the Apply form on your site
2. Check Resend dashboard for sent emails
3. Check Supabase `emails_sent` table for logs
4. Monitor for any failed emails

## Next Steps

After email setup:
1. Build admin dashboard for lead management
2. Add real-time notifications
3. Set up automated follow-ups
4. Create email analytics dashboard