# Backend Update Summary

## SQL Scripts to Run

### If You're Having Issues:

1. **Run Diagnostics First**: `/supabase/diagnose-tables.sql`
   - Shows current state of all tables, columns, policies, and auth functions
   - Helps identify what's missing or misconfigured

2. **Fix Missing Columns** (if needed): `/supabase/fix-missing-columns.sql`
   - Run if you get "column user_id does not exist" errors
   - Safely adds missing columns to existing tables
   - Re-creates RLS policies after columns are added

### Main Setup Scripts:

3. **Main Setup Script**: `/supabase/complete-backend-setup.sql`
   - Creates/reconciles all tables
   - Sets up email templates with html_body/text_body
   - Configures RLS policies
   - Adds auto-fill trigger for queries
   - Creates indexes for performance

4. **Email Logging Table**: `/supabase/emails-sent-table.sql`
   - HARDENED VERSION - handles all edge cases
   - Creates emails_sent table with column reconciliation
   - Uses auth.jwt()->>'role' for maximum compatibility
   - Includes unique index to prevent duplicate logs
   - Has optional verification queries at the end

5. **Verify Setup**: `/supabase/verify-setup.sql`
   - Run after all scripts to confirm everything is working

## Code Updates Already Applied

### 1. API Updates
- **send-email.ts**: Already using html_body/text_body correctly, removed 'active' check
- **submit-query.ts**: Updated to include user_id and use correct template name

### 2. Component Updates
- **ExploreWithOpoch.tsx**: Now sends userId with query submissions
- **AuthContext.tsx**: Already has welcome-once logic using user metadata

## Environment Variables Required

Ensure these are set in Vercel:
- `VITE_SUPABASE_URL`
- `VITE_SUPABASE_ANON_KEY`
- `SUPABASE_SERVICE_ROLE_KEY` (or `supabase_service_role_key`)
- `RESEND_API_KEY`

## Key Improvements in This Update

1. **Email Templates**: Now use separate html_body and text_body for better deliverability
2. **Query Auto-fill**: Queries automatically get user_id filled via trigger
3. **Security**: Removed unnecessary anon access to profiles
4. **Performance**: Added comprehensive indexes on all foreign keys and commonly queried fields
5. **Reliability**: UPSERT logic prevents race conditions in user profile creation

## Testing Checklist

After running the SQL scripts:

1. **Auth Flow**:
   - [ ] Sign up creates profile with email
   - [ ] Welcome email sent only once
   - [ ] Profile data persists

2. **Query Submission**:
   - [ ] Queries can be submitted without explicitly passing user_id
   - [ ] Query confirmation emails are sent
   - [ ] Queries are linked to correct user

3. **Email System**:
   - [ ] Templates load with html_body and text_body
   - [ ] Emails are logged in emails_sent table
   - [ ] PNG typelogo displays properly

4. **Form Submission**:
   - [ ] Apply form pre-fills user data
   - [ ] Lead submissions work through server endpoint
   - [ ] Profile updates after first submission

## Next Steps

1. Run the SQL scripts in order
2. Deploy the updated API files
3. Test all functionality
4. Monitor for any errors in logs