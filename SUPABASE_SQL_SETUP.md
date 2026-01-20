# Supabase Database Setup Instructions

## Quick Links
- **SQL Editor**: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/sql/new
- **Table Editor**: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/editor

## Step 1: Run Base Schema

Go to SQL Editor and run this first:

```sql
-- Copy everything from supabase/schema.sql
-- This creates:
-- - leads table (for Apply form submissions)
-- - queries table (for AskBox submissions)
-- - Enables Row Level Security
-- - Creates indexes for performance
```

## Step 2: Run Email System Schema

After Step 1 completes, run:

```sql
-- Copy everything from supabase/schema_phase3.sql
-- This creates:
-- - email_templates table
-- - emails_sent table (for tracking)
-- - Functions for email sending
```

## Step 3: Seed Email Templates

Finally, run:

```sql
-- Copy everything from supabase/seed_email_templates.sql
-- This adds default email templates:
-- - application_received
-- - weekly_digest
-- - moonshot_update
```

## Verification

After running all schemas, check:
1. Go to Table Editor
2. You should see these tables:
   - `leads` (empty)
   - `queries` (empty)
   - `email_templates` (3 templates)
   - `emails_sent` (empty)

## Test the Connection

1. Visit http://localhost:3000/admin/leads
2. Click "Test Connection" button
3. You should see "✅ Success"
4. Click "Test Insert" to create a test lead

## Ready to Use!

Once all schemas are run, your forms will:
- Save submissions to `leads` table
- Send confirmation emails via Resend
- Track all email activity

Need help? Check the browser console for debug info.