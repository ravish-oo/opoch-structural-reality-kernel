# RLS Fix Instructions

## The Error
You got the error `column "user_id" does not exist` because the original script assumed the `queries` table had a `user_id` column, but it actually uses `lead_id` to link queries to leads.

## Solution

### 1. Run the Corrected Script
Execute the contents of `/supabase/fix-rls-corrected.sql` in your Supabase SQL Editor.

This corrected script:
- ✅ Properly handles the `queries` table schema (no user_id column)
- ✅ Allows anyone (anonymous and authenticated) to insert queries
- ✅ Allows anyone to insert leads (for the apply form)
- ✅ Grants proper permissions to all roles

### 2. What the Script Does

**For the `leads` table:**
- Anyone can insert (needed for the apply form)
- Authenticated users can view all leads
- Service role has full access

**For the `queries` table:**
- Anyone can insert (needed for the AskBox component)
- Authenticated users can view all queries
- Service role has full access

**For email tables:**
- Only service role can access email templates
- Service role can insert email logs
- Authenticated users can view email logs

### 3. Testing

After running the script, test:

1. **Apply Form**: Submit a test application
2. **AskBox**: Submit a query (when logged in)
3. **Email Sending**: Should work when forms are submitted

### 4. Verification

The script includes verification queries that will show:
- Current RLS status
- All active policies
- Permission test results
- Final confirmation message

## If Issues Persist

If you still get timeout errors:
1. Check the browser console for specific error messages
2. Use the debug utility: `await window.debugSupabase()` in console
3. Temporarily disable RLS to confirm it's the issue: `ALTER TABLE leads DISABLE ROW LEVEL SECURITY;`