# Auth & Database Debugging Guide

## Issues Fixed

### 1. Auth State Not Updating After Login
**Problem**: After login, the user avatar wasn't showing and the auth state wasn't properly updating.

**Fixes Applied**:
1. **Extended auth callback timeout**: Changed from 1 second to 2 seconds to give Supabase more time to process
2. **Added session verification**: The callback page now verifies the session before redirecting
3. **Added window focus refresh**: Auth state refreshes when the window regains focus
4. **Better error handling**: Shows errors if authentication fails

### 2. Supabase Insert Timeout
**Problem**: Form submissions were timing out when trying to insert into the database.

**Root Cause**: RLS (Row Level Security) policies were blocking the insert operation.

**Fix**: Created comprehensive RLS policies in `/supabase/fix-rls-final.sql`

## Steps to Apply Fixes

### 1. Run the RLS Fix Script
```sql
-- In Supabase SQL Editor, run:
/supabase/fix-rls-final.sql
```

This script:
- Properly configures RLS for all tables
- Ensures anonymous users can insert into leads table
- Grants proper permissions to all roles
- Provides verification queries to confirm everything works

### 2. Test Authentication
1. Sign out completely
2. Sign in again via Google/GitHub
3. You should see:
   - Proper redirect after 2 seconds
   - User avatar appears in the nav
   - Auth state persists across page refreshes

### 3. Test Form Submissions
1. Try submitting the apply form
2. Should complete without timeout
3. Check Supabase dashboard to verify the entry was created

## Debugging Commands

If issues persist, run these in browser console:

```javascript
// Check current auth state
const { data: { session } } = await window.supabase.auth.getSession()
console.log('Current session:', session)

// Check if user can insert into leads
const { error } = await window.supabase
  .from('leads')
  .insert({ 
    name: 'Test User',
    email: 'test@example.com',
    research_query: 'Test query'
  })
console.log('Insert result:', error || 'Success')

// Force refresh auth state
await window.supabase.auth.refreshSession()
```

## Common Issues & Solutions

### "RLS policy violation" error
- Run the fix-rls-final.sql script
- Ensure you're using the latest database schema

### Auth state not persisting
- Clear browser cache and cookies
- Check Supabase dashboard for any auth configuration issues
- Verify redirect URLs in Supabase auth settings include `/auth/callback`

### Forms still timing out
- Check browser console for specific errors
- Verify environment variables are set correctly
- Ensure Supabase project is not paused or rate-limited

## Next Steps After Testing

1. Remove debug logs from production code
2. Monitor error logs for any edge cases
3. Consider adding retry logic for database operations