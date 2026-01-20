# Deployment Guide - Final Fixes

## Overview of Changes

This guide covers all the fixes implemented to resolve:
1. "Explore with Opoch" layout issues
2. Auth state not updating after login
3. Database timeout issues
4. Service worker chrome-extension errors
5. Accessibility warnings

## 1. Apply RLS Policies (CRITICAL - Do This First!)

Run the step-by-step RLS fix script in your Supabase SQL Editor:

```sql
-- Run each section of /supabase/fix-rls-step-by-step.sql
-- This will properly configure permissions for:
-- - Anonymous users to insert leads and queries
-- - Authenticated users to view data
-- - Service role to have full access
```

**Important**: Run each section separately and verify the results before proceeding.

## 2. Deploy Code Changes

All code changes have been pushed to GitHub. Vercel will automatically deploy them.

### What Was Changed:

#### A. New "Explore with Opoch" Component
- Created `/src/components/ExploreWithOpoch.tsx`
- Fixes layout issues (button below input, not inside)
- Allows typing before sign-in (sunk cost fallacy)
- Properly updates UI after authentication
- Mobile-responsive design

#### B. Auth State Improvements
- Extended auth callback timeout to 2 seconds
- Added session verification in callback page
- Added window focus listener to refresh auth state
- Fixed user avatar not showing after login

#### C. Service Worker Fix
- Updated `/public/sw.js` to ignore chrome-extension:// URLs
- Only caches responses from same origin
- Prevents API requests from being cached

#### D. Removed Timeout Workarounds
- Cleaned up timeout logic from ApplyModal
- Proper RLS policies eliminate need for timeouts

#### E. Accessibility Fixes
- Added aria-describedby to dialogs
- Proper ARIA labels for form fields

## 3. Verify Environment Variables

Ensure these are set in Vercel:
- `VITE_SUPABASE_URL`
- `VITE_SUPABASE_ANON_KEY`
- `SUPABASE_SERVICE_ROLE_KEY`
- `RESEND_API_KEY`

## 4. Testing Checklist

### Auth Flow:
- [ ] Sign out completely
- [ ] Click "Sign in to ask" in Explore section
- [ ] Sign in with Google/GitHub
- [ ] Verify avatar appears in nav
- [ ] Verify "Sign in to ask" changes to "Submit" button
- [ ] Verify you can submit queries

### Form Submissions:
- [ ] Submit apply form - should complete without timeout
- [ ] Check Supabase dashboard for new lead entry
- [ ] Verify email is sent with correct reply-to (hello@opoch.com)
- [ ] Logo displays properly in email

### Mobile Testing:
- [ ] Explore with Opoch section displays properly
- [ ] No overlapping text or buttons
- [ ] Forms are usable on small screens

### Service Worker:
- [ ] Check browser console - no chrome-extension errors
- [ ] Offline page works when internet disconnected
- [ ] API requests aren't being cached

## 5. Post-Deployment Monitoring

1. **Check Browser Console** for any errors
2. **Monitor Supabase Dashboard** for failed queries
3. **Test Form Submissions** from different browsers
4. **Verify Email Delivery** and formatting

## 6. If Issues Persist

### Database Timeouts:
```sql
-- Emergency: Temporarily disable RLS
ALTER TABLE leads DISABLE ROW LEVEL SECURITY;
ALTER TABLE queries DISABLE ROW LEVEL SECURITY;
```

### Auth Not Updating:
```javascript
// Force refresh in browser console
await window.supabase.auth.refreshSession()
```

### Clear Service Worker:
```javascript
// In browser console
navigator.serviceWorker.getRegistrations().then(regs => 
  regs.forEach(reg => reg.unregister())
)
```

## 7. Final Notes

- All debug console.logs have been kept for now to aid troubleshooting
- Once everything is confirmed working, remove debug statements
- The new ExploreWithOpoch component replaces both AskBox and AskBoxTeaser
- Service worker now properly ignores external requests

## Success Metrics

You'll know everything is working when:
✅ Users can type in Explore box before signing in
✅ Auth state updates immediately after login
✅ Forms submit without 10-second timeouts
✅ No chrome-extension errors in console
✅ Mobile layout is clean and usable
✅ Emails send with correct reply-to and logo