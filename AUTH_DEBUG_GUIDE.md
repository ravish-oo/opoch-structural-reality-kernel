# Google OAuth Debug Guide

## Current Status
- ✅ Google OAuth enabled in Supabase
- ⚠️ Login button not working yet

## Common Issues & Solutions

### 1. Check Supabase Configuration
Go to: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/auth/providers

Ensure:
- Google provider is enabled
- Client ID and Secret are filled in
- Authorized domains include: `opoch.com` and `localhost:3000`

### 2. Update Site URL in Supabase
Go to: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/auth/url-configuration

Set:
- Site URL: `https://opoch.com`
- Redirect URLs:
  ```
  https://opoch.com/auth/callback
  http://localhost:3000/auth/callback
  ```

### 3. Google Console Configuration
Go to: https://console.cloud.google.com/apis/credentials

For your OAuth 2.0 Client:
- Authorized JavaScript origins:
  ```
  https://opoch.com
  https://qaoodcvaismvqcydcudx.supabase.co
  http://localhost:3000
  ```
- Authorized redirect URIs:
  ```
  https://qaoodcvaismvqcydcudx.supabase.co/auth/v1/callback
  https://opoch.com/auth/callback
  http://localhost:3000/auth/callback
  ```

### 4. Browser Console Debugging
When you click "Continue with Google" at https://opoch.com/admin/leads:

1. Open browser console (F12)
2. Check for errors
3. Look for messages like:
   - "Auth not configured" → Environment variables missing
   - "Error signing in with Google" → OAuth misconfigured
   - Network errors → CORS or redirect issues

### 5. Test Authentication Flow
```javascript
// In browser console at opoch.com
console.log('Supabase URL:', import.meta.env.VITE_SUPABASE_URL);
console.log('Anon Key exists:', !!import.meta.env.VITE_SUPABASE_ANON_KEY);
```

### 6. Quick Fixes to Try

1. **Clear browser cache and cookies**
2. **Try incognito/private mode**
3. **Check if popup blockers are interfering**
4. **Ensure you're using HTTPS on production**

### 7. Manual Test in Console
```javascript
// Test if Supabase client exists
if (window.supabase) {
  console.log('Supabase client loaded');
} else {
  console.log('Supabase client NOT found');
}
```

## Next Steps After Fix

Once Google login works:
1. Test the login flow end-to-end
2. Check if user profile is created in `profiles` table
3. Verify admin status assignment works
4. Test protected routes access

## Need More Help?

1. Check Supabase logs: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/logs/auth
2. Review OAuth consent screen in Google Console
3. Ensure your Google account has access (if using test mode)