# Backend Testing Guide

## Overview
This guide helps verify that all backend functionality is working correctly after implementing the fixes.

## Prerequisites
1. SQL script executed in Supabase (`/supabase/fix_backend.sql`)
2. Environment variables configured in Vercel (see `/docs/vercel-env-setup.md`)
3. Application deployed to Vercel

## Test Scenarios

### 1. Application Form Submission (Apply Modal)
**Steps:**
1. Navigate to homepage
2. Click any "Apply now" button
3. Fill in all required fields:
   - Full name
   - Email
   - Designation
   - Organization
   - Reason for access (min 10 chars)
   - Optional: Phone, Research query
4. Submit the form

**Expected Results:**
- ✅ Success message appears: "Thank you! Application received..."
- ✅ Toast notification shows: "Query submitted!"
- ✅ Email sent to applicant with subject: "Application Received - Opoch Membership"
- ✅ Data saved in Supabase `leads` table
- ✅ Entry created in `emails_sent` table

**Verify in Supabase:**
```sql
-- Check latest leads
SELECT * FROM leads ORDER BY created_at DESC LIMIT 5;

-- Check email send status
SELECT * FROM emails_sent ORDER BY created_at DESC LIMIT 5;
```

### 2. Authentication Flow & Welcome Email
**Steps:**
1. Navigate to RBT section (#rbt)
2. Click "Show details"
3. Notice the AskBoxTeaser with rotating prompts
4. Click "Sign in to ask" button
5. Sign in with Google or GitHub

**Expected Results:**
- ✅ AskBoxTeaser shows rotating prompts before authentication
- ✅ Successful authentication
- ✅ Welcome email sent with branded design and subject: "Welcome to Opoch - Let's accelerate your moonshot! 🚀"
- ✅ AskBox component appears after sign in (functional version)
- ✅ User metadata updated with `welcome_email_sent: true`

**Verify in Supabase:**
```sql
-- Check auth metadata
SELECT id, email, raw_user_meta_data 
FROM auth.users 
WHERE email = 'your-test-email@example.com';

-- Check welcome email was sent
SELECT * FROM emails_sent 
WHERE to_email = 'your-test-email@example.com' 
AND template_name = 'welcome';
```

### 3. Query Submission (AskBox)
**Steps:**
1. Sign in (if not already)
2. Navigate to RBT section
3. Enter a query in the AskBox (or use a suggested prompt)
4. Submit the query

**Expected Results:**
- ✅ Toast notification: "Query submitted! We'll analyze your problem and get back to you soon."
- ✅ Query saved in Supabase `queries` table
- ✅ AskBox clears after submission

**Verify in Supabase:**
```sql
-- Check latest queries
SELECT * FROM queries ORDER BY created_at DESC LIMIT 5;
```

### 4. Error Handling
**Test Missing Required Fields:**
1. Try submitting Apply form with empty fields
2. Expected: Field-specific error messages appear

**Test Network Errors:**
1. Disable internet/use Network tab to simulate offline
2. Submit form
3. Expected: Toast error notification with descriptive message

### 5. Email Template Verification
**Check Templates Exist:**
```sql
SELECT name, subject, active FROM email_templates;
```

**Expected Output:**
- welcome: "Welcome to Opoch - Let's accelerate your moonshot! 🚀"
- apply-received: "Application Received - Opoch Membership"

### 6. RLS Policy Verification
**Test Permissions:**
```sql
-- Test leads insert (should work for both anon and authenticated)
-- This would be tested via the application

-- Test queries insert (should only work for authenticated)
-- This would be tested via the AskBox

-- Check policies are active
SELECT schemaname, tablename, policyname, permissive, roles, cmd 
FROM pg_policies 
WHERE tablename IN ('leads', 'queries', 'email_templates', 'emails_sent');
```

## Common Issues & Solutions

### Issue: Forms not submitting
**Solution:** 
- Check browser console for errors
- Verify Supabase URL and anon key in environment
- Check RLS policies are properly set

### Issue: Emails not being sent
**Solution:**
- Check Vercel function logs: Dashboard → Functions → send-email
- Verify Resend API key is valid
- Ensure email templates exist in database
- Check `emails_sent` table for error messages

### Issue: Welcome email sent multiple times
**Solution:**
- Check user metadata for `welcome_email_sent` flag
- Verify AuthContext logic is checking the flag correctly

### Issue: AskBox not appearing after sign in
**Solution:**
- Hard refresh the page (Cmd+Shift+R)
- Check authentication state in browser console
- Verify user object is properly set in AuthContext

## Production Monitoring

### Key Metrics to Track:
1. **Successful form submissions** vs **errors**
2. **Email delivery rate** (check `emails_sent` table)
3. **Query submission volume**
4. **Authentication success rate**

### SQL Queries for Monitoring:
```sql
-- Daily leads
SELECT DATE(created_at) as date, COUNT(*) as count 
FROM leads 
GROUP BY DATE(created_at) 
ORDER BY date DESC;

-- Email success rate
SELECT 
  status, 
  COUNT(*) as count,
  ROUND(COUNT(*) * 100.0 / SUM(COUNT(*)) OVER (), 2) as percentage
FROM emails_sent
WHERE created_at > NOW() - INTERVAL '7 days'
GROUP BY status;

-- Active users submitting queries
SELECT DATE(created_at) as date, COUNT(DISTINCT user_id) as unique_users
FROM queries
WHERE created_at > NOW() - INTERVAL '7 days'
GROUP BY DATE(created_at)
ORDER BY date DESC;
```

## Next Steps
1. Run through all test scenarios
2. Fix any issues found
3. Set up monitoring alerts for critical failures
4. Document any custom configurations needed