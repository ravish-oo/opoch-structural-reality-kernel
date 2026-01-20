-- ========================================
-- VERIFICATION SCRIPT
-- Run this after the main setup to verify everything is correct
-- ========================================

-- 1. Check all tables exist with correct columns
SELECT 
  'Tables and Columns' as check_type,
  table_name,
  array_agg(column_name ORDER BY ordinal_position) as columns
FROM information_schema.columns
WHERE table_schema = 'public' 
  AND table_name IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
GROUP BY table_name
ORDER BY table_name;

-- 2. Check email templates
SELECT 
  'Email Templates' as check_type,
  name,
  subject,
  LENGTH(html_body) AS html_length,
  LENGTH(text_body) AS text_length,
  CASE 
    WHEN html_body LIKE '%https://www.opoch.com/Opoch%20Assets/Opoch%20Typelogo.png%' 
    THEN 'Using PNG logo ✓' 
    ELSE 'Logo path needs update' 
  END as logo_status
FROM public.email_templates
ORDER BY name;

-- 3. Check RLS is enabled
SELECT 
  'RLS Status' as check_type,
  schemaname,
  tablename,
  rowsecurity
FROM pg_tables
WHERE schemaname = 'public' 
  AND tablename IN ('profiles', 'email_templates', 'leads', 'queries', 'emails_sent')
ORDER BY tablename;

-- 4. Check policies
SELECT 
  'RLS Policies' as check_type,
  tablename,
  policyname,
  permissive,
  array_to_string(roles, ', ') as roles,
  cmd
FROM pg_policies
WHERE schemaname = 'public' 
  AND tablename IN ('profiles', 'email_templates', 'leads', 'queries', 'emails_sent')
ORDER BY tablename, policyname;

-- 5. Check triggers
SELECT 
  'Triggers' as check_type,
  tgname as trigger_name,
  tgrelid::regclass::text AS table_name,
  proname as function_name
FROM pg_trigger t
JOIN pg_proc p ON t.tgfoid = p.oid
WHERE tgrelid IN (
  'auth.users'::regclass, 
  'public.queries'::regclass, 
  'public.profiles'::regclass,
  'public.email_templates'::regclass
)
AND NOT tgisinternal
ORDER BY table_name, trigger_name;

-- 6. Check indexes
SELECT 
  'Indexes' as check_type,
  schemaname,
  tablename,
  indexname
FROM pg_indexes
WHERE schemaname = 'public' 
  AND tablename IN ('profiles', 'leads', 'queries', 'emails_sent')
  AND indexname NOT LIKE '%_pkey'
ORDER BY tablename, indexname;

-- 7. Check functions
SELECT 
  'Functions' as check_type,
  proname as function_name,
  pg_get_function_identity_arguments(oid) as arguments,
  prosrc as first_100_chars
FROM pg_proc
WHERE pronamespace = 'public'::regnamespace
  AND proname IN ('handle_new_user', 'set_query_user_id', 'current_uid', 'touch_updated_at')
ORDER BY proname;

-- 8. Test current_uid() function (will return NULL if not authenticated)
SELECT 
  'Current UID Test' as check_type,
  public.current_uid() as current_user_id,
  CASE 
    WHEN public.current_uid() IS NULL 
    THEN 'NULL (expected in SQL editor)' 
    ELSE 'Has value' 
  END as status;

-- Summary
SELECT 
  'SETUP VERIFICATION COMPLETE' as status,
  'Check the results above to ensure:' as instructions,
  '1. All tables have correct columns' as step1,
  '2. Email templates exist with PNG logo' as step2,
  '3. RLS is enabled on all tables' as step3,
  '4. Policies are created correctly' as step4,
  '5. Triggers are in place' as step5,
  '6. Indexes exist for performance' as step6;