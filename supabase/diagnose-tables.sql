-- ========================================
-- DIAGNOSTIC SCRIPT - RUN THIS TO SEE CURRENT STATE
-- ========================================

-- 1) Check which tables exist
SELECT 
  'Existing Tables' as diagnostic,
  table_name
FROM information_schema.tables
WHERE table_schema = 'public' 
  AND table_name IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY table_name;

-- 2) Check columns for each table
SELECT 
  'Table Columns' as diagnostic,
  table_name, 
  column_name,
  data_type,
  is_nullable
FROM information_schema.columns
WHERE table_schema = 'public'
  AND table_name IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY table_name, ordinal_position;

-- 3) Check if extensions are enabled
SELECT 
  'Extensions' as diagnostic,
  extname,
  extversion
FROM pg_extension
WHERE extname IN ('pgcrypto', 'uuid-ossp')
ORDER BY extname;

-- 4) Check RLS status
SELECT 
  'RLS Status' as diagnostic,
  schemaname,
  tablename,
  rowsecurity
FROM pg_tables
WHERE schemaname = 'public'
  AND tablename IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY tablename;

-- 5) Check existing policies
SELECT 
  'RLS Policies' as diagnostic,
  schemaname,
  tablename,
  policyname,
  cmd,
  permissive,
  roles
FROM pg_policies
WHERE schemaname = 'public'
  AND tablename IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY tablename, policyname;

-- 6) Check indexes
SELECT 
  'Indexes' as diagnostic,
  schemaname,
  tablename,
  indexname
FROM pg_indexes
WHERE schemaname = 'public'
  AND tablename IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY tablename, indexname;

-- 7) Check foreign key constraints
SELECT 
  'Foreign Keys' as diagnostic,
  tc.table_name,
  tc.constraint_name,
  kcu.column_name,
  ccu.table_name AS foreign_table_name,
  ccu.column_name AS foreign_column_name
FROM information_schema.table_constraints AS tc
JOIN information_schema.key_column_usage AS kcu
  ON tc.constraint_name = kcu.constraint_name
  AND tc.table_schema = kcu.table_schema
JOIN information_schema.constraint_column_usage AS ccu
  ON ccu.constraint_name = tc.constraint_name
  AND ccu.table_schema = tc.table_schema
WHERE tc.constraint_type = 'FOREIGN KEY'
  AND tc.table_schema = 'public'
  AND tc.table_name IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY tc.table_name, tc.constraint_name;

-- 8) Check triggers
SELECT 
  'Triggers' as diagnostic,
  event_object_table as table_name,
  trigger_name,
  event_manipulation,
  action_timing,
  action_statement
FROM information_schema.triggers
WHERE trigger_schema = 'public'
  AND event_object_table IN ('profiles', 'leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY event_object_table, trigger_name;

-- 9) Check auth schema triggers (for user creation)
SELECT 
  'Auth Triggers' as diagnostic,
  tgname as trigger_name,
  tgrelid::regclass::text AS table_name,
  proname as function_name
FROM pg_trigger t
JOIN pg_proc p ON t.tgfoid = p.oid
WHERE tgrelid = 'auth.users'::regclass
  AND NOT tgisinternal
ORDER BY trigger_name;

-- 10) Test auth.jwt() function availability
SELECT 
  'Auth Functions' as diagnostic,
  CASE 
    WHEN EXISTS (SELECT 1 FROM pg_proc WHERE proname = 'jwt' AND pronamespace = 'auth'::regnamespace) 
    THEN 'auth.jwt() exists' 
    ELSE 'auth.jwt() NOT FOUND' 
  END as jwt_status,
  CASE 
    WHEN EXISTS (SELECT 1 FROM pg_proc WHERE proname = 'role' AND pronamespace = 'auth'::regnamespace) 
    THEN 'auth.role() exists' 
    ELSE 'auth.role() NOT FOUND' 
  END as role_status,
  CASE 
    WHEN EXISTS (SELECT 1 FROM pg_proc WHERE proname = 'uid' AND pronamespace = 'auth'::regnamespace) 
    THEN 'auth.uid() exists' 
    ELSE 'auth.uid() NOT FOUND' 
  END as uid_status;