-- Comprehensive RLS Diagnostic and Fix Script
-- This will diagnose and fix the leads table RLS issues

-- ============================================
-- STEP 1: DIAGNOSTICS
-- ============================================

-- Check if RLS is enabled
SELECT 
    'RLS Status' as check_type,
    CASE WHEN relrowsecurity THEN 'ENABLED ⚠️' ELSE 'DISABLED ✓' END as status,
    'If ENABLED, policies control access' as notes
FROM pg_class 
WHERE relname = 'leads';

-- Show ALL existing policies
SELECT 
    'Existing Policies' as check_type,
    policyname as policy_name,
    cmd as operation,
    roles as applies_to
FROM pg_policies 
WHERE tablename = 'leads'
ORDER BY policyname;

-- Check current role and permissions
SELECT 
    'Current Role' as check_type,
    current_user as value,
    current_role as role;

-- Check table permissions for different roles
SELECT 
    'Permissions Check' as check_type,
    'anon role INSERT' as permission,
    has_table_privilege('anon', 'leads', 'INSERT')::text as has_permission
UNION ALL
SELECT 
    'Permissions Check',
    'authenticated role INSERT',
    has_table_privilege('authenticated', 'leads', 'INSERT')::text
UNION ALL
SELECT 
    'Permissions Check',
    'anon role SELECT',
    has_table_privilege('anon', 'leads', 'SELECT')::text;

-- ============================================
-- STEP 2: CLEAN UP ALL POLICIES
-- ============================================

-- Drop ALL existing policies (including the conflicting one)
DO $$ 
DECLARE
    pol record;
BEGIN
    FOR pol IN 
        SELECT policyname 
        FROM pg_policies 
        WHERE tablename = 'leads'
    LOOP
        EXECUTE format('DROP POLICY IF EXISTS %I ON leads', pol.policyname);
        RAISE NOTICE 'Dropped policy: %', pol.policyname;
    END LOOP;
END $$;

-- ============================================
-- STEP 3: CREATE FRESH POLICIES
-- ============================================

-- Create simple, working policies with unique names
CREATE POLICY "leads_insert_policy_v2" ON leads
    FOR INSERT 
    WITH CHECK (true);  -- Anyone can insert

CREATE POLICY "leads_select_policy_v2" ON leads
    FOR SELECT
    USING (true);  -- Anyone can read (you can restrict this later)

-- Enable RLS
ALTER TABLE leads ENABLE ROW LEVEL SECURITY;

-- ============================================
-- STEP 4: GRANT PERMISSIONS
-- ============================================

-- Grant ALL necessary permissions to ensure it works
GRANT USAGE ON SCHEMA public TO anon;
GRANT USAGE ON SCHEMA public TO authenticated;

GRANT ALL ON TABLE leads TO anon;
GRANT ALL ON TABLE leads TO authenticated;

-- Grant sequence permissions (for auto-generated IDs)
GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO anon;
GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO authenticated;

-- ============================================
-- STEP 5: VERIFY CONFIGURATION
-- ============================================

-- Final check - should show new policies
SELECT 
    '✓ NEW POLICIES' as status,
    policyname,
    cmd as operation,
    'Should show leads_insert_policy_v2 and leads_select_policy_v2' as expected
FROM pg_policies 
WHERE tablename = 'leads';

-- Test insert capability
SELECT 
    '✓ INSERT TEST' as status,
    'If you see this, SELECT works' as message,
    has_table_privilege('anon', 'leads', 'INSERT') as anon_can_insert,
    has_table_privilege('authenticated', 'leads', 'INSERT') as auth_can_insert;

-- ============================================
-- EMERGENCY OPTION: Disable RLS
-- ============================================
-- If nothing above works, uncomment this line:
-- ALTER TABLE leads DISABLE ROW LEVEL SECURITY;

-- Then to re-enable later:
 ALTER TABLE leads ENABLE ROW LEVEL SECURITY;