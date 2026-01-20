-- Final RLS Fix Script
-- This script properly configures RLS policies for production use

-- ============================================
-- STEP 1: Check Current Status
-- ============================================
SELECT 
    'Current RLS Status' as info,
    tablename,
    CASE WHEN rowsecurity THEN 'ENABLED' ELSE 'DISABLED' END as rls_status
FROM pg_tables 
JOIN pg_class ON pg_tables.tablename = pg_class.relname
WHERE schemaname = 'public' 
AND tablename IN ('leads', 'queries', 'email_templates', 'emails_sent');

-- ============================================
-- STEP 2: Properly Configure Leads Table
-- ============================================

-- First, ensure RLS is properly configured
ALTER TABLE leads ENABLE ROW LEVEL SECURITY;

-- Drop all existing policies on leads
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
    END LOOP;
END $$;

-- Create comprehensive policies for leads table
-- Policy 1: Anyone can insert (for the apply form)
CREATE POLICY "public_insert_leads" ON leads
    FOR INSERT 
    WITH CHECK (true);

-- Policy 2: Authenticated users can view all leads
CREATE POLICY "authenticated_select_leads" ON leads
    FOR SELECT
    TO authenticated
    USING (true);

-- Policy 3: Service role has full access
CREATE POLICY "service_role_all_leads" ON leads
    FOR ALL
    TO service_role
    USING (true)
    WITH CHECK (true);

-- ============================================
-- STEP 3: Configure Queries Table
-- ============================================
ALTER TABLE queries ENABLE ROW LEVEL SECURITY;

-- Drop existing policies
DO $$ 
DECLARE
    pol record;
BEGIN
    FOR pol IN 
        SELECT policyname 
        FROM pg_policies 
        WHERE tablename = 'queries'
    LOOP
        EXECUTE format('DROP POLICY IF EXISTS %I ON queries', pol.policyname);
    END LOOP;
END $$;

-- Create policies for queries
-- Authenticated users can insert and view their own queries
CREATE POLICY "users_insert_own_queries" ON queries
    FOR INSERT
    TO authenticated
    WITH CHECK (auth.uid()::text = user_id);

CREATE POLICY "users_view_own_queries" ON queries
    FOR SELECT
    TO authenticated
    USING (auth.uid()::text = user_id);

-- Service role has full access
CREATE POLICY "service_role_all_queries" ON queries
    FOR ALL
    TO service_role
    USING (true)
    WITH CHECK (true);

-- ============================================
-- STEP 4: Configure Email Tables
-- ============================================
ALTER TABLE email_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE emails_sent ENABLE ROW LEVEL SECURITY;

-- Email templates - only service role can access
CREATE POLICY "service_role_email_templates" ON email_templates
    FOR ALL
    TO service_role
    USING (true)
    WITH CHECK (true);

-- Emails sent - service role can insert, authenticated can view
CREATE POLICY "service_role_insert_emails" ON emails_sent
    FOR INSERT
    TO service_role
    WITH CHECK (true);

CREATE POLICY "authenticated_view_emails" ON emails_sent
    FOR SELECT
    TO authenticated
    USING (true);

-- ============================================
-- STEP 5: Grant Necessary Permissions
-- ============================================

-- Ensure anon role can insert into leads
GRANT USAGE ON SCHEMA public TO anon;
GRANT INSERT ON TABLE leads TO anon;
GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO anon;

-- Ensure authenticated role has proper permissions
GRANT USAGE ON SCHEMA public TO authenticated;
GRANT ALL ON TABLE queries TO authenticated;
GRANT SELECT ON TABLE leads TO authenticated;
GRANT SELECT ON TABLE emails_sent TO authenticated;
GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO authenticated;

-- ============================================
-- STEP 6: Verify Final Configuration
-- ============================================

-- Show all policies after changes
SELECT 
    tablename,
    policyname,
    permissive,
    roles,
    cmd,
    qual,
    with_check
FROM pg_policies 
WHERE schemaname = 'public'
AND tablename IN ('leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY tablename, policyname;

-- Test permissions
SELECT 
    'Permission Test Results' as info,
    'anon can INSERT into leads' as test,
    has_table_privilege('anon', 'leads', 'INSERT') as result
UNION ALL
SELECT 
    'Permission Test Results',
    'authenticated can INSERT into queries',
    has_table_privilege('authenticated', 'queries', 'INSERT')
UNION ALL
SELECT 
    'Permission Test Results',
    'authenticated can SELECT from leads',
    has_table_privilege('authenticated', 'leads', 'SELECT');

-- Show final RLS status
SELECT 
    '✓ FINAL STATUS' as status,
    'RLS is now properly configured. The apply form should work without timeouts.' as message;