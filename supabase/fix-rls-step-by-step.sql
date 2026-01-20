-- Step-by-Step RLS Fix
-- Run each section separately to identify any issues

-- ============================================
-- SECTION 1: Check Current Status
-- ============================================
SELECT 
    tablename,
    CASE 
        WHEN rowsecurity THEN 'ENABLED' 
        ELSE 'DISABLED' 
    END as rls_status
FROM pg_tables 
JOIN pg_class ON pg_tables.tablename = pg_class.relname
WHERE schemaname = 'public' 
AND tablename IN ('leads', 'queries', 'email_templates', 'emails_sent');

-- ============================================
-- SECTION 2: Fix Leads Table
-- ============================================

-- Enable RLS
ALTER TABLE leads ENABLE ROW LEVEL SECURITY;

-- Drop all existing policies on leads (run one at a time if needed)
DROP POLICY IF EXISTS "public_insert_leads" ON leads;
DROP POLICY IF EXISTS "authenticated_select_leads" ON leads;
DROP POLICY IF EXISTS "service_role_all_leads" ON leads;
DROP POLICY IF EXISTS "Enable insert for all users" ON leads;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON leads;
DROP POLICY IF EXISTS "Anyone can insert leads" ON leads;
DROP POLICY IF EXISTS "leads_insert_policy_v2" ON leads;
DROP POLICY IF EXISTS "leads_select_policy_v2" ON leads;

-- Create new policies for leads
CREATE POLICY "public_insert_leads" ON leads
    FOR INSERT 
    WITH CHECK (true);

CREATE POLICY "authenticated_select_leads" ON leads
    FOR SELECT
    TO authenticated
    USING (true);

CREATE POLICY "service_role_all_leads" ON leads
    FOR ALL
    TO service_role
    USING (true)
    WITH CHECK (true);

-- Grant permissions for leads
GRANT INSERT ON TABLE leads TO anon;
GRANT SELECT ON TABLE leads TO authenticated;

-- ============================================
-- SECTION 3: Fix Queries Table
-- ============================================

-- Enable RLS
ALTER TABLE queries ENABLE ROW LEVEL SECURITY;

-- Drop all existing policies on queries
DROP POLICY IF EXISTS "anyone_insert_queries" ON queries;
DROP POLICY IF EXISTS "authenticated_view_queries" ON queries;
DROP POLICY IF EXISTS "service_role_all_queries" ON queries;
DROP POLICY IF EXISTS "Enable insert for authenticated users" ON queries;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON queries;
DROP POLICY IF EXISTS "Anyone can insert queries" ON queries;
DROP POLICY IF EXISTS "users_insert_own_queries" ON queries;
DROP POLICY IF EXISTS "users_view_own_queries" ON queries;

-- Create new policies for queries
CREATE POLICY "anyone_insert_queries" ON queries
    FOR INSERT
    WITH CHECK (true);

CREATE POLICY "authenticated_view_queries" ON queries
    FOR SELECT
    TO authenticated
    USING (true);

CREATE POLICY "service_role_all_queries" ON queries
    FOR ALL
    TO service_role
    USING (true)
    WITH CHECK (true);

-- Grant permissions for queries
GRANT INSERT ON TABLE queries TO anon;
GRANT SELECT ON TABLE queries TO authenticated;

-- ============================================
-- SECTION 4: Fix Email Tables
-- ============================================

-- Enable RLS
ALTER TABLE email_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE emails_sent ENABLE ROW LEVEL SECURITY;

-- Drop existing policies
DROP POLICY IF EXISTS "service_role_email_templates" ON email_templates;
DROP POLICY IF EXISTS "Enable read for service role" ON email_templates;

DROP POLICY IF EXISTS "service_role_insert_emails" ON emails_sent;
DROP POLICY IF EXISTS "authenticated_view_emails" ON emails_sent;
DROP POLICY IF EXISTS "Enable insert for service role" ON emails_sent;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON emails_sent;

-- Create new policies
CREATE POLICY "service_role_email_templates" ON email_templates
    FOR ALL
    TO service_role
    USING (true)
    WITH CHECK (true);

CREATE POLICY "service_role_insert_emails" ON emails_sent
    FOR INSERT
    TO service_role
    WITH CHECK (true);

CREATE POLICY "authenticated_view_emails" ON emails_sent
    FOR SELECT
    TO authenticated
    USING (true);

-- ============================================
-- SECTION 5: Grant Schema Permissions
-- ============================================

-- Grant schema usage
GRANT USAGE ON SCHEMA public TO anon;
GRANT USAGE ON SCHEMA public TO authenticated;

-- Grant sequence usage for auto-incrementing IDs
GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO anon;
GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO authenticated;

-- ============================================
-- SECTION 6: Verify Configuration
-- ============================================

-- Check final permissions
SELECT 
    'anon can INSERT into leads' as test,
    has_table_privilege('anon', 'leads', 'INSERT') as result
UNION ALL
SELECT 
    'anon can INSERT into queries',
    has_table_privilege('anon', 'queries', 'INSERT')
UNION ALL
SELECT 
    'authenticated can SELECT from leads',
    has_table_privilege('authenticated', 'leads', 'SELECT')
UNION ALL
SELECT 
    'authenticated can SELECT from queries',
    has_table_privilege('authenticated', 'queries', 'SELECT');

-- Show all active policies
SELECT 
    tablename,
    policyname,
    cmd as operation,
    roles
FROM pg_policies 
WHERE schemaname = 'public'
AND tablename IN ('leads', 'queries', 'email_templates', 'emails_sent')
ORDER BY tablename, policyname;

-- Final message
SELECT 
    '✓ Configuration Complete' as status,
    'If all tests show TRUE, your RLS is properly configured!' as message;