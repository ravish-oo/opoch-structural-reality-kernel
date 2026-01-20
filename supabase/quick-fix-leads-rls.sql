-- Quick Fix for Leads Table RLS
-- Run this in Supabase SQL Editor to fix form submission

-- 1. First, check if RLS is enabled
-- If this returns TRUE, RLS is blocking inserts
SELECT relrowsecurity FROM pg_class WHERE relname = 'leads';

-- 2. OPTION A: Disable RLS temporarily (Quick fix for testing)
-- Uncomment the line below to disable RLS:
-- ALTER TABLE leads DISABLE ROW LEVEL SECURITY;

-- 3. OPTION B: Fix RLS policies (Recommended)
-- Drop all existing policies on leads table
DROP POLICY IF EXISTS "Enable insert for all users" ON leads;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON leads;
DROP POLICY IF EXISTS "Enable read for all users" ON leads;
DROP POLICY IF EXISTS "Allow anonymous inserts" ON leads;
DROP POLICY IF EXISTS "Allow authenticated inserts" ON leads;

-- Create a simple, permissive insert policy
CREATE POLICY "Anyone can insert leads" ON leads
    FOR INSERT 
    TO public
    WITH CHECK (true);

-- Create read policy for authenticated users
CREATE POLICY "Authenticated users can read leads" ON leads
    FOR SELECT
    TO authenticated
    USING (true);

-- Make sure RLS is enabled
ALTER TABLE leads ENABLE ROW LEVEL SECURITY;

-- Grant necessary permissions
GRANT INSERT ON leads TO anon;
GRANT INSERT ON leads TO authenticated;
GRANT SELECT ON leads TO authenticated;

-- Test the policy
-- This should return data if policies are working
SELECT schemaname, tablename, policyname, permissive, roles, cmd 
FROM pg_policies 
WHERE tablename = 'leads';

-- Additional debug info
SELECT 
    'Current user:' as info, current_user as value
UNION ALL
SELECT 
    'Has INSERT permission:', 
    has_table_privilege('leads', 'INSERT')::text
UNION ALL
SELECT 
    'Has SELECT permission:', 
    has_table_privilege('leads', 'SELECT')::text;