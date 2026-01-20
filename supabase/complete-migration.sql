-- Complete Migration Script for Opoch Database
-- This script handles all necessary database setup and fixes

-- 1. Create agent_logs table (required for MCP auditing)
CREATE TABLE IF NOT EXISTS public.agent_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  actor TEXT NOT NULL DEFAULT 'mcp_supabase',
  op TEXT NOT NULL,
  payload JSONB NOT NULL DEFAULT '{}'::jsonb
);

-- Enable RLS on agent_logs
ALTER TABLE public.agent_logs ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if they exist
DROP POLICY IF EXISTS "agent_logs_insert_service" ON public.agent_logs;
DROP POLICY IF EXISTS "agent_logs_select_service" ON public.agent_logs;

-- Create policies for agent_logs
CREATE POLICY "agent_logs_insert_service" ON public.agent_logs
  FOR INSERT
  WITH CHECK ((auth.jwt()->>'role') IN ('service_role','supabase_admin'));

CREATE POLICY "agent_logs_select_service" ON public.agent_logs
  FOR SELECT
  USING ((auth.jwt()->>'role') IN ('service_role','supabase_admin'));

-- Create indexes for agent_logs
CREATE INDEX IF NOT EXISTS agent_logs_created_at_idx ON public.agent_logs(created_at DESC);
CREATE INDEX IF NOT EXISTS agent_logs_op_idx ON public.agent_logs(op);

-- 2. Fix missing columns in existing tables
DO $$ 
BEGIN
  -- Add missing columns to leads table if needed
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='leads' AND column_name='metadata') THEN
    ALTER TABLE public.leads ADD COLUMN metadata JSONB DEFAULT '{}'::jsonb;
  END IF;
  
  -- Add missing columns to queries table if needed
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='queries' AND column_name='response') THEN
    ALTER TABLE public.queries ADD COLUMN response TEXT;
  END IF;
  
  -- Add missing columns to emails_sent table if needed
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='provider') THEN
    ALTER TABLE public.emails_sent ADD COLUMN provider TEXT;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='provider_id') THEN
    ALTER TABLE public.emails_sent ADD COLUMN provider_id TEXT;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='html_body') THEN
    ALTER TABLE public.emails_sent ADD COLUMN html_body TEXT;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='text_body') THEN
    ALTER TABLE public.emails_sent ADD COLUMN text_body TEXT;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='attachments') THEN
    ALTER TABLE public.emails_sent ADD COLUMN attachments JSONB;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='metadata') THEN
    ALTER TABLE public.emails_sent ADD COLUMN metadata JSONB DEFAULT '{}'::jsonb;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='sent_by') THEN
    ALTER TABLE public.emails_sent ADD COLUMN sent_by UUID;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns 
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='error') THEN
    ALTER TABLE public.emails_sent ADD COLUMN error TEXT;
  END IF;
END$$;

-- 3. Create or update RLS policies for all tables

-- Leads table policies
DROP POLICY IF EXISTS "leads_authenticated_all" ON public.leads;
DROP POLICY IF EXISTS "leads_public_insert" ON public.leads;

CREATE POLICY "leads_authenticated_all" ON public.leads
  FOR ALL
  USING ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'))
  WITH CHECK ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'));

CREATE POLICY "leads_public_insert" ON public.leads
  FOR INSERT
  WITH CHECK (true);

-- Queries table policies
DROP POLICY IF EXISTS "queries_public_insert" ON public.queries;
DROP POLICY IF EXISTS "queries_authenticated_all" ON public.queries;

CREATE POLICY "queries_public_insert" ON public.queries
  FOR INSERT
  WITH CHECK (true);

CREATE POLICY "queries_authenticated_all" ON public.queries
  FOR ALL
  USING ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'))
  WITH CHECK ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'));

-- Emails_sent table policies
DROP POLICY IF EXISTS "emails_sent_authenticated_all" ON public.emails_sent;

CREATE POLICY "emails_sent_authenticated_all" ON public.emails_sent
  FOR ALL
  USING ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'))
  WITH CHECK ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'));

-- Email_templates table policies
DROP POLICY IF EXISTS "email_templates_authenticated_read" ON public.email_templates;
DROP POLICY IF EXISTS "email_templates_service_all" ON public.email_templates;

CREATE POLICY "email_templates_authenticated_read" ON public.email_templates
  FOR SELECT
  USING ((auth.jwt()->>'role') IN ('authenticated', 'service_role', 'supabase_admin'));

CREATE POLICY "email_templates_service_all" ON public.email_templates
  FOR ALL
  USING ((auth.jwt()->>'role') IN ('service_role', 'supabase_admin'))
  WITH CHECK ((auth.jwt()->>'role') IN ('service_role', 'supabase_admin'));

-- Profiles table policies
DROP POLICY IF EXISTS "profiles_users_self" ON public.profiles;
DROP POLICY IF EXISTS "profiles_public_read" ON public.profiles;

CREATE POLICY "profiles_users_self" ON public.profiles
  FOR ALL
  USING ((auth.uid() = id) OR ((auth.jwt()->>'role') IN ('service_role', 'supabase_admin')))
  WITH CHECK ((auth.uid() = id) OR ((auth.jwt()->>'role') IN ('service_role', 'supabase_admin')));

CREATE POLICY "profiles_public_read" ON public.profiles
  FOR SELECT
  USING (true);

-- 4. Create indexes for performance
CREATE INDEX IF NOT EXISTS leads_email_idx ON public.leads(email);
CREATE INDEX IF NOT EXISTS leads_status_idx ON public.leads(status);
CREATE INDEX IF NOT EXISTS leads_created_at_idx ON public.leads(created_at DESC);
CREATE INDEX IF NOT EXISTS queries_created_at_idx ON public.queries(created_at DESC);
CREATE INDEX IF NOT EXISTS emails_sent_created_at_idx ON public.emails_sent(created_at DESC);
CREATE INDEX IF NOT EXISTS emails_sent_lead_id_idx ON public.emails_sent(lead_id);

-- 5. Add foreign key constraints if missing
DO $$ 
BEGIN
  -- Add foreign key for emails_sent.sent_by if not exists
  IF NOT EXISTS (SELECT 1 FROM information_schema.table_constraints 
                 WHERE constraint_name = 'emails_sent_sent_by_fkey' 
                 AND table_name = 'emails_sent') THEN
    ALTER TABLE public.emails_sent
      ADD CONSTRAINT emails_sent_sent_by_fkey
      FOREIGN KEY (sent_by) REFERENCES auth.users(id) ON DELETE SET NULL;
  END IF;
END$$;

-- 6. Grant necessary permissions
GRANT USAGE ON SCHEMA public TO anon, authenticated;
GRANT ALL ON ALL TABLES IN SCHEMA public TO anon, authenticated;
GRANT ALL ON ALL SEQUENCES IN SCHEMA public TO anon, authenticated;
GRANT ALL ON ALL FUNCTIONS IN SCHEMA public TO anon, authenticated;

-- Success message
DO $$ 
BEGIN
  RAISE NOTICE 'Migration completed successfully!';
END$$;