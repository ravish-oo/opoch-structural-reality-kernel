-- ========================================
-- FIX MISSING COLUMNS (RECONCILIATION SCRIPT)
-- Run this BEFORE the emails-sent-table.sql if you get user_id errors
-- ========================================

-- 1) First, inspect which columns are missing
SELECT 
  'Current Column Status' as check_type,
  table_name, 
  array_agg(column_name ORDER BY ordinal_position) as existing_columns
FROM information_schema.columns
WHERE table_schema='public'
  AND table_name IN ('leads','queries','emails_sent')
GROUP BY table_name
ORDER BY table_name;

-- 2) Reconcile LEADS table - add missing columns
DO $$
BEGIN
  IF EXISTS (SELECT 1 FROM information_schema.tables
             WHERE table_schema='public' AND table_name='leads') THEN
    
    -- Add user_id if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='user_id') THEN
      ALTER TABLE public.leads ADD COLUMN user_id UUID;
      ALTER TABLE public.leads
        ADD CONSTRAINT leads_user_id_fkey
        FOREIGN KEY (user_id) REFERENCES auth.users(id) ON DELETE SET NULL;
      RAISE NOTICE 'Added user_id column to leads table';
    END IF;

    -- Add created_at if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='created_at') THEN
      ALTER TABLE public.leads ADD COLUMN created_at TIMESTAMPTZ NOT NULL DEFAULT NOW();
      RAISE NOTICE 'Added created_at column to leads table';
    END IF;

    -- Add other expected columns if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='email') THEN
      ALTER TABLE public.leads ADD COLUMN email TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='name') THEN
      ALTER TABLE public.leads ADD COLUMN name TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='phone') THEN
      ALTER TABLE public.leads ADD COLUMN phone TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='designation') THEN
      ALTER TABLE public.leads ADD COLUMN designation TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='organization') THEN
      ALTER TABLE public.leads ADD COLUMN organization TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='reason') THEN
      ALTER TABLE public.leads ADD COLUMN reason TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='research_query') THEN
      ALTER TABLE public.leads ADD COLUMN research_query TEXT;
    END IF;

    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='leads' AND column_name='source') THEN
      ALTER TABLE public.leads ADD COLUMN source TEXT;
    END IF;
  END IF;
END$$;

-- 3) Reconcile QUERIES table - add missing columns
DO $$
BEGIN
  IF EXISTS (SELECT 1 FROM information_schema.tables
             WHERE table_schema='public' AND table_name='queries') THEN
    
    -- Add user_id if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='queries' AND column_name='user_id') THEN
      ALTER TABLE public.queries ADD COLUMN user_id UUID;
      ALTER TABLE public.queries
        ADD CONSTRAINT queries_user_id_fkey
        FOREIGN KEY (user_id) REFERENCES auth.users(id) ON DELETE CASCADE;
      RAISE NOTICE 'Added user_id column to queries table';
    END IF;

    -- Add created_at if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='queries' AND column_name='created_at') THEN
      ALTER TABLE public.queries ADD COLUMN created_at TIMESTAMPTZ NOT NULL DEFAULT NOW();
      RAISE NOTICE 'Added created_at column to queries table';
    END IF;

    -- Add text if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='queries' AND column_name='text') THEN
      ALTER TABLE public.queries ADD COLUMN "text" TEXT;
      RAISE NOTICE 'Added text column to queries table';
    END IF;

    -- Add context if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='queries' AND column_name='context') THEN
      ALTER TABLE public.queries ADD COLUMN context JSONB NOT NULL DEFAULT '{}'::jsonb;
      RAISE NOTICE 'Added context column to queries table';
    END IF;

    -- Add emailed_at if missing
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema='public' AND table_name='queries' AND column_name='emailed_at') THEN
      ALTER TABLE public.queries ADD COLUMN emailed_at TIMESTAMPTZ;
      RAISE NOTICE 'Added emailed_at column to queries table';
    END IF;
  END IF;
END$$;

-- 4) Re-create RLS policies that reference user_id (after columns are added)
-- Leads policies
ALTER TABLE public.leads ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS "leads_insert_auth" ON public.leads;
DROP POLICY IF EXISTS "leads_select_own"  ON public.leads;

CREATE POLICY "leads_insert_auth" ON public.leads
  FOR INSERT WITH CHECK (auth.role() = 'authenticated');

CREATE POLICY "leads_select_own" ON public.leads
  FOR SELECT USING (
    user_id IS NOT DISTINCT FROM auth.uid()
    OR (email IS NOT NULL AND email = auth.jwt()->>'email')
  );

-- Queries policies  
ALTER TABLE public.queries ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS "queries_insert_owner" ON public.queries;
DROP POLICY IF EXISTS "queries_select_owner" ON public.queries;

-- Auto-fill user_id trigger for queries
CREATE OR REPLACE FUNCTION public.current_uid()
RETURNS UUID 
LANGUAGE SQL 
STABLE 
AS $$
  SELECT (NULLIF(current_setting('request.jwt.claims', true),'')::jsonb->>'sub')::uuid
$$;

CREATE OR REPLACE FUNCTION public.set_query_user_id()
RETURNS TRIGGER 
LANGUAGE plpgsql 
AS $$
BEGIN
  IF NEW.user_id IS NULL THEN
    NEW.user_id := public.current_uid();
  END IF;
  RETURN NEW;
END;
$$;

DROP TRIGGER IF EXISTS trg_queries_set_user ON public.queries;
CREATE TRIGGER trg_queries_set_user
  BEFORE INSERT ON public.queries
  FOR EACH ROW EXECUTE FUNCTION public.set_query_user_id();

-- Relaxed insert policy to allow null user_id (trigger will fill it)
CREATE POLICY "queries_insert_owner" ON public.queries
  FOR INSERT WITH CHECK (COALESCE(user_id, auth.uid()) = auth.uid());

CREATE POLICY "queries_select_owner" ON public.queries
  FOR SELECT USING (user_id = auth.uid());

-- 5) Verify columns were added successfully
SELECT 
  'After Reconciliation' as check_type,
  table_name, 
  array_agg(column_name ORDER BY ordinal_position) as columns_after_fix
FROM information_schema.columns
WHERE table_schema='public'
  AND table_name IN ('leads','queries')
GROUP BY table_name
ORDER BY table_name;

-- 6) Check active policies
SELECT 
  'Active Policies' as check_type,
  schemaname, 
  tablename, 
  policyname, 
  cmd
FROM pg_policies
WHERE schemaname='public'
  AND tablename IN ('leads','queries','emails_sent')
ORDER BY tablename, policyname;

-- PostgREST reload
NOTIFY pgrst, 'reload schema';