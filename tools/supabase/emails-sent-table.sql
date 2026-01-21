-- ========================================
-- EMAILS_SENT AUDIT TABLE (HARDENED)
-- ========================================

-- 0) UUID support
CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- 1) Create if missing
CREATE TABLE IF NOT EXISTS public.emails_sent (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  to_email TEXT NOT NULL,
  template_name TEXT NOT NULL,
  subject TEXT NOT NULL,
  status TEXT NOT NULL CHECK (status IN ('sent','failed','bounced','delivered')),
  provider TEXT NOT NULL,        -- 'resend', 'postmark', 'ses', etc.
  provider_id TEXT,              -- provider message id
  error TEXT,
  metadata JSONB NOT NULL DEFAULT '{}'::jsonb,
  lead_id UUID REFERENCES public.leads(id) ON DELETE SET NULL,
  user_id UUID REFERENCES auth.users(id) ON DELETE SET NULL
);

-- 2) Reconcile missing columns if the table pre-existed with a different schema
DO $$
BEGIN
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='provider') THEN
    ALTER TABLE public.emails_sent ADD COLUMN provider TEXT;
  END IF;
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='provider_id') THEN
    ALTER TABLE public.emails_sent ADD COLUMN provider_id TEXT;
  END IF;
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='metadata') THEN
    ALTER TABLE public.emails_sent ADD COLUMN metadata JSONB NOT NULL DEFAULT '{}'::jsonb;
  END IF;
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='lead_id') THEN
    ALTER TABLE public.emails_sent ADD COLUMN lead_id UUID REFERENCES public.leads(id) ON DELETE SET NULL;
  END IF;
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                 WHERE table_schema='public' AND table_name='emails_sent' AND column_name='user_id') THEN
    ALTER TABLE public.emails_sent ADD COLUMN user_id UUID REFERENCES auth.users(id) ON DELETE SET NULL;
  END IF;
END$$;

-- 3) Indexes (all idempotent)
CREATE INDEX IF NOT EXISTS idx_emails_sent_to_email       ON public.emails_sent(to_email);
CREATE INDEX IF NOT EXISTS idx_emails_sent_template_name  ON public.emails_sent(template_name);
CREATE INDEX IF NOT EXISTS idx_emails_sent_status         ON public.emails_sent(status);
CREATE INDEX IF NOT EXISTS idx_emails_sent_created_at     ON public.emails_sent(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_emails_sent_lead_id        ON public.emails_sent(lead_id);
CREATE INDEX IF NOT EXISTS idx_emails_sent_user_id        ON public.emails_sent(user_id);

-- 4) Unique-by-provider message id (avoid double-logs on retries)
DO $$
BEGIN
  IF NOT EXISTS (
    SELECT 1 FROM pg_indexes
    WHERE schemaname='public' AND indexname='uniq_emails_sent_provider_msgid'
  ) THEN
    CREATE UNIQUE INDEX uniq_emails_sent_provider_msgid
      ON public.emails_sent(provider, provider_id)
      WHERE provider_id IS NOT NULL;
  END IF;
END$$;

-- 5) RLS
ALTER TABLE public.emails_sent ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS "emails_sent_insert_service" ON public.emails_sent;
DROP POLICY IF EXISTS "emails_sent_select_own"     ON public.emails_sent;

-- NOTE: Using auth.jwt()->>'role' for portability (some projects don't have auth.role())
CREATE POLICY "emails_sent_insert_service" ON public.emails_sent
  FOR INSERT
  WITH CHECK (
    (auth.jwt()->>'role') IN ('service_role','supabase_admin')
  );

CREATE POLICY "emails_sent_select_own" ON public.emails_sent
  FOR SELECT
  USING (
    (auth.jwt()->>'role') IN ('service_role','supabase_admin')
    OR user_id = auth.uid()
    OR to_email = auth.jwt()->>'email'
  );

-- 6) Minimal grants (RLS still controls access)
GRANT USAGE ON SCHEMA public TO authenticated;
GRANT SELECT ON public.emails_sent TO authenticated;

-- 7) Refresh PostgREST cache
NOTIFY pgrst, 'reload schema';

-- ========================================
-- VERIFICATION QUERIES (OPTIONAL)
-- ========================================

-- Uncomment to inspect columns
-- SELECT column_name, data_type 
-- FROM information_schema.columns
-- WHERE table_schema='public' AND table_name='emails_sent'
-- ORDER BY ordinal_position;

-- Uncomment to inspect policies
-- SELECT policyname, cmd, roles 
-- FROM pg_policies
-- WHERE schemaname='public' AND tablename='emails_sent';

-- Uncomment to test insert (from SQL editor uses service_role)
-- INSERT INTO public.emails_sent (to_email, template_name, subject, status, provider, provider_id)
-- VALUES ('test@opoch.com','smoke','hello','sent','resend','test-123');