-- ========================================
-- OPOCH COMPREHENSIVE BACKEND SETUP (FINAL VERSION)
-- This script includes all improvements and best practices
-- Run this entire script in your Supabase SQL Editor
-- ========================================

-- 0) Enable extensions
CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- ========================================
-- PART 1: RECONCILE EXISTING TABLES
-- ========================================

-- 1) Reconcile email_templates table with correct columns
CREATE TABLE IF NOT EXISTS public.email_templates (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  name TEXT NOT NULL UNIQUE,
  subject TEXT NOT NULL,
  html_body TEXT,
  text_body TEXT,
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Add missing columns to email_templates if needed
DO $$
BEGIN
  -- Add html_body if missing
  IF NOT EXISTS (
    SELECT 1 FROM information_schema.columns
    WHERE table_schema='public' AND table_name='email_templates' AND column_name='html_body'
  ) THEN
    ALTER TABLE public.email_templates ADD COLUMN html_body TEXT;
  END IF;

  -- Add text_body if missing
  IF NOT EXISTS (
    SELECT 1 FROM information_schema.columns
    WHERE table_schema='public' AND table_name='email_templates' AND column_name='text_body'
  ) THEN
    ALTER TABLE public.email_templates ADD COLUMN text_body TEXT;
  END IF;

  -- If legacy 'body' column exists, migrate data then drop it
  IF EXISTS (
    SELECT 1 FROM information_schema.columns
    WHERE table_schema='public' AND table_name='email_templates' AND column_name='body'
  ) THEN
    UPDATE public.email_templates
    SET 
      html_body = COALESCE(html_body, body),
      text_body = COALESCE(text_body, regexp_replace(body, '<[^>]+>', '', 'g'));
    -- Drop the body column after migration
    ALTER TABLE public.email_templates DROP COLUMN body;
  END IF;
END$$;

-- 2) Reconcile profiles table
CREATE TABLE IF NOT EXISTS public.profiles (
  id UUID PRIMARY KEY REFERENCES auth.users(id) ON DELETE CASCADE,
  email TEXT,
  full_name TEXT,
  phone TEXT,
  designation TEXT,
  organization TEXT,
  reason TEXT,
  has_completed_profile BOOLEAN DEFAULT FALSE,
  welcomed_at TIMESTAMPTZ,
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Add missing columns to profiles if table already exists
DO $$
BEGIN
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='profiles' AND column_name='email') THEN
    ALTER TABLE public.profiles ADD COLUMN email TEXT;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='profiles' AND column_name='welcomed_at') THEN
    ALTER TABLE public.profiles ADD COLUMN welcomed_at TIMESTAMPTZ;
  END IF;
  
  IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='profiles' AND column_name='has_completed_profile') THEN
    ALTER TABLE public.profiles ADD COLUMN has_completed_profile BOOLEAN DEFAULT FALSE;
  END IF;
END$$;

-- 3) Reconcile leads table
CREATE TABLE IF NOT EXISTS public.leads (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  user_id UUID REFERENCES auth.users(id) ON DELETE SET NULL,
  email TEXT,
  name TEXT,
  phone TEXT,
  designation TEXT,
  organization TEXT,
  reason TEXT,
  research_query TEXT,
  source TEXT
);

-- Add missing columns to leads if table already exists
DO $$
BEGIN
  IF EXISTS (SELECT 1 FROM information_schema.tables WHERE table_schema='public' AND table_name='leads') THEN
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='user_id') THEN
      ALTER TABLE public.leads ADD COLUMN user_id UUID;
      ALTER TABLE public.leads ADD CONSTRAINT leads_user_id_fkey FOREIGN KEY (user_id) REFERENCES auth.users(id) ON DELETE SET NULL;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='created_at') THEN
      ALTER TABLE public.leads ADD COLUMN created_at TIMESTAMPTZ NOT NULL DEFAULT NOW();
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='email') THEN
      ALTER TABLE public.leads ADD COLUMN email TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='name') THEN
      ALTER TABLE public.leads ADD COLUMN name TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='phone') THEN
      ALTER TABLE public.leads ADD COLUMN phone TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='designation') THEN
      ALTER TABLE public.leads ADD COLUMN designation TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='organization') THEN
      ALTER TABLE public.leads ADD COLUMN organization TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='reason') THEN
      ALTER TABLE public.leads ADD COLUMN reason TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='research_query') THEN
      ALTER TABLE public.leads ADD COLUMN research_query TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='leads' AND column_name='source') THEN
      ALTER TABLE public.leads ADD COLUMN source TEXT;
    END IF;
  END IF;
END$$;

-- 4) Reconcile queries table
CREATE TABLE IF NOT EXISTS public.queries (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  user_id UUID REFERENCES auth.users(id) ON DELETE CASCADE,
  text TEXT NOT NULL,
  context JSONB NOT NULL DEFAULT '{}'::jsonb,
  emailed_at TIMESTAMPTZ
);

-- Add missing columns to queries if table already exists
DO $$
BEGIN
  IF EXISTS (SELECT 1 FROM information_schema.tables WHERE table_schema='public' AND table_name='queries') THEN
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='queries' AND column_name='user_id') THEN
      ALTER TABLE public.queries ADD COLUMN user_id UUID;
      ALTER TABLE public.queries ADD CONSTRAINT queries_user_id_fkey FOREIGN KEY (user_id) REFERENCES auth.users(id) ON DELETE CASCADE;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='queries' AND column_name='created_at') THEN
      ALTER TABLE public.queries ADD COLUMN created_at TIMESTAMPTZ NOT NULL DEFAULT NOW();
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='queries' AND column_name='text') THEN
      ALTER TABLE public.queries ADD COLUMN text TEXT;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='queries' AND column_name='context') THEN
      ALTER TABLE public.queries ADD COLUMN context JSONB NOT NULL DEFAULT '{}'::jsonb;
    END IF;
    
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns WHERE table_schema='public' AND table_name='queries' AND column_name='emailed_at') THEN
      ALTER TABLE public.queries ADD COLUMN emailed_at TIMESTAMPTZ;
    END IF;
  END IF;
END$$;

-- ========================================
-- PART 2: SETUP TRIGGERS AND FUNCTIONS
-- ========================================

-- 5) Create updated_at trigger function
CREATE OR REPLACE FUNCTION public.touch_updated_at()
RETURNS TRIGGER LANGUAGE plpgsql AS $$
BEGIN 
  NEW.updated_at = NOW(); 
  RETURN NEW; 
END;
$$;

-- 6) Add updated_at triggers
DROP TRIGGER IF EXISTS trg_profiles_touch ON public.profiles;
CREATE TRIGGER trg_profiles_touch BEFORE UPDATE ON public.profiles
  FOR EACH ROW EXECUTE FUNCTION public.touch_updated_at();

DROP TRIGGER IF EXISTS trg_email_templates_touch ON public.email_templates;
CREATE TRIGGER trg_email_templates_touch BEFORE UPDATE ON public.email_templates
  FOR EACH ROW EXECUTE FUNCTION public.touch_updated_at();

-- 7) Create handle_new_user function with UPSERT
CREATE OR REPLACE FUNCTION public.handle_new_user()
RETURNS TRIGGER
LANGUAGE plpgsql
SECURITY DEFINER SET search_path = public
AS $$
BEGIN
  INSERT INTO public.profiles (id, email, full_name, updated_at)
  VALUES (
    NEW.id,
    NEW.email,
    COALESCE(NEW.raw_user_meta_data->>'full_name', NEW.raw_user_meta_data->>'name', ''),
    NOW()
  )
  ON CONFLICT (id) DO UPDATE SET
    email = EXCLUDED.email,
    full_name = COALESCE(EXCLUDED.full_name, profiles.full_name),
    updated_at = NOW();
  
  RETURN NEW;
END;
$$;

-- 8) Create auth trigger
DROP TRIGGER IF EXISTS on_auth_user_created ON auth.users;
CREATE TRIGGER on_auth_user_created
  AFTER INSERT ON auth.users
  FOR EACH ROW EXECUTE FUNCTION public.handle_new_user();

-- 9) Helper function to get current user ID from JWT
CREATE OR REPLACE FUNCTION public.current_uid()
RETURNS UUID 
LANGUAGE SQL 
STABLE 
AS $$
  SELECT (NULLIF(current_setting('request.jwt.claims', true),'')::jsonb->>'sub')::uuid
$$;

-- 10) Auto-fill user_id for queries
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

-- ========================================
-- PART 3: ENABLE RLS AND CREATE POLICIES
-- ========================================

-- 11) Enable RLS on all tables
ALTER TABLE public.profiles ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.email_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.leads ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.queries ENABLE ROW LEVEL SECURITY;

-- 12) Drop existing policies explicitly
DROP POLICY IF EXISTS "profiles_select_own" ON public.profiles;
DROP POLICY IF EXISTS "profiles_insert_own" ON public.profiles;
DROP POLICY IF EXISTS "profiles_update_own" ON public.profiles;
DROP POLICY IF EXISTS "email_templates_read" ON public.email_templates;
DROP POLICY IF EXISTS "leads_insert_auth" ON public.leads;
DROP POLICY IF EXISTS "leads_select_own" ON public.leads;
DROP POLICY IF EXISTS "queries_insert_owner" ON public.queries;
DROP POLICY IF EXISTS "queries_select_owner" ON public.queries;

-- 13) Create RLS policies for profiles
CREATE POLICY "profiles_select_own" ON public.profiles
  FOR SELECT USING (auth.uid() = id);

CREATE POLICY "profiles_insert_own" ON public.profiles
  FOR INSERT WITH CHECK (auth.uid() = id);

CREATE POLICY "profiles_update_own" ON public.profiles
  FOR UPDATE USING (auth.uid() = id) WITH CHECK (auth.uid() = id);

-- 14) Create RLS policies for email_templates (authenticated only)
CREATE POLICY "email_templates_read" ON public.email_templates
  FOR SELECT USING (auth.role() IN ('authenticated', 'service_role', 'anon'));

-- 15) Create RLS policies for leads
CREATE POLICY "leads_insert_auth" ON public.leads
  FOR INSERT WITH CHECK (auth.role() = 'authenticated');

CREATE POLICY "leads_select_own" ON public.leads
  FOR SELECT USING (
    (user_id IS NOT DISTINCT FROM auth.uid())
    OR (email IS NOT NULL AND email = auth.jwt()->>'email')
  );

-- 16) Create RLS policies for queries (relaxed insert to allow null user_id)
CREATE POLICY "queries_insert_owner" ON public.queries
  FOR INSERT WITH CHECK (COALESCE(user_id, auth.uid()) = auth.uid());

CREATE POLICY "queries_select_owner" ON public.queries
  FOR SELECT USING (user_id = auth.uid());

-- ========================================
-- PART 4: PERMISSIONS AND INDEXES
-- ========================================

-- 17) Grant appropriate permissions (no SELECT on profiles for anon)
GRANT USAGE ON SCHEMA public TO anon, authenticated;
GRANT SELECT ON public.email_templates TO authenticated;
GRANT SELECT, INSERT, UPDATE ON public.profiles TO authenticated;
GRANT SELECT, INSERT ON public.leads TO authenticated;
GRANT SELECT, INSERT ON public.queries TO authenticated;

-- Revoke unnecessary permissions
REVOKE SELECT ON public.profiles FROM anon;
REVOKE SELECT ON public.email_templates FROM anon;

-- 18) Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_profiles_email ON public.profiles(email);
CREATE INDEX IF NOT EXISTS idx_leads_email ON public.leads(email);
CREATE INDEX IF NOT EXISTS idx_leads_user_id ON public.leads(user_id);
CREATE INDEX IF NOT EXISTS idx_leads_created_at ON public.leads(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_queries_user_id ON public.queries(user_id);
CREATE INDEX IF NOT EXISTS idx_queries_created_at ON public.queries(created_at DESC);

-- ========================================
-- PART 5: EMAIL TEMPLATES (USING html_body AND text_body)
-- ========================================

-- 19) Insert/Update email templates with PNG typelogo
INSERT INTO public.email_templates (name, subject, html_body, text_body)
VALUES 
  (
    'welcome',
    'Welcome to Opoch - Age of Truth',
    -- HTML version
    '<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Welcome to Opoch</title>
</head>
<body style="margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, ''Helvetica Neue'', Arial, sans-serif; background-color: #0B0F1A;">
    <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="background-color: #0B0F1A;">
        <tr>
            <td align="center" style="padding: 40px 20px;">
                <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="max-width: 600px; background-color: #1A1F2E; border-radius: 16px; overflow: hidden;">
                    <tr>
                        <td align="center" style="padding: 40px 20px 20px;">
                            <img src="https://www.opoch.com/Opoch%20Assets/Opoch%20Typelogo.png" alt="Opoch" width="150" style="display: block; margin: 0 auto;">
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 20px 40px;">
                            <h1 style="margin: 0 0 20px; font-size: 28px; font-weight: 600; color: #FFFFFF; text-align: center;">
                                Welcome to the Age of Truth
                            </h1>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                {{user_name}},
                            </p>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                You''ve just taken the first step into a new era of technical consulting. Opoch isn''t just another consultancy – we''re your partners in navigating the most complex technical challenges.
                            </p>
                            <p style="margin: 0 0 30px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                Whether you''re architecting distributed systems, implementing AI/ML solutions, or solving deep technical problems, we''re here to deliver clarity and results without the fluff.
                            </p>
                            <div style="margin: 30px 0; padding: 20px; background-color: #0B0F1A; border-radius: 12px;">
                                <h2 style="margin: 0 0 15px; font-size: 20px; font-weight: 600; color: #FFFFFF;">
                                    What''s Next?
                                </h2>
                                <ul style="margin: 0; padding-left: 20px; color: #E0E0E0;">
                                    <li style="margin-bottom: 10px; font-size: 16px;">Submit your technical query or challenge</li>
                                    <li style="margin-bottom: 10px; font-size: 16px;">Apply for membership to unlock full access</li>
                                    <li style="margin-bottom: 10px; font-size: 16px;">Explore our moonshot projects</li>
                                </ul>
                            </div>
                            <div style="text-align: center; margin: 30px 0;">
                                <a href="https://www.opoch.com" style="display: inline-block; padding: 14px 32px; font-size: 16px; font-weight: 600; color: #000000; background-color: #FFFFFF; text-decoration: none; border-radius: 50px; transition: all 0.3s ease;">
                                    Get Started
                                </a>
                            </div>
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 20px 40px 30px; border-top: 1px solid #2A2F3E;">
                            <p style="margin: 0; font-size: 14px; color: #A0A0A0; text-align: center;">
                                Questions? Reach out at 
                                <a href="mailto:hello@opoch.com" style="color: #FFFFFF; text-decoration: none;">hello@opoch.com</a>
                            </p>
                            <p style="margin: 10px 0 0; font-size: 14px; color: #A0A0A0; text-align: center;">
                                300 4th St, San Francisco, CA 94107
                            </p>
                        </td>
                    </tr>
                </table>
            </td>
        </tr>
    </table>
</body>
</html>',
    -- Plain text version
    'Welcome to Opoch - Age of Truth

{{user_name}},

You''ve just taken the first step into a new era of technical consulting. Opoch isn''t just another consultancy – we''re your partners in navigating the most complex technical challenges.

Whether you''re architecting distributed systems, implementing AI/ML solutions, or solving deep technical problems, we''re here to deliver clarity and results without the fluff.

What''s Next?
• Submit your technical query or challenge
• Apply for membership to unlock full access
• Explore our moonshot projects

Get Started: https://www.opoch.com

Questions? Reach out at hello@opoch.com
300 4th St, San Francisco, CA 94107'
  ),
  (
    'lead_confirmation',
    'Application Received - Opoch',
    -- HTML version
    '<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Application Received</title>
</head>
<body style="margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, ''Helvetica Neue'', Arial, sans-serif; background-color: #0B0F1A;">
    <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="background-color: #0B0F1A;">
        <tr>
            <td align="center" style="padding: 40px 20px;">
                <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="max-width: 600px; background-color: #1A1F2E; border-radius: 16px; overflow: hidden;">
                    <tr>
                        <td align="center" style="padding: 40px 20px 20px;">
                            <img src="https://www.opoch.com/Opoch%20Assets/Opoch%20Typelogo.png" alt="Opoch" width="150" style="display: block; margin: 0 auto;">
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 20px 40px;">
                            <h1 style="margin: 0 0 20px; font-size: 28px; font-weight: 600; color: #FFFFFF; text-align: center;">
                                Application Received
                            </h1>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                Thank you for applying to Opoch. We''ve received your application and are excited to learn about your technical challenges.
                            </p>
                            <div style="margin: 30px 0; padding: 20px; background-color: #0B0F1A; border-radius: 12px;">
                                <h2 style="margin: 0 0 15px; font-size: 18px; font-weight: 600; color: #FFFFFF;">
                                    Your Application Details:
                                </h2>
                                <p style="margin: 0 0 10px; font-size: 16px; color: #E0E0E0;">
                                    <strong>Name:</strong> {{lead_name}}
                                </p>
                                <p style="margin: 0 0 10px; font-size: 16px; color: #E0E0E0;">
                                    <strong>Organization:</strong> {{lead_organization}}
                                </p>
                                <p style="margin: 0 0 10px; font-size: 16px; color: #E0E0E0;">
                                    <strong>Designation:</strong> {{lead_designation}}
                                </p>
                            </div>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                Our team will review your application and respond within 24 hours. We''re looking forward to understanding how we can help you navigate your technical challenges.
                            </p>
                            <div style="text-align: center; margin: 30px 0;">
                                <a href="https://www.opoch.com/moonshots" style="display: inline-block; padding: 14px 32px; font-size: 16px; font-weight: 600; color: #000000; background-color: #FFFFFF; text-decoration: none; border-radius: 50px; transition: all 0.3s ease;">
                                    Explore Moonshots
                                </a>
                            </div>
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 20px 40px 30px; border-top: 1px solid #2A2F3E;">
                            <p style="margin: 0; font-size: 14px; color: #A0A0A0; text-align: center;">
                                Questions? Reach out at 
                                <a href="mailto:hello@opoch.com" style="color: #FFFFFF; text-decoration: none;">hello@opoch.com</a>
                            </p>
                            <p style="margin: 10px 0 0; font-size: 14px; color: #A0A0A0; text-align: center;">
                                300 4th St, San Francisco, CA 94107
                            </p>
                        </td>
                    </tr>
                </table>
            </td>
        </tr>
    </table>
</body>
</html>',
    -- Plain text version
    'Application Received - Opoch

Thank you for applying to Opoch. We''ve received your application and are excited to learn about your technical challenges.

Your Application Details:
• Name: {{lead_name}}
• Organization: {{lead_organization}}
• Designation: {{lead_designation}}

Our team will review your application and respond within 24 hours. We''re looking forward to understanding how we can help you navigate your technical challenges.

Explore Moonshots: https://www.opoch.com/moonshots

Questions? Reach out at hello@opoch.com
300 4th St, San Francisco, CA 94107'
  ),
  (
    'query_confirmation',
    'We received your query - Opoch',
    -- HTML version
    '<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Query Received</title>
</head>
<body style="margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, ''Helvetica Neue'', Arial, sans-serif; background-color: #0B0F1A;">
    <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="background-color: #0B0F1A;">
        <tr>
            <td align="center" style="padding: 40px 20px;">
                <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="max-width: 600px; background-color: #1A1F2E; border-radius: 16px; overflow: hidden;">
                    <tr>
                        <td align="center" style="padding: 40px 20px 20px;">
                            <img src="https://www.opoch.com/Opoch%20Assets/Opoch%20Typelogo.png" alt="Opoch" width="150" style="display: block; margin: 0 auto;">
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 20px 40px;">
                            <h1 style="margin: 0 0 20px; font-size: 28px; font-weight: 600; color: #FFFFFF; text-align: center;">
                                Query Received
                            </h1>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                Hi {{user_name}},
                            </p>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                We''ve received your technical query and our team is reviewing it. You can expect a detailed response within 24 hours.
                            </p>
                            <div style="margin: 30px 0; padding: 20px; background-color: #0B0F1A; border-radius: 12px;">
                                <h2 style="margin: 0 0 15px; font-size: 18px; font-weight: 600; color: #FFFFFF;">
                                    Your Query:
                                </h2>
                                <p style="margin: 0; font-size: 16px; line-height: 1.6; color: #E0E0E0; white-space: pre-wrap;">{{query_content}}</p>
                            </div>
                            <p style="margin: 0 0 20px; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                Our experts are analyzing your question and will provide actionable insights tailored to your specific challenge.
                            </p>
                            <div style="margin: 30px 0; padding: 20px; background-color: #2A2F3E; border-radius: 12px; border-left: 4px solid #FFFFFF;">
                                <p style="margin: 0; font-size: 16px; line-height: 1.6; color: #E0E0E0;">
                                    <strong>Pro tip:</strong> While you wait, explore our moonshot projects to see how we''re tackling some of the world''s most ambitious technical challenges.
                                </p>
                            </div>
                            <div style="text-align: center; margin: 30px 0;">
                                <a href="https://www.opoch.com/moonshots" style="display: inline-block; padding: 14px 32px; font-size: 16px; font-weight: 600; color: #000000; background-color: #FFFFFF; text-decoration: none; border-radius: 50px; transition: all 0.3s ease;">
                                    Explore Moonshots
                                </a>
                            </div>
                        </td>
                    </tr>
                    <tr>
                        <td style="padding: 20px 40px 30px; border-top: 1px solid #2A2F3E;">
                            <p style="margin: 0; font-size: 14px; color: #A0A0A0; text-align: center;">
                                Need immediate assistance? Reach out at 
                                <a href="mailto:hello@opoch.com" style="color: #FFFFFF; text-decoration: none;">hello@opoch.com</a>
                            </p>
                            <p style="margin: 10px 0 0; font-size: 14px; color: #A0A0A0; text-align: center;">
                                300 4th St, San Francisco, CA 94107
                            </p>
                        </td>
                    </tr>
                </table>
            </td>
        </tr>
    </table>
</body>
</html>',
    -- Plain text version
    'We received your query - Opoch

Hi {{user_name}},

We''ve received your technical query and our team is reviewing it. You can expect a detailed response within 24 hours.

Your Query:
{{query_content}}

Our experts are analyzing your question and will provide actionable insights tailored to your specific challenge.

Pro tip: While you wait, explore our moonshot projects to see how we''re tackling some of the world''s most ambitious technical challenges.

Explore Moonshots: https://www.opoch.com/moonshots

Need immediate assistance? Reach out at hello@opoch.com
300 4th St, San Francisco, CA 94107'
  )
ON CONFLICT (name) DO UPDATE SET
  subject = EXCLUDED.subject,
  html_body = EXCLUDED.html_body,
  text_body = EXCLUDED.text_body,
  updated_at = NOW();

-- 20) Refresh schema for PostgREST
NOTIFY pgrst, 'reload schema';

-- ========================================
-- VERIFICATION QUERIES
-- ========================================

-- Check email_templates structure
SELECT 
  column_name,
  data_type,
  is_nullable
FROM information_schema.columns
WHERE table_schema = 'public' 
  AND table_name = 'email_templates'
ORDER BY ordinal_position;

-- Check that templates were inserted correctly
SELECT 
  name,
  subject,
  LENGTH(html_body) AS html_length,
  LENGTH(text_body) AS text_length,
  created_at
FROM public.email_templates
ORDER BY name;

-- Check all table columns to verify everything is correct
SELECT 
  table_name,
  column_name,
  data_type
FROM information_schema.columns
WHERE table_schema = 'public' 
  AND table_name IN ('profiles', 'leads', 'queries', 'email_templates')
ORDER BY table_name, ordinal_position;

-- Check RLS policies
SELECT 
  schemaname,
  tablename,
  policyname,
  permissive,
  roles,
  cmd
FROM pg_policies
WHERE schemaname = 'public' 
  AND tablename IN ('profiles', 'email_templates', 'leads', 'queries')
ORDER BY tablename, policyname;

-- Check triggers
SELECT 
  tgname,
  tgtype::int,
  tgrelid::regclass::text AS table_name
FROM pg_trigger
WHERE tgrelid IN ('auth.users'::regclass, 'public.queries'::regclass, 'public.profiles'::regclass)
ORDER BY table_name, tgname;

-- ========================================
-- IMPORTANT NOTES:
-- 1. This script includes all recommended improvements
-- 2. Email templates use html_body and text_body columns
-- 3. Queries auto-fill user_id via trigger
-- 4. No SELECT on profiles for anon users
-- 5. Email templates restricted to authenticated users
-- 6. All tables have proper indexes for performance
-- ========================================