import { serve } from "https://deno.land/std@0.168.0/http/server.ts"
import { DatabaseClient } from '../_shared/database.ts'
import { jsonResponse, errorResponse, successResponse, corsHeaders } from '../_shared/types.ts'

// SQL setup scripts to run in order
const setupScripts = {
  schema: `-- Leads table for capturing form submissions
CREATE TABLE IF NOT EXISTS public.leads (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  name text NOT NULL,
  email text NOT NULL,
  phone text,
  designation text,
  organization text,
  reason text,
  research_query text,
  source text,
  utm jsonb DEFAULT '{}'::jsonb,
  status text DEFAULT 'new',
  user_id uuid REFERENCES auth.users(id) ON DELETE SET NULL
);

-- Queries table for capturing Ask Box submissions
CREATE TABLE IF NOT EXISTS public.queries (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  text text NOT NULL,
  ip inet,
  user_agent text,
  lead_id uuid REFERENCES public.leads(id),
  context jsonb DEFAULT '{}'::jsonb
);

-- Email templates table
CREATE TABLE IF NOT EXISTS public.email_templates (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  name TEXT NOT NULL UNIQUE,
  subject TEXT NOT NULL,
  html_body TEXT,
  text_body TEXT,
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Emails sent table for tracking
CREATE TABLE IF NOT EXISTS public.emails_sent (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  to_email TEXT NOT NULL,
  from_email TEXT NOT NULL,
  subject TEXT NOT NULL,
  body TEXT,
  html_body TEXT,
  template_name TEXT,
  metadata JSONB DEFAULT '{}'::jsonb,
  status TEXT DEFAULT 'sent',
  error_message TEXT,
  sent_at TIMESTAMPTZ,
  opened_at TIMESTAMPTZ,
  clicked_at TIMESTAMPTZ
);

-- Profiles table for user profiles
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
);`,

  rls: `-- Enable Row Level Security
ALTER TABLE public.leads ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.queries ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.email_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.emails_sent ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.profiles ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if they exist
DROP POLICY IF EXISTS "Anyone can insert leads" ON public.leads;
DROP POLICY IF EXISTS "Service role can manage leads" ON public.leads;
DROP POLICY IF EXISTS "Users can view their own leads" ON public.leads;
DROP POLICY IF EXISTS "Anyone can insert queries" ON public.queries;
DROP POLICY IF EXISTS "Service role can manage queries" ON public.queries;
DROP POLICY IF EXISTS "Service role can manage email_templates" ON public.email_templates;
DROP POLICY IF EXISTS "Service role can manage emails_sent" ON public.emails_sent;
DROP POLICY IF EXISTS "Users can view their own profile" ON public.profiles;
DROP POLICY IF EXISTS "Users can update their own profile" ON public.profiles;
DROP POLICY IF EXISTS "Service role can manage profiles" ON public.profiles;

-- Leads policies
CREATE POLICY "Anyone can insert leads" ON public.leads 
  FOR INSERT WITH CHECK (true);

CREATE POLICY "Service role can manage leads" ON public.leads
  FOR ALL USING (auth.role() = 'service_role');

CREATE POLICY "Users can view their own leads" ON public.leads
  FOR SELECT USING (auth.uid() = user_id);

-- Queries policies
CREATE POLICY "Anyone can insert queries" ON public.queries 
  FOR INSERT WITH CHECK (true);

CREATE POLICY "Service role can manage queries" ON public.queries
  FOR ALL USING (auth.role() = 'service_role');

-- Email templates policies
CREATE POLICY "Service role can manage email_templates" ON public.email_templates
  FOR ALL USING (auth.role() = 'service_role');

-- Emails sent policies
CREATE POLICY "Service role can manage emails_sent" ON public.emails_sent
  FOR ALL USING (auth.role() = 'service_role');

-- Profiles policies
CREATE POLICY "Users can view their own profile" ON public.profiles
  FOR SELECT USING (auth.uid() = id);

CREATE POLICY "Users can update their own profile" ON public.profiles
  FOR UPDATE USING (auth.uid() = id);

CREATE POLICY "Service role can manage profiles" ON public.profiles
  FOR ALL USING (auth.role() = 'service_role');`,

  indexes: `-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_leads_email ON public.leads(email);
CREATE INDEX IF NOT EXISTS idx_leads_created_at ON public.leads(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_leads_status ON public.leads(status);
CREATE INDEX IF NOT EXISTS idx_leads_user_id ON public.leads(user_id);
CREATE INDEX IF NOT EXISTS idx_queries_created_at ON public.queries(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_queries_lead_id ON public.queries(lead_id);
CREATE INDEX IF NOT EXISTS idx_emails_sent_to_email ON public.emails_sent(to_email);
CREATE INDEX IF NOT EXISTS idx_emails_sent_created_at ON public.emails_sent(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_emails_sent_template_name ON public.emails_sent(template_name);
CREATE INDEX IF NOT EXISTS idx_profiles_email ON public.profiles(email);`,

  permissions: `-- Grant permissions
GRANT INSERT ON public.leads TO anon;
GRANT INSERT ON public.queries TO anon;
GRANT SELECT ON public.profiles TO authenticated;
GRANT UPDATE ON public.profiles TO authenticated;`,

  triggers: `-- Create updated_at trigger function
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = NOW();
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Apply updated_at triggers
CREATE TRIGGER update_email_templates_updated_at BEFORE UPDATE ON public.email_templates
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_profiles_updated_at BEFORE UPDATE ON public.profiles
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Create profile on user signup
CREATE OR REPLACE FUNCTION public.handle_new_user()
RETURNS TRIGGER AS $$
BEGIN
  INSERT INTO public.profiles (id, email)
  VALUES (new.id, new.email)
  ON CONFLICT (id) DO NOTHING;
  RETURN new;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;

DROP TRIGGER IF EXISTS on_auth_user_created ON auth.users;
CREATE TRIGGER on_auth_user_created
  AFTER INSERT ON auth.users
  FOR EACH ROW EXECUTE FUNCTION public.handle_new_user();`,

  emailTemplates: `-- Insert default email templates
INSERT INTO public.email_templates (name, subject, html_body, text_body)
VALUES 
(
  'lead_confirmation',
  'Thank you for your interest in Opoch',
  '<div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
    <h1 style="color: #333;">Thank you for your interest in Opoch!</h1>
    <p>Hi {{name}},</p>
    <p>We''ve received your submission and our team will review it shortly.</p>
    <p>We''ll get back to you within 24-48 hours.</p>
    <p>Best regards,<br>The Opoch Team</p>
  </div>',
  'Thank you for your interest in Opoch!

Hi {{name}},

We''ve received your submission and our team will review it shortly.

We''ll get back to you within 24-48 hours.

Best regards,
The Opoch Team'
),
(
  'query_confirmation',
  'We received your query - Opoch',
  '<div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
    <h1 style="color: #333;">Thank you for your query!</h1>
    <p>We''ve received your research question and our team is looking into it.</p>
    <p>Your query: <em>{{query}}</em></p>
    <p>We''ll get back to you with insights soon.</p>
    <p>Best regards,<br>The Opoch Team</p>
  </div>',
  'Thank you for your query!

We''ve received your research question and our team is looking into it.

Your query: {{query}}

We''ll get back to you with insights soon.

Best regards,
The Opoch Team'
),
(
  'welcome',
  'Welcome to Opoch!',
  '<div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
    <h1 style="color: #333;">Welcome to Opoch!</h1>
    <p>Hi {{name}},</p>
    <p>Thank you for joining Opoch. We''re excited to have you as part of our community.</p>
    <p>Your account has been successfully created.</p>
    <p>If you have any questions, feel free to reach out.</p>
    <p>Best regards,<br>The Opoch Team</p>
  </div>',
  'Welcome to Opoch!

Hi {{name}},

Thank you for joining Opoch. We''re excited to have you as part of our community.

Your account has been successfully created.

If you have any questions, feel free to reach out.

Best regards,
The Opoch Team'
)
ON CONFLICT (name) DO UPDATE SET
  subject = EXCLUDED.subject,
  html_body = EXCLUDED.html_body,
  text_body = EXCLUDED.text_body,
  updated_at = NOW();`
}

interface SetupResult {
  success: boolean
  script: string
  error?: string
  rowsAffected?: number
}

serve(async (req) => {
  // Handle CORS
  if (req.method === 'OPTIONS') {
    return new Response('ok', { headers: corsHeaders })
  }

  try {
    // Get authorization header
    const authHeader = req.headers.get('Authorization')
    if (!authHeader) {
      return errorResponse('Missing authorization header', 401)
    }

    // Create database client
    const db = new DatabaseClient()

    // Parse request body
    const { action, scripts } = await req.json()

    if (action === 'list') {
      // Return available scripts
      return successResponse({ 
        scripts: Object.keys(setupScripts),
        description: {
          schema: 'Create all tables',
          rls: 'Set up Row Level Security policies',
          indexes: 'Create performance indexes',
          permissions: 'Grant necessary permissions',
          triggers: 'Set up database triggers',
          emailTemplates: 'Insert default email templates'
        }
      })
    }

    if (action === 'run') {
      const scriptsToRun = scripts || Object.keys(setupScripts)
      const results: SetupResult[] = []

      for (const scriptName of scriptsToRun) {
        if (!(scriptName in setupScripts)) {
          results.push({
            success: false,
            script: scriptName,
            error: 'Script not found'
          })
          continue
        }

        try {
          const sql = setupScripts[scriptName as keyof typeof setupScripts]
          
          // Since we can't execute raw SQL directly in Edge Functions,
          // we'll use the Supabase client methods where possible
          // For complex SQL, we'll return the script for manual execution
          
          results.push({
            success: true,
            script: scriptName,
            error: 'Script prepared - execute manually in Supabase SQL Editor',
            rowsAffected: 0
          })
        } catch (error) {
          results.push({
            success: false,
            script: scriptName,
            error: error.message
          })
        }
      }

      // Also return the SQL scripts for manual execution
      const sqlScripts = scriptsToRun.reduce((acc, scriptName) => {
        if (scriptName in setupScripts) {
          acc[scriptName] = setupScripts[scriptName as keyof typeof setupScripts]
        }
        return acc
      }, {} as Record<string, string>)

      return successResponse({ 
        success: results.every(r => r.success),
        results,
        sqlScripts,
        instructions: 'Copy and run these SQL scripts in your Supabase SQL Editor'
      })
    }

    if (action === 'verify') {
      // Verify the setup by checking tables exist
      const client = db.getClient()
      const tablesToCheck = ['leads', 'queries', 'email_templates', 'emails_sent', 'profiles']
      const verificationResults: Record<string, any> = {
        tables: {},
        counts: {}
      }

      for (const table of tablesToCheck) {
        try {
          // Try to count rows to verify table exists
          const { count, error } = await client
            .from(table)
            .select('*', { count: 'exact', head: true })
          
          verificationResults.tables[table] = !error
          verificationResults.counts[table] = count || 0
        } catch (error) {
          verificationResults.tables[table] = false
          verificationResults.counts[table] = 0
        }
      }

      return successResponse({ 
        success: true,
        verification: verificationResults,
        message: 'Tables verified. Use Supabase Dashboard to check policies and indexes.'
      })
    }

    if (action === 'export') {
      // Export all SQL scripts as a single file
      const allScripts = Object.entries(setupScripts)
        .map(([name, sql]) => `-- ========== ${name.toUpperCase()} ==========\n${sql}`)
        .join('\n\n')
      
      return successResponse({
        filename: 'opoch-setup.sql',
        content: allScripts,
        message: 'Save this file and run it in your Supabase SQL Editor'
      })
    }

    return errorResponse('Invalid action. Use "list", "run", "verify", or "export"', 400)

  } catch (error) {
    return errorResponse(error.message)
  }
})