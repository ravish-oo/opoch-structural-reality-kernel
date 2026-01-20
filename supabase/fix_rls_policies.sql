-- Fix RLS (Row Level Security) policies for all tables

-- Enable RLS on all tables
ALTER TABLE public.leads ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.email_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.emails_sent ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.queries ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.updates ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.newsletter_subscribers ENABLE ROW LEVEL SECURITY;

-- Leads table policies
-- Allow authenticated users to insert leads (for the apply form)
CREATE POLICY "Anyone can insert leads" 
  ON public.leads FOR INSERT 
  WITH CHECK (true);

-- Allow service role to read all leads (for admin)
CREATE POLICY "Service role can read all leads" 
  ON public.leads FOR SELECT 
  USING (auth.jwt() ->> 'role' = 'service_role');

-- Allow authenticated users to read their own leads
CREATE POLICY "Users can read own leads" 
  ON public.leads FOR SELECT 
  USING (email = auth.jwt() ->> 'email');

-- Email templates policies (read-only for everyone, admin can modify)
CREATE POLICY "Anyone can read active email templates" 
  ON public.email_templates FOR SELECT 
  USING (active = true);

CREATE POLICY "Service role can manage email templates" 
  ON public.email_templates FOR ALL 
  USING (auth.jwt() ->> 'role' = 'service_role');

-- Emails sent policies
CREATE POLICY "Service role can read all emails sent" 
  ON public.emails_sent FOR SELECT 
  USING (auth.jwt() ->> 'role' = 'service_role');

CREATE POLICY "Service role can insert emails sent" 
  ON public.emails_sent FOR INSERT 
  WITH CHECK (auth.jwt() ->> 'role' = 'service_role');

-- Queries table policies
CREATE POLICY "Anyone can insert queries" 
  ON public.queries FOR INSERT 
  WITH CHECK (true);

CREATE POLICY "Service role can read all queries" 
  ON public.queries FOR SELECT 
  USING (auth.jwt() ->> 'role' = 'service_role');

-- Updates table policies
CREATE POLICY "Anyone can read published updates" 
  ON public.updates FOR SELECT 
  USING (published = true);

CREATE POLICY "Service role can manage updates" 
  ON public.updates FOR ALL 
  USING (auth.jwt() ->> 'role' = 'service_role');

-- Newsletter subscribers policies
CREATE POLICY "Anyone can subscribe to newsletter" 
  ON public.newsletter_subscribers FOR INSERT 
  WITH CHECK (true);

CREATE POLICY "Service role can read all subscribers" 
  ON public.newsletter_subscribers FOR SELECT 
  USING (auth.jwt() ->> 'role' = 'service_role');

-- Fix the search_path warnings for functions
ALTER FUNCTION public.handle_new_user() SET search_path = public;
ALTER FUNCTION public.update_updated_at_column() SET search_path = public;

-- Add OTP expiry configuration (this should be done in Supabase dashboard Auth settings)
-- For now, we'll just add a comment about it
COMMENT ON SCHEMA public IS 'OTP expiry should be configured to at least 60 seconds in Supabase Auth settings';

-- Grant necessary permissions
GRANT USAGE ON SCHEMA public TO anon, authenticated;
GRANT SELECT ON ALL TABLES IN SCHEMA public TO anon;
GRANT INSERT ON public.leads TO anon, authenticated;
GRANT INSERT ON public.queries TO anon, authenticated;
GRANT INSERT ON public.newsletter_subscribers TO anon, authenticated;