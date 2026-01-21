-- Phase 3: Email System Tables

-- Email templates
CREATE TABLE IF NOT EXISTS public.email_templates (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  updated_at timestamptz DEFAULT now(),
  name text NOT NULL UNIQUE, -- e.g., 'welcome', 'apply-received', 'follow-up'
  subject text NOT NULL,
  html_body text NOT NULL,
  text_body text,
  variables jsonb DEFAULT '[]'::jsonb, -- List of required variables
  active boolean DEFAULT true
);

-- Email send log
CREATE TABLE IF NOT EXISTS public.emails_sent (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  to_email text NOT NULL,
  template_name text REFERENCES public.email_templates(name),
  subject text NOT NULL,
  status text NOT NULL DEFAULT 'pending', -- pending | sent | failed | bounced
  provider text, -- 'resend' | 'postmark'
  provider_id text, -- ID from email provider
  metadata jsonb DEFAULT '{}'::jsonb,
  lead_id uuid REFERENCES public.leads(id),
  error text
);

-- Updates/Blog posts for newsletter
CREATE TABLE IF NOT EXISTS public.updates (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  updated_at timestamptz DEFAULT now(),
  published_at timestamptz,
  slug text NOT NULL UNIQUE,
  title text NOT NULL,
  excerpt text,
  content text NOT NULL, -- MDX content
  author text NOT NULL DEFAULT 'Opoch Team',
  tags text[] DEFAULT '{}',
  featured boolean DEFAULT false,
  status text DEFAULT 'draft' -- draft | published | archived
);

-- Newsletter subscribers (separate from leads)
CREATE TABLE IF NOT EXISTS public.newsletter_subscribers (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  email text NOT NULL UNIQUE,
  status text DEFAULT 'active', -- active | unsubscribed | bounced
  unsubscribed_at timestamptz,
  source text, -- 'website' | 'lead-conversion' | 'manual'
  lead_id uuid REFERENCES public.leads(id)
);

-- Phase 4: Auth tables (for future)
CREATE TABLE IF NOT EXISTS public.profiles (
  id uuid PRIMARY KEY REFERENCES auth.users(id),
  created_at timestamptz DEFAULT now(),
  updated_at timestamptz DEFAULT now(),
  lead_id uuid REFERENCES public.leads(id),
  full_name text,
  avatar_url text,
  role text DEFAULT 'member' -- member | admin
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_emails_sent_to_email ON public.emails_sent(to_email);
CREATE INDEX IF NOT EXISTS idx_emails_sent_created_at ON public.emails_sent(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_emails_sent_status ON public.emails_sent(status);
CREATE INDEX IF NOT EXISTS idx_updates_slug ON public.updates(slug);
CREATE INDEX IF NOT EXISTS idx_updates_status ON public.updates(status);
CREATE INDEX IF NOT EXISTS idx_updates_published_at ON public.updates(published_at DESC);
CREATE INDEX IF NOT EXISTS idx_newsletter_subscribers_email ON public.newsletter_subscribers(email);
CREATE INDEX IF NOT EXISTS idx_newsletter_subscribers_status ON public.newsletter_subscribers(status);

-- RLS Policies
ALTER TABLE public.email_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.emails_sent ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.updates ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.newsletter_subscribers ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.profiles ENABLE ROW LEVEL SECURITY;

-- Public can read published updates
CREATE POLICY "Anyone can read published updates" ON public.updates
  FOR SELECT USING (status = 'published' AND published_at <= now());

-- Anyone can subscribe to newsletter
CREATE POLICY "Anyone can subscribe to newsletter" ON public.newsletter_subscribers
  FOR INSERT WITH CHECK (true);

-- Profiles are viewable by owner
CREATE POLICY "Users can view own profile" ON public.profiles
  FOR SELECT USING (auth.uid() = id);

CREATE POLICY "Users can update own profile" ON public.profiles
  FOR UPDATE USING (auth.uid() = id);

-- Functions
CREATE OR REPLACE FUNCTION public.handle_new_user()
RETURNS trigger AS $$
BEGIN
  INSERT INTO public.profiles (id, full_name, avatar_url)
  VALUES (new.id, new.raw_user_meta_data->>'full_name', new.raw_user_meta_data->>'avatar_url');
  RETURN new;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;

-- Trigger for new user creation
CREATE OR REPLACE TRIGGER on_auth_user_created
  AFTER INSERT ON auth.users
  FOR EACH ROW EXECUTE FUNCTION public.handle_new_user();

-- Update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = now();
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_email_templates_updated_at BEFORE UPDATE ON public.email_templates
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_updates_updated_at BEFORE UPDATE ON public.updates
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_profiles_updated_at BEFORE UPDATE ON public.profiles
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();