-- Leads table for capturing form submissions
CREATE TABLE IF NOT EXISTS public.leads (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at timestamptz DEFAULT now(),
  name text NOT NULL,
  email text NOT NULL,
  phone text,
  designation text,
  organization text,
  reason text,            -- "why you want it"
  research_query text,    -- optional research question
  source text,            -- "header-cta" | "hero" | "footer" | "mobile"
  utm jsonb DEFAULT '{}'::jsonb,
  status text DEFAULT 'new' -- new | triage | contacted | closed
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

-- Enable Row Level Security
ALTER TABLE public.leads ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.queries ENABLE ROW LEVEL SECURITY;

-- Policies for anonymous inserts (public can submit forms)
CREATE POLICY "Anyone can insert leads" ON public.leads 
  FOR INSERT WITH CHECK (true);

CREATE POLICY "Anyone can insert queries" ON public.queries 
  FOR INSERT WITH CHECK (true);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_leads_email ON public.leads(email);
CREATE INDEX IF NOT EXISTS idx_leads_created_at ON public.leads(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_leads_status ON public.leads(status);
CREATE INDEX IF NOT EXISTS idx_queries_created_at ON public.queries(created_at DESC);

-- Grant permissions
GRANT INSERT ON public.leads TO anon;
GRANT INSERT ON public.queries TO anon;