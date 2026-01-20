-- Add test data to verify database connection

-- Test leads
INSERT INTO public.leads (name, email, phone, designation, organization, reason, research_query, source, status)
VALUES 
  ('John Doe', 'john@testcompany.com', '+1234567890', 'CTO', 'Test Corp', 'Looking for ML architecture guidance', 'How to scale our recommendation system to 1B users?', 'admin-test', 'new'),
  ('Jane Smith', 'jane@research.edu', NULL, 'Principal Investigator', 'University Research Lab', 'Need help with distributed computing', 'Optimizing quantum simulation algorithms', 'admin-test', 'new'),
  ('Alice Johnson', 'alice@startup.io', '+9876543210', 'Founder', 'AI Startup', 'Building a new AI product', 'Best practices for training LLMs on custom data', 'hero', 'contacted');

-- Test queries
INSERT INTO public.queries (text, context)
VALUES 
  ('How do I implement distributed caching at scale?', '{"source": "askbox", "page": "homepage"}'),
  ('What is the best approach for real-time data processing?', '{"source": "askbox", "page": "moonshots"}'),
  ('Can you help with WebRTC implementation?', '{"source": "test"}');

-- Test newsletter subscribers
INSERT INTO public.newsletter_subscribers (email, source, status)
VALUES 
  ('newsletter1@example.com', 'website', 'active'),
  ('newsletter2@example.com', 'lead-conversion', 'active'),
  ('unsubscribed@example.com', 'website', 'unsubscribed');

-- Test email activity (assuming templates exist)
INSERT INTO public.emails_sent (to_email, template_name, subject, status, provider, metadata)
VALUES 
  ('john@testcompany.com', 'welcome', 'Welcome to Opoch - Your journey begins', 'sent', 'resend', '{"lead_id": null}'),
  ('jane@research.edu', 'apply-received', 'Application received - Opoch', 'sent', 'resend', '{"research_query": "Optimizing quantum simulation algorithms"}');

-- Summary
SELECT 
  (SELECT COUNT(*) FROM public.leads) as total_leads,
  (SELECT COUNT(*) FROM public.queries) as total_queries,
  (SELECT COUNT(*) FROM public.newsletter_subscribers) as total_subscribers,
  (SELECT COUNT(*) FROM public.emails_sent) as total_emails_sent;