-- Fix Backend: Email Templates and RLS Policies

-- 1. Create email_templates table if it doesn't exist
CREATE TABLE IF NOT EXISTS email_templates (
  id UUID DEFAULT gen_random_uuid() PRIMARY KEY,
  name TEXT NOT NULL UNIQUE,
  subject TEXT NOT NULL,
  html_body TEXT NOT NULL,
  text_body TEXT,
  active BOOLEAN DEFAULT true,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT now(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- 2. Insert email templates
INSERT INTO email_templates (name, subject, html_body, text_body) 
VALUES 
  (
    'welcome',
    'Welcome to Opoch - Let''s accelerate your moonshot! 🚀',
    '<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Welcome to Opoch</title>
  <!--[if mso]>
  <style type="text/css">
    .outlook-font {
      font-family: Arial, sans-serif !important;
    }
  </style>
  <![endif]-->
</head>
<body style="margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, ''Helvetica Neue'', Arial, sans-serif; background-color: #000000;">
  <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="background-color: #000000;">
    <tr>
      <td align="center" style="padding: 40px 20px;">
        <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="600" style="max-width: 600px;">
          <!-- Logo Header -->
          <tr>
            <td align="center" style="padding: 40px 0 30px 0;">
              <img src="https://opoch.com/Opoch%20Assets/Opoch%20Typelogo.svg" alt="Opoch" width="150" height="50" style="display: block; border: 0;">
            </td>
          </tr>
          
          <!-- Main Content -->
          <tr>
            <td style="background-color: #0B0F1A; border-radius: 16px; padding: 48px 40px;">
              <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                <tr>
                  <td>
                    <h1 style="margin: 0 0 20px 0; font-size: 28px; line-height: 36px; color: #FFFFFF; font-weight: 600;">
                      Welcome to Opoch, {{name}}! 🚀
                    </h1>
                    <p style="margin: 0 0 24px 0; font-size: 16px; line-height: 24px; color: #E5E5E5;">
                      You''ve just joined a community of deep-tech founders, researchers, and engineers who refuse to accept "impossible" as an answer.
                    </p>
                    
                    <div style="background-color: rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 24px; margin: 0 0 24px 0;">
                      <h2 style="margin: 0 0 16px 0; font-size: 18px; color: #FFFFFF; font-weight: 600;">
                        What happens next?
                      </h2>
                      <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                        <tr>
                          <td style="padding: 0 0 12px 0;">
                            <table role="presentation" cellspacing="0" cellpadding="0" border="0">
                              <tr>
                                <td style="color: #10B981; font-size: 20px; padding-right: 12px;">•</td>
                                <td style="color: #E5E5E5; font-size: 15px; line-height: 22px;">Submit your hardest technical questions through our platform</td>
                              </tr>
                            </table>
                          </td>
                        </tr>
                        <tr>
                          <td style="padding: 0 0 12px 0;">
                            <table role="presentation" cellspacing="0" cellpadding="0" border="0">
                              <tr>
                                <td style="color: #10B981; font-size: 20px; padding-right: 12px;">•</td>
                                <td style="color: #E5E5E5; font-size: 15px; line-height: 22px;">Our team will analyze constraints and show you what''s possible</td>
                              </tr>
                            </table>
                          </td>
                        </tr>
                        <tr>
                          <td>
                            <table role="presentation" cellspacing="0" cellpadding="0" border="0">
                              <tr>
                                <td style="color: #10B981; font-size: 20px; padding-right: 12px;">•</td>
                                <td style="color: #E5E5E5; font-size: 15px; line-height: 22px;">Get a precise plan you can defend and act on</td>
                              </tr>
                            </table>
                          </td>
                        </tr>
                      </table>
                    </div>
                    
                    <!-- CTA Button -->
                    <table role="presentation" cellspacing="0" cellpadding="0" border="0" style="margin: 32px 0;">
                      <tr>
                        <td align="center">
                          <a href="https://opoch.com" style="display: inline-block; padding: 14px 32px; background-color: #FFFFFF; color: #000000; text-decoration: none; font-size: 16px; font-weight: 500; border-radius: 24px;">
                            Start exploring →
                          </a>
                        </td>
                      </tr>
                    </table>
                    
                    <p style="margin: 32px 0 0 0; font-size: 14px; line-height: 20px; color: #999999;">
                      — The Opoch Team
                    </p>
                  </td>
                </tr>
              </table>
            </td>
          </tr>
          
          <!-- Footer -->
          <tr>
            <td style="padding: 32px 0 0 0;">
              <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                <tr>
                  <td align="center" style="padding: 0 0 16px 0;">
                    <p style="margin: 0; font-size: 13px; line-height: 18px; color: #666666;">
                      300 4th St, San Francisco, CA 94107
                    </p>
                  </td>
                </tr>
                <tr>
                  <td align="center">
                    <p style="margin: 0; font-size: 13px; line-height: 18px; color: #666666;">
                      © 2025 Opoch. All rights reserved.
                    </p>
                  </td>
                </tr>
              </table>
            </td>
          </tr>
        </table>
      </td>
    </tr>
  </table>
</body>
</html>',
    'Welcome to Opoch, {{name}}! 🚀

You''ve just joined a community of deep-tech founders, researchers, and engineers who refuse to accept "impossible" as an answer.

What happens next?
• Submit your hardest technical questions through our platform
• Our team will analyze constraints and show you what''s possible
• Get a precise plan you can defend and act on

Ready to accelerate? Visit https://opoch.com

— The Opoch Team

300 4th St, San Francisco, CA 94107
© 2025 Opoch. All rights reserved.'
  ),
  (
    'apply-received',
    'Application Received - Opoch Membership',
    '<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Application Received - Opoch</title>
  <!--[if mso]>
  <style type="text/css">
    .outlook-font {
      font-family: Arial, sans-serif !important;
    }
  </style>
  <![endif]-->
</head>
<body style="margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, ''Helvetica Neue'', Arial, sans-serif; background-color: #000000;">
  <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%" style="background-color: #000000;">
    <tr>
      <td align="center" style="padding: 40px 20px;">
        <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="600" style="max-width: 600px;">
          <!-- Logo Header -->
          <tr>
            <td align="center" style="padding: 40px 0 30px 0;">
              <img src="https://opoch.com/Opoch%20Assets/Opoch%20Typelogo.svg" alt="Opoch" width="150" height="50" style="display: block; border: 0;">
            </td>
          </tr>
          
          <!-- Main Content -->
          <tr>
            <td style="background-color: #0B0F1A; border-radius: 16px; padding: 48px 40px;">
              <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                <tr>
                  <td>
                    <h1 style="margin: 0 0 20px 0; font-size: 28px; line-height: 36px; color: #FFFFFF; font-weight: 600;">
                      Thanks for applying, {{name}}! ✓
                    </h1>
                    <p style="margin: 0 0 24px 0; font-size: 16px; line-height: 24px; color: #E5E5E5;">
                      We received your application for Opoch membership. Here''s what happens next:
                    </p>
                    
                    <!-- Timeline -->
                    <div style="background-color: rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 24px; margin: 0 0 24px 0;">
                      <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                        <tr>
                          <td style="padding: 0 0 16px 0;">
                            <table role="presentation" cellspacing="0" cellpadding="0" border="0">
                              <tr>
                                <td style="color: #10B981; font-size: 16px; font-weight: 600; padding-right: 12px; vertical-align: top;">1.</td>
                                <td>
                                  <span style="color: #FFFFFF; font-size: 15px; font-weight: 500;">Review (1-2 days)</span><br>
                                  <span style="color: #999999; font-size: 14px; line-height: 20px;">We''ll review your application and research query</span>
                                </td>
                              </tr>
                            </table>
                          </td>
                        </tr>
                        <tr>
                          <td style="padding: 0 0 16px 0;">
                            <table role="presentation" cellspacing="0" cellpadding="0" border="0">
                              <tr>
                                <td style="color: #10B981; font-size: 16px; font-weight: 600; padding-right: 12px; vertical-align: top;">2.</td>
                                <td>
                                  <span style="color: #FFFFFF; font-size: 15px; font-weight: 500;">Deep dive scheduling</span><br>
                                  <span style="color: #999999; font-size: 14px; line-height: 20px;">If we''re a good fit, we''ll reach out to schedule a session</span>
                                </td>
                              </tr>
                            </table>
                          </td>
                        </tr>
                        <tr>
                          <td>
                            <table role="presentation" cellspacing="0" cellpadding="0" border="0">
                              <tr>
                                <td style="color: #10B981; font-size: 16px; font-weight: 600; padding-right: 12px; vertical-align: top;">3.</td>
                                <td>
                                  <span style="color: #FFFFFF; font-size: 15px; font-weight: 500;">Delivery</span><br>
                                  <span style="color: #999999; font-size: 14px; line-height: 20px;">You''ll get a clear action plan with next steps</span>
                                </td>
                              </tr>
                            </table>
                          </td>
                        </tr>
                      </table>
                    </div>
                    
                    <!-- Research Query -->
                    <div style="background-color: rgba(16, 185, 129, 0.1); border: 1px solid rgba(16, 185, 129, 0.2); border-radius: 12px; padding: 20px; margin: 0 0 24px 0;">
                      <p style="margin: 0 0 8px 0; font-size: 14px; color: #10B981; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px;">
                        Your research query:
                      </p>
                      <p style="margin: 0; font-size: 15px; line-height: 22px; color: #E5E5E5; font-style: italic;">
                        {{research_query}}
                      </p>
                    </div>
                    
                    <p style="margin: 0 0 24px 0; font-size: 15px; line-height: 22px; color: #E5E5E5;">
                      While you wait, feel free to explore our moonshot projects:
                    </p>
                    
                    <!-- CTA Button -->
                    <table role="presentation" cellspacing="0" cellpadding="0" border="0" style="margin: 0 0 32px 0;">
                      <tr>
                        <td>
                          <a href="https://opoch.com/moonshots" style="display: inline-block; padding: 12px 28px; background-color: transparent; color: #FFFFFF; text-decoration: none; font-size: 15px; font-weight: 500; border: 1px solid rgba(255, 255, 255, 0.2); border-radius: 24px;">
                            Browse moonshots →
                          </a>
                        </td>
                      </tr>
                    </table>
                    
                    <p style="margin: 32px 0 0 0; font-size: 14px; line-height: 20px; color: #999999;">
                      — The Opoch Team
                    </p>
                  </td>
                </tr>
              </table>
            </td>
          </tr>
          
          <!-- Footer -->
          <tr>
            <td style="padding: 32px 0 0 0;">
              <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                <tr>
                  <td align="center" style="padding: 0 0 8px 0;">
                    <p style="margin: 0; font-size: 13px; line-height: 18px; color: #666666;">
                      Application ID: {{lead_id}}
                    </p>
                  </td>
                </tr>
                <tr>
                  <td align="center" style="padding: 0 0 16px 0;">
                    <p style="margin: 0; font-size: 13px; line-height: 18px; color: #666666;">
                      300 4th St, San Francisco, CA 94107
                    </p>
                  </td>
                </tr>
                <tr>
                  <td align="center">
                    <p style="margin: 0; font-size: 13px; line-height: 18px; color: #666666;">
                      © 2025 Opoch. All rights reserved.
                    </p>
                  </td>
                </tr>
              </table>
            </td>
          </tr>
        </table>
      </td>
    </tr>
  </table>
</body>
</html>',
    'Thanks for applying, {{name}}! ✓

We received your application for Opoch membership. Here''s what happens next:

1. Review (1-2 days)
   We''ll review your application and research query

2. Deep dive scheduling
   If we''re a good fit, we''ll reach out to schedule a session

3. Delivery
   You''ll get a clear action plan with next steps

YOUR RESEARCH QUERY:
{{research_query}}

While you wait, feel free to explore our moonshot projects at https://opoch.com/moonshots

— The Opoch Team

Application ID: {{lead_id}}
300 4th St, San Francisco, CA 94107
© 2025 Opoch. All rights reserved.'
  )
ON CONFLICT (name) 
DO UPDATE SET 
  subject = EXCLUDED.subject,
  html_body = EXCLUDED.html_body,
  text_body = EXCLUDED.text_body,
  updated_at = now();

-- 3. Create emails_sent table if it doesn't exist
CREATE TABLE IF NOT EXISTS emails_sent (
  id UUID DEFAULT gen_random_uuid() PRIMARY KEY,
  to_email TEXT NOT NULL,
  template_name TEXT,
  subject TEXT,
  status TEXT NOT NULL CHECK (status IN ('sent', 'failed', 'pending')),
  provider TEXT,
  provider_id TEXT,
  error TEXT,
  metadata JSONB,
  lead_id UUID,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- 4. Fix RLS policies for leads table
ALTER TABLE leads ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if they exist
DROP POLICY IF EXISTS "Enable insert for all users" ON leads;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON leads;

-- Create new policies
CREATE POLICY "Enable insert for all users" ON leads
  FOR INSERT 
  TO anon, authenticated
  WITH CHECK (true);

CREATE POLICY "Enable read for authenticated users" ON leads
  FOR SELECT
  TO authenticated
  USING (true);

-- 5. Fix RLS policies for queries table
ALTER TABLE queries ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if they exist  
DROP POLICY IF EXISTS "Enable insert for authenticated users" ON queries;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON queries;

-- Create new policies
CREATE POLICY "Enable insert for authenticated users" ON queries
  FOR INSERT
  TO authenticated
  WITH CHECK (true);

CREATE POLICY "Enable read for authenticated users" ON queries
  FOR SELECT
  TO authenticated
  USING (true);

-- 6. Fix RLS policies for email_templates table
ALTER TABLE email_templates ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if they exist
DROP POLICY IF EXISTS "Enable read for service role" ON email_templates;

-- Create policy for service role to read templates
CREATE POLICY "Enable read for service role" ON email_templates
  FOR SELECT
  TO service_role
  USING (active = true);

-- 7. Fix RLS policies for emails_sent table
ALTER TABLE emails_sent ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if they exist
DROP POLICY IF EXISTS "Enable insert for service role" ON emails_sent;
DROP POLICY IF EXISTS "Enable read for authenticated users" ON emails_sent;

-- Create policies
CREATE POLICY "Enable insert for service role" ON emails_sent
  FOR INSERT
  TO service_role
  WITH CHECK (true);

CREATE POLICY "Enable read for authenticated users" ON emails_sent
  FOR SELECT
  TO authenticated
  USING (true);

-- 8. Add indexes for better performance
CREATE INDEX IF NOT EXISTS idx_leads_created_at ON leads(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_queries_created_at ON queries(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_emails_sent_created_at ON emails_sent(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_emails_sent_to_email ON emails_sent(to_email);

-- Grant necessary permissions
GRANT ALL ON leads TO anon;
GRANT ALL ON queries TO authenticated;
GRANT SELECT ON email_templates TO service_role;
GRANT ALL ON emails_sent TO service_role;