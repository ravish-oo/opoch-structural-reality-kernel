-- Add query notification email template
INSERT INTO email_templates (name, subject, html_body, text_body) 
VALUES (
  'query-received',
  'We received your query - Opoch',
  '<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Query Received - Opoch</title>
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
              <a href="https://opoch.com" style="text-decoration: none;">
                <img src="https://opoch.com/Opoch%20Assets/Opoch%20Typelogo.png" alt="Opoch" width="150" height="auto" style="display: block; border: 0;">
              </a>
            </td>
          </tr>
          
          <!-- Main Content -->
          <tr>
            <td style="background-color: #0B0F1A; border-radius: 16px; padding: 48px 40px;">
              <table role="presentation" cellspacing="0" cellpadding="0" border="0" width="100%">
                <tr>
                  <td>
                    <h1 style="margin: 0 0 20px 0; font-size: 28px; line-height: 36px; color: #FFFFFF; font-weight: 600;">
                      Got it, {{name}}! 📥
                    </h1>
                    <p style="margin: 0 0 24px 0; font-size: 16px; line-height: 24px; color: #E5E5E5;">
                      We''ve received your technical query and our team is already analyzing it.
                    </p>
                    
                    <!-- Query Display -->
                    <div style="background-color: rgba(16, 185, 129, 0.1); border: 1px solid rgba(16, 185, 129, 0.2); border-radius: 12px; padding: 20px; margin: 0 0 24px 0;">
                      <p style="margin: 0 0 8px 0; font-size: 14px; color: #10B981; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px;">
                        Your query:
                      </p>
                      <p style="margin: 0; font-size: 15px; line-height: 22px; color: #E5E5E5; font-style: italic;">
                        {{query}}
                      </p>
                    </div>
                    
                    <!-- What happens next -->
                    <div style="background-color: rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 24px; margin: 0 0 24px 0;">
                      <h2 style="margin: 0 0 16px 0; font-size: 18px; color: #FFFFFF; font-weight: 600;">
                        What happens next?
                      </h2>
                      <p style="margin: 0 0 12px 0; font-size: 15px; line-height: 22px; color: #E5E5E5;">
                        Our experts are reviewing your query to understand the constraints and identify potential solutions.
                      </p>
                      <p style="margin: 0; font-size: 15px; line-height: 22px; color: #E5E5E5;">
                        We''ll get back to you within 24-48 hours with initial thoughts or follow-up questions.
                      </p>
                    </div>
                    
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
                      Query ID: {{query_id}}
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
  'Got it, {{name}}! 📥

We''ve received your technical query and our team is already analyzing it.

YOUR QUERY:
{{query}}

WHAT HAPPENS NEXT?
Our experts are reviewing your query to understand the constraints and identify potential solutions.

We''ll get back to you within 24-48 hours with initial thoughts or follow-up questions.

— The Opoch Team

Query ID: {{query_id}}
300 4th St, San Francisco, CA 94107
© 2025 Opoch. All rights reserved.'
)
ON CONFLICT (name) 
DO UPDATE SET 
  subject = EXCLUDED.subject,
  html_body = EXCLUDED.html_body,
  text_body = EXCLUDED.text_body,
  updated_at = now();