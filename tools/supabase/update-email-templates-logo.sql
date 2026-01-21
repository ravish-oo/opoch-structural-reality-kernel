-- Update Email Templates with PNG Logo
-- This fixes the logo display issue in email clients

-- Update welcome email template
UPDATE email_templates 
SET 
  html_body = '<!DOCTYPE html>
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
              <a href="https://opoch.com" style="text-decoration: none;">
                <img src="https://opoch.com/Opoch%20Assets/Opoch%20Logo.png" alt="Opoch" width="150" height="150" style="display: block; border: 0; height: auto;">
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
  updated_at = now()
WHERE name = 'welcome';

-- Update application received email template
UPDATE email_templates 
SET 
  html_body = '<!DOCTYPE html>
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
              <a href="https://opoch.com" style="text-decoration: none;">
                <img src="https://opoch.com/Opoch%20Assets/Opoch%20Logo.png" alt="Opoch" width="150" height="150" style="display: block; border: 0; height: auto;">
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
  updated_at = now()
WHERE name = 'apply-received';

-- Verify the updates
SELECT name, updated_at FROM email_templates WHERE name IN ('welcome', 'apply-received');