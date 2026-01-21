-- Seed email templates

INSERT INTO public.email_templates (name, subject, html_body, text_body, variables) VALUES
(
  'welcome',
  'Welcome to Opoch - Your journey begins',
  '<div style="font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, sans-serif; max-width: 600px; margin: 0 auto;">
    <div style="background: #000; padding: 40px 20px; text-align: center;">
      <h1 style="color: #fff; margin: 0;">Welcome to Opoch</h1>
    </div>
    <div style="background: #f5f5f5; padding: 40px 20px;">
      <p style="font-size: 18px; color: #333;">Hi {{name}},</p>
      <p style="color: #666; line-height: 1.6;">Thank you for applying to Opoch. We''re excited to help you tackle your hardest technical challenges.</p>
      <p style="color: #666; line-height: 1.6;">We''ll review your application within 24 hours and reach out to schedule your first session.</p>
      <p style="color: #666; line-height: 1.6;">In the meantime, feel free to browse our <a href="https://www.opoch.com/moonshots" style="color: #00B0FF;">moonshots</a> to see the kind of problems we love solving.</p>
      <div style="margin: 30px 0;">
        <a href="https://www.opoch.com" style="background: #000; color: #fff; padding: 12px 24px; text-decoration: none; border-radius: 6px; display: inline-block;">Visit Opoch</a>
      </div>
      <p style="color: #999; font-size: 14px;">Best,<br>The Opoch Team</p>
    </div>
  </div>',
  'Hi {{name}},

Thank you for applying to Opoch. We''re excited to help you tackle your hardest technical challenges.

We''ll review your application within 24 hours and reach out to schedule your first session.

In the meantime, feel free to browse our moonshots at https://www.opoch.com/moonshots to see the kind of problems we love solving.

Best,
The Opoch Team',
  '["name"]'::jsonb
),
(
  'apply-received',
  'Application received - Opoch',
  '<div style="font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, sans-serif; max-width: 600px; margin: 0 auto;">
    <div style="background: #000; padding: 40px 20px; text-align: center;">
      <h1 style="color: #fff; margin: 0;">Application Received</h1>
    </div>
    <div style="background: #f5f5f5; padding: 40px 20px;">
      <p style="font-size: 18px; color: #333;">Hi {{name}},</p>
      <p style="color: #666; line-height: 1.6;">We''ve received your application for Opoch membership. Here''s what happens next:</p>
      <ul style="color: #666; line-height: 1.8;">
        <li><strong>Within 24 hours:</strong> We''ll review your application and research question</li>
        <li><strong>If approved:</strong> We''ll schedule a deep dive session (in SF or remote)</li>
        <li><strong>Your first session:</strong> We''ll map out your problem space and identify the shortest path to a solution</li>
      </ul>
      <p style="color: #666; line-height: 1.6;">Your research question: <em>"{{research_query}}"</em></p>
      <p style="color: #666; line-height: 1.6;">We''re looking forward to diving deep on this with you.</p>
      <p style="color: #999; font-size: 14px;">Best,<br>The Opoch Team</p>
    </div>
  </div>',
  'Hi {{name}},

We''ve received your application for Opoch membership. Here''s what happens next:

- Within 24 hours: We''ll review your application and research question
- If approved: We''ll schedule a deep dive session (in SF or remote)
- Your first session: We''ll map out your problem space and identify the shortest path to a solution

Your research question: "{{research_query}}"

We''re looking forward to diving deep on this with you.

Best,
The Opoch Team',
  '["name", "research_query"]'::jsonb
),
(
  'follow-up',
  'Quick question about your Opoch application',
  '<div style="font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, sans-serif; max-width: 600px; margin: 0 auto;">
    <div style="background: #000; padding: 40px 20px; text-align: center;">
      <h1 style="color: #fff; margin: 0; font-size: 24px;">Quick Question</h1>
    </div>
    <div style="background: #f5f5f5; padding: 40px 20px;">
      <p style="font-size: 18px; color: #333;">Hi {{name}},</p>
      <p style="color: #666; line-height: 1.6;">Thanks for your interest in Opoch. To make sure we can deliver maximum value in your first session, we have one question:</p>
      <p style="background: #fff; padding: 20px; border-left: 4px solid #00B0FF; color: #333; margin: 20px 0;">
        <strong>What would a 10× success look like for you?</strong><br>
        <span style="color: #666; font-size: 14px;">Be specific about metrics, timeline, and impact.</span>
      </p>
      <p style="color: #666; line-height: 1.6;">Just reply to this email with your answer. This helps us prepare the right resources and experts for your challenge.</p>
      <p style="color: #999; font-size: 14px;">Best,<br>The Opoch Team</p>
    </div>
  </div>',
  'Hi {{name}},

Thanks for your interest in Opoch. To make sure we can deliver maximum value in your first session, we have one question:

**What would a 10× success look like for you?**
Be specific about metrics, timeline, and impact.

Just reply to this email with your answer. This helps us prepare the right resources and experts for your challenge.

Best,
The Opoch Team',
  '["name"]'::jsonb
),
(
  'meeting-booked',
  'Session confirmed - {{date}} at {{time}}',
  '<div style="font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, sans-serif; max-width: 600px; margin: 0 auto;">
    <div style="background: #000; padding: 40px 20px; text-align: center;">
      <h1 style="color: #fff; margin: 0;">Session Confirmed</h1>
    </div>
    <div style="background: #f5f5f5; padding: 40px 20px;">
      <p style="font-size: 18px; color: #333;">Hi {{name}},</p>
      <p style="color: #666; line-height: 1.6;">Your Opoch session is confirmed!</p>
      <div style="background: #fff; padding: 20px; margin: 20px 0; border-radius: 8px;">
        <p style="margin: 0; color: #333;"><strong>Date:</strong> {{date}}</p>
        <p style="margin: 10px 0 0 0; color: #333;"><strong>Time:</strong> {{time}}</p>
        <p style="margin: 10px 0 0 0; color: #333;"><strong>Location:</strong> {{location}}</p>
      </div>
      <p style="color: #666; line-height: 1.6;"><strong>What to bring:</strong></p>
      <ul style="color: #666; line-height: 1.8;">
        <li>Your dataset or system specifications</li>
        <li>Any constraints (technical, budget, timeline)</li>
        <li>Questions about implementation</li>
      </ul>
      <p style="color: #666; line-height: 1.6;">We''ll spend the session mapping your problem space and identifying the shortest path to a solution.</p>
      <div style="margin: 30px 0;">
        <a href="{{calendar_link}}" style="background: #000; color: #fff; padding: 12px 24px; text-decoration: none; border-radius: 6px; display: inline-block;">Add to Calendar</a>
      </div>
      <p style="color: #999; font-size: 14px;">See you soon,<br>The Opoch Team</p>
    </div>
  </div>',
  'Hi {{name}},

Your Opoch session is confirmed!

Date: {{date}}
Time: {{time}}
Location: {{location}}

What to bring:
- Your dataset or system specifications
- Any constraints (technical, budget, timeline)
- Questions about implementation

We''ll spend the session mapping your problem space and identifying the shortest path to a solution.

See you soon,
The Opoch Team',
  '["name", "date", "time", "location", "calendar_link"]'::jsonb
),
(
  'weekly-update',
  'This week at Opoch: {{headline}}',
  '<div style="font-family: -apple-system, BlinkMacSystemFont, ''Segoe UI'', Roboto, sans-serif; max-width: 600px; margin: 0 auto;">
    <div style="background: #000; padding: 40px 20px; text-align: center;">
      <h1 style="color: #fff; margin: 0; font-size: 28px;">This Week at Opoch</h1>
    </div>
    <div style="background: #f5f5f5; padding: 40px 20px;">
      <p style="font-size: 18px; color: #333;">{{headline}}</p>
      <div style="margin: 30px 0;">{{content}}</div>
      <hr style="border: none; border-top: 1px solid #ddd; margin: 30px 0;">
      <p style="color: #666; font-size: 14px; text-align: center;">
        <a href="https://www.opoch.com/updates" style="color: #00B0FF;">View all updates</a> • 
        <a href="{{unsubscribe_link}}" style="color: #999;">Unsubscribe</a>
      </p>
    </div>
  </div>',
  '{{headline}}

{{content}}

---

View all updates: https://www.opoch.com/updates
Unsubscribe: {{unsubscribe_link}}',
  '["headline", "content", "unsubscribe_link"]'::jsonb
);