-- Update all email templates to use PNG typelogo instead of SVG
UPDATE email_templates 
SET 
  html_body = REPLACE(
    html_body,
    'https://opoch.com/Opoch%20Assets/Opoch%20Typelogo.svg',
    'https://opoch.com/Opoch%20Assets/Opoch%20Typelogo.png'
  ),
  updated_at = now()
WHERE html_body LIKE '%Typelogo.svg%';

-- Also update any templates using the regular logo to use typelogo
UPDATE email_templates 
SET 
  html_body = REPLACE(
    html_body,
    '<img src="https://opoch.com/Opoch%20Assets/Opoch%20Logo.png" alt="Opoch" width="150" height="150"',
    '<img src="https://opoch.com/Opoch%20Assets/Opoch%20Typelogo.png" alt="Opoch" width="200" height="auto"'
  ),
  updated_at = now()
WHERE html_body LIKE '%Opoch%20Logo.png%';

-- Verify the updates
SELECT name, 
       CASE 
         WHEN html_body LIKE '%Typelogo.png%' THEN 'Using PNG Typelogo ✓'
         WHEN html_body LIKE '%Logo.png%' THEN 'Using regular Logo'
         ELSE 'No logo found'
       END as logo_status,
       updated_at 
FROM email_templates;