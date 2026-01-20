# Email Testing Guide

## What We Fixed

1. **Reply-to Address**: Changed from `dv@zo.xyz` to `hello@opoch.com` in `/api/send-email.ts` (line 103)
2. **Logo Display**: Updated email templates to use PNG logo instead of SVG for better email client compatibility

## Steps to Apply the Fixes

### 1. Run the Email Template Update Script

1. Go to your Supabase dashboard
2. Navigate to SQL Editor
3. Copy and run the entire contents of `/supabase/update-email-templates-logo.sql`
4. You should see a success message showing the templates were updated

### 2. Deploy the API Changes

Since you mentioned Vercel is connected to GitHub:
1. Commit and push the changes to GitHub
2. Vercel will automatically deploy the updated `send-email.ts` with the correct reply-to address

### 3. Test Email Delivery

#### Test Welcome Email
1. Sign out of your application
2. Sign in again from the RBT section (How It Works)
3. Check your email for:
   - Logo displays correctly (PNG image)
   - Reply-to address is `hello@opoch.com`
   - Email formatting looks correct

#### Test Application Email
1. Submit a new application through the Apply form
2. Check your email for:
   - Logo displays correctly
   - Reply-to address is `hello@opoch.com`
   - Your research query is displayed correctly

## Verification Checklist

- [ ] Logo appears in email (should be Opoch Logo.png)
- [ ] Reply-to address is hello@opoch.com (not dv@zo.xyz)
- [ ] Email formatting is preserved
- [ ] All dynamic variables ({{name}}, {{research_query}}) are replaced correctly
- [ ] Links in email work properly

## Troubleshooting

If logo still doesn't appear:
1. Verify the image URL is accessible: https://opoch.com/Opoch%20Assets/Opoch%20Logo.png
2. Check if your email client blocks external images (try "Load Images" option)
3. Test in different email clients (Gmail, Outlook, Apple Mail)

If reply-to is still wrong:
1. Ensure the latest code is deployed to Vercel
2. Check Vercel deployment logs for any errors
3. Verify no environment variables are overriding the reply-to address

## Next Steps

After testing confirms everything works:
1. Re-enable RLS policies (currently disabled as emergency fix)
2. Clean up debug code in `/src/components/ApplyModal.tsx`
3. Remove console.log statements from production code