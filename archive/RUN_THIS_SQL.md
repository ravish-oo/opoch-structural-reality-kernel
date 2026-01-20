# 🚀 Quick Database Setup - 3 Steps

## Go to SQL Editor:
https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/sql/new

## Step 1: Base Tables
Copy ALL content from `supabase/schema.sql` and paste in SQL Editor, then click "Run"

## Step 2: Email System
Copy ALL content from `supabase/schema_phase3.sql` and paste in SQL Editor, then click "Run"

## Step 3: Email Templates
Copy ALL content from `supabase/seed_email_templates.sql` and paste in SQL Editor, then click "Run"

## ✅ Verify It Worked
After running all 3 SQL files, run this command:
```bash
npm run test-supabase
```

You should see "✅ All tests passed!"

## 🎉 Then Test Your Forms!
1. Visit http://localhost:3000/
2. Click "Apply" and submit the form
3. Check your leads table: https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/editor/29152

That's it! Your database is ready for production use.