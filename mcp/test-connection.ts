import { createClient } from "@supabase/supabase-js";
import { config } from 'dotenv';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = dirname(fileURLToPath(import.meta.url));
config({ path: join(dirname(__dirname), '.env.local') });

// Environment setup
const SUPABASE_URL = process.env.VITE_SUPABASE_URL;
const SUPABASE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY;

if (!SUPABASE_URL || !SUPABASE_KEY) {
  console.error("Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY");
  process.exit(1);
}

console.log(`Testing connection to Supabase at ${SUPABASE_URL}`);

const supabase = createClient(SUPABASE_URL, SUPABASE_KEY, { 
  auth: { persistSession: false } 
});

async function testConnection() {
  try {
    // Test 1: Check if we can query profiles table
    console.log("\n1. Testing profiles table...");
    const { data: profiles, error: profileError } = await supabase
      .from('profiles')
      .select('count')
      .limit(1);
    
    if (profileError) {
      console.error("❌ Profiles table error:", profileError);
    } else {
      console.log("✅ Profiles table accessible");
    }

    // Test 2: Check if we can query email_templates
    console.log("\n2. Testing email_templates table...");
    const { data: templates, error: templateError } = await supabase
      .from('email_templates')
      .select('name, subject')
      .limit(5);
    
    if (templateError) {
      console.error("❌ Email templates error:", templateError);
    } else {
      console.log("✅ Email templates found:", templates?.length || 0);
      templates?.forEach(t => console.log(`   - ${t.name}: ${t.subject}`));
    }

    // Test 3: Check tables that might not exist yet
    console.log("\n3. Checking all expected tables...");
    const tables = ['profiles', 'leads', 'queries', 'emails_sent', 'email_templates', 'agent_logs'];
    
    for (const table of tables) {
      const { error } = await supabase.from(table).select('count').limit(1);
      if (error?.code === 'PGRST116') {
        console.log(`⚠️  Table '${table}' does not exist`);
      } else if (error) {
        console.log(`❌ Table '${table}' error:`, error.message);
      } else {
        console.log(`✅ Table '${table}' exists`);
      }
    }

    console.log("\n✅ Connection test complete!");
    
  } catch (error) {
    console.error("\n❌ Connection test failed:", error);
  }
}

testConnection();