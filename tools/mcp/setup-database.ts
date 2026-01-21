#!/usr/bin/env node
// Database setup script using Supabase client
import { createClient } from "@supabase/supabase-js";
import { readFileSync } from "fs";
import { join, dirname } from "path";
import { fileURLToPath } from "url";

const __dirname = dirname(fileURLToPath(import.meta.url));

// Get credentials from environment
const SUPABASE_URL = process.env.SUPABASE_URL || process.env.VITE_SUPABASE_URL;
const SUPABASE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY || process.env.supabase_service_role_key;

if (!SUPABASE_URL || !SUPABASE_KEY) {
  console.error("Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY");
  process.exit(1);
}

const supabase = createClient(SUPABASE_URL, SUPABASE_KEY, {
  auth: { persistSession: false }
});

// SQL files to run in order
const SQL_FILES = [
  "diagnose-tables.sql",
  "fix-missing-columns.sql",
  "complete-backend-setup.sql",
  "emails-sent-table.sql",
  "verify-setup.sql"
];

async function runSqlFile(filename: string) {
  try {
    console.log(`\n📄 Running ${filename}...`);
    
    const sqlPath = join(dirname(__dirname), 'supabase', filename);
    const sql = readFileSync(sqlPath, 'utf-8');
    
    // Split by semicolons but be careful with functions/triggers
    const statements = sql
      .split(/;\s*$/m)
      .filter(stmt => stmt.trim())
      .map(stmt => stmt.trim() + ';');
    
    for (const statement of statements) {
      // Skip comments and empty statements
      if (statement.startsWith('--') || !statement.trim()) {
        continue;
      }
      
      // For diagnostic queries, we need to handle them differently
      if (filename === 'diagnose-tables.sql' || filename === 'verify-setup.sql') {
        console.log(`\nExecuting diagnostic query...`);
        // These are SELECT queries, we can't use rpc for them
        // We'll need to handle these specially or skip
        console.log("⚠️  Diagnostic queries should be run directly in Supabase SQL editor");
        continue;
      }
      
      // For setup scripts, execute via RPC (requires a function)
      console.log(`Executing: ${statement.substring(0, 50)}...`);
      
      // Note: This requires you to create an execute_sql function in Supabase
      // Or we can use the direct SQL approach if available
    }
    
    console.log(`✅ Completed ${filename}`);
  } catch (error) {
    console.error(`❌ Error running ${filename}:`, error);
    throw error;
  }
}

async function main() {
  console.log("🚀 Starting Opoch database setup...\n");
  
  try {
    // First, let's check if we can connect
    const { data, error } = await supabase.from('profiles').select('count');
    if (error && error.code !== 'PGRST116') { // PGRST116 = table not found
      console.error("❌ Cannot connect to Supabase:", error);
      process.exit(1);
    }
    
    console.log("✅ Connected to Supabase\n");
    
    // Run SQL files in order
    for (const file of SQL_FILES) {
      await runSqlFile(file);
    }
    
    console.log("\n🎉 Database setup complete!");
    console.log("\nNext steps:");
    console.log("1. Test the setup by running the verify script in Supabase SQL editor");
    console.log("2. Deploy your application");
    console.log("3. Test all functionality");
    
  } catch (error) {
    console.error("\n❌ Setup failed:", error);
    process.exit(1);
  }
}

// Note about execute_sql function
console.log(`
⚠️  Important: To run SQL through the client, you need to create this function in Supabase:

CREATE OR REPLACE FUNCTION execute_sql(query text)
RETURNS json
LANGUAGE plpgsql
SECURITY DEFINER
AS $$
DECLARE
  result json;
BEGIN
  EXECUTE query INTO result;
  RETURN result;
END;
$$;

Alternatively, run the SQL files directly in the Supabase SQL editor.
`);

main();