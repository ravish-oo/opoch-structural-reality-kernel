#!/usr/bin/env node

/**
 * Opoch Setup Verification Script
 * Checks if all required environment variables and configurations are in place
 */

import { readFileSync, existsSync } from 'fs';
import { join } from 'path';
import { fileURLToPath } from 'url';
import { dirname } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = join(__dirname, '..');

console.log('🔧 Opoch Setup Verification\n');

// Check for .env.local file
const envPath = join(projectRoot, '.env.local');
const envExists = existsSync(envPath);

console.log('📁 Environment File:');
console.log(`   .env.local: ${envExists ? '✅' : '❌'}`);

if (!envExists) {
  console.log('\n❌ Setup incomplete!');
  console.log('📋 Next steps:');
  console.log('   1. Rename "env.local" to ".env.local"');
  console.log('   2. Update the Supabase API key in .env.local');
  console.log('   3. Run this script again');
  process.exit(1);
}

// Read and validate environment variables
try {
  const envContent = readFileSync(envPath, 'utf8');
  
  console.log('\n🔑 Environment Variables:');
  
  const requiredVars = [
    'VITE_SUPABASE_URL',
    'VITE_SUPABASE_ANON_KEY'
  ];
  
  const optionalVars = [
    'RESEND_API_KEY',
    'SUPABASE_SERVICE_ROLE_KEY'
  ];
  
  const envVars = {};
  envContent.split('\n').forEach(line => {
    if (line.includes('=') && !line.startsWith('#')) {
      const [key, value] = line.split('=');
      envVars[key.trim()] = value.trim();
    }
  });
  
  // Check required variables
  let allRequired = true;
  requiredVars.forEach(varName => {
    const value = envVars[varName];
    const isSet = value && value !== 'your-project-id.supabase.co' && !value.includes('your-');
    console.log(`   ${varName}: ${isSet ? '✅' : '❌'}`);
    if (!isSet) allRequired = false;
  });
  
  // Check optional variables
  console.log('\n🔧 Optional Variables:');
  optionalVars.forEach(varName => {
    const value = envVars[varName];
    const isSet = value && !value.includes('your_') && !value.includes('re_your');
    console.log(`   ${varName}: ${isSet ? '✅' : '⚠️  Not set'}`);
  });
  
  if (!allRequired) {
    console.log('\n❌ Required variables missing!');
    console.log('📋 Fix your .env.local file:');
    console.log('   1. Go to https://supabase.com/dashboard/project/qaoodcvaismvqcydcudx/settings/api');
    console.log('   2. Copy the exact Project URL and anon/public key');
    console.log('   3. Update your .env.local file');
    console.log('   4. Restart your dev server');
    process.exit(1);
  }
  
  // Check database schema files
  console.log('\n🗄️ Database Schema Files:');
  const schemaFiles = ['schema.sql', 'schema_phase3.sql'];
  
  schemaFiles.forEach(file => {
    const filePath = join(projectRoot, 'supabase', file);
    const exists = existsSync(filePath);
    console.log(`   ${file}: ${exists ? '✅' : '❌'}`);
  });
  
  console.log('\n✅ Environment setup looks good!');
  console.log('📋 Next steps:');
  console.log('   1. Run the database schemas in Supabase SQL Editor');
  console.log('   2. Start your dev server: npm run dev');
  console.log('   3. Visit http://localhost:3001/admin/leads to test connection');
  
} catch (error) {
  console.error('❌ Error reading .env.local:', error.message);
  process.exit(1);
}
