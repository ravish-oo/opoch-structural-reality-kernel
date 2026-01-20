#!/bin/bash

# Security audit script for Opoch

set -e

echo "🔒 Running security audit..."

# Check for vulnerabilities in dependencies
echo "📦 Checking dependencies for vulnerabilities..."
npm audit || {
    echo "⚠️  Vulnerabilities found in dependencies"
}

# Check for exposed secrets
echo "🔍 Checking for exposed secrets..."
if grep -r "SUPABASE_SERVICE_ROLE_KEY\|RESEND_API_KEY\|sk_\|pk_" --exclude-dir=node_modules --exclude-dir=.git --exclude="*.sh" .; then
    echo "❌ Potential secrets exposed!"
    exit 1
fi

# Check for console.logs in production code
echo "🚨 Checking for console.logs..."
CONSOLE_COUNT=$(grep -r "console\." --include="*.ts" --include="*.tsx" --exclude-dir=node_modules --exclude-dir=.git src/ | wc -l)
if [ $CONSOLE_COUNT -gt 0 ]; then
    echo "⚠️  Found $CONSOLE_COUNT console statements. Consider removing for production."
fi

# Check TypeScript strict mode
echo "📋 Checking TypeScript configuration..."
if ! grep -q '"strict": true' tsconfig.json; then
    echo "⚠️  TypeScript strict mode is not enabled"
fi

# Check for HTTPS enforcement
echo "🔐 Checking HTTPS enforcement..."
if ! grep -q "Strict-Transport-Security" vercel.json; then
    echo "⚠️  HSTS header not configured"
fi

# Check CSP
echo "🛡️ Checking Content Security Policy..."
if ! grep -q "Content-Security-Policy" index.html && ! grep -q "Content-Security-Policy" vercel.json; then
    echo "⚠️  CSP not configured"
fi

# Check for unused dependencies
echo "📊 Checking for unused dependencies..."
npx depcheck --json || true

echo "✅ Security audit complete!"