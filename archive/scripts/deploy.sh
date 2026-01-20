#!/bin/bash

# Opoch Deployment Script
# This script handles the deployment process with safety checks

set -e # Exit on error

echo "🚀 Starting Opoch deployment..."

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
if ! command_exists node; then
    echo "❌ Node.js is not installed"
    exit 1
fi

if ! command_exists npm; then
    echo "❌ npm is not installed"
    exit 1
fi

if ! command_exists vercel; then
    echo "❌ Vercel CLI is not installed. Installing..."
    npm i -g vercel
fi

# Get deployment environment
if [ -z "$1" ]; then
    echo "Usage: ./deploy.sh [preview|production]"
    exit 1
fi

DEPLOY_ENV=$1

# Run tests
echo "🧪 Running tests..."
npm test -- --run || {
    echo "❌ Tests failed. Deployment cancelled."
    exit 1
}

# Type checking
echo "📝 Running type checks..."
npx tsc --noEmit || {
    echo "❌ TypeScript errors found. Deployment cancelled."
    exit 1
}

# Build the project
echo "🔨 Building project..."
npm run build || {
    echo "❌ Build failed. Deployment cancelled."
    exit 1
}

# Check build size
echo "📊 Checking build size..."
MAX_SIZE=5000000 # 5MB
BUILD_SIZE=$(du -sb dist | cut -f1)
if [ $BUILD_SIZE -gt $MAX_SIZE ]; then
    echo "⚠️  Warning: Build size ($BUILD_SIZE bytes) exceeds recommended size"
fi

# Deploy to Vercel
echo "🌐 Deploying to Vercel ($DEPLOY_ENV)..."
if [ "$DEPLOY_ENV" == "production" ]; then
    vercel --prod
else
    vercel
fi

echo "✅ Deployment complete!"