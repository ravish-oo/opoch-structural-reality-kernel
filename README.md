# Opoch - Technical Consulting Platform

Get unstuck. Build faster. Decide with confidence.

## Overview

Opoch is a modern technical consulting platform built with React, TypeScript, and Vite. It features a moonshots catalog, lead capture system, email automation, and comprehensive analytics.

## Features

- 🚀 **Moonshots Catalog**: Browse ambitious technical challenges
- 📧 **Email System**: Automated email templates with Resend integration
- 🔐 **Authentication**: Google and GitHub SSO via Supabase Auth
- 📊 **Analytics**: Vercel Analytics and custom event tracking
- ⚡ **Performance**: Lazy loading, PWA support, and optimized assets
- 🛡️ **Security**: CSP, security headers, and best practices

## Tech Stack

- **Frontend**: React 18, TypeScript, Vite
- **Styling**: Tailwind CSS, Framer Motion
- **Database**: Supabase (PostgreSQL)
- **Email**: Resend
- **Deployment**: Vercel
- **Analytics**: Vercel Analytics & Speed Insights

## Getting Started

### Prerequisites

- Node.js 20.9+ (we use 20.19.4 LTS)
- npm 10+
- Supabase account
- Resend account (for emails)
- Vercel account (for deployment)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/opoch/website.git
cd opoch
```

2. Set up Node version (using nvm):
```bash
nvm install
nvm use
```

3. Install dependencies:
```bash
npm install
```

4. Copy environment variables:
```bash
cp .env.example .env.local
```

5. Configure environment variables in `.env.local`

6. Set up Supabase:
   - Create a new project
   - Run the SQL schemas in order:
     - `supabase/schema.sql`
     - `supabase/schema_phase3.sql`
     - `supabase/fix_backend.sql` (for email templates and RLS policies)

7. Start development server:
```bash
npm run dev
```

## Development

### Available Scripts

- `npm run dev` - Start development server
- `npm run build` - Build for production
- `npm run preview` - Preview production build
- `npm test` - Run tests
- `npm run lint` - Run ESLint

### Project Structure

```
opoch/
├── api/                 # Vercel serverless functions
├── docs/               # Documentation
├── public/             # Static assets
├── scripts/            # Build and deployment scripts
├── src/
│   ├── components/     # React components
│   ├── contexts/       # React contexts
│   ├── data/          # Static data
│   ├── lib/           # Utilities
│   └── pages/         # Page components
└── supabase/          # Database schemas
```

## Deployment

### Automatic Deployment

Push to `main` branch triggers automatic deployment via GitHub Actions.

### Manual Deployment

```bash
# Preview deployment
./scripts/deploy.sh preview

# Production deployment
./scripts/deploy.sh production
```

### Security Audit

Run security checks before deployment:
```bash
./scripts/security-check.sh
```

## Environment Variables

### Client-side (VITE_ prefix)
- `VITE_SUPABASE_URL` - Supabase project URL
- `VITE_SUPABASE_ANON_KEY` - Supabase anonymous key

### Server-side (Vercel)
- `SUPABASE_SERVICE_ROLE_KEY` - Supabase service role key
- `RESEND_API_KEY` - Resend API key

## Security Best Practices

### Environment Variables
- **NEVER** commit `.env.local` or any file containing real API keys
- `.env.local` is already in `.gitignore` - keep it that way
- Use `.env.example` as a template with placeholder values only
- Only `VITE_` prefixed variables are exposed to the client
- Keep `SERVICE_ROLE_KEY` secure - it has admin database privileges

### API Keys
- Regularly rotate API keys
- Use environment-specific keys (dev/staging/prod)
- Monitor API key usage for anomalies
- Immediately revoke compromised keys

### Before Committing
1. Check that no secrets are in your code: `git diff --staged`
2. Ensure `.env.local` is not staged: `git status`
3. Review `.env.example` contains only placeholders

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Security

- All API endpoints require authentication
- CSP headers prevent XSS attacks
- Security headers configured in `vercel.json`
- Regular dependency updates via Dependabot

## License

Private and confidential. All rights reserved.

## Support

For support, email hello@opoch.com or visit our [documentation](https://docs.opoch.com).