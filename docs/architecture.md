# Opoch Architecture Documentation

## Technology Stack

### Frontend
- **Framework**: React 18.2
- **Build Tool**: Vite 5.4
- **Language**: TypeScript 5.0
- **Styling**: Tailwind CSS 3.3
- **UI Components**: Custom components + Radix UI primitives
- **Animations**: Framer Motion 11.0
- **Forms**: React Hook Form + Zod validation

### Backend Services
- **Database**: Supabase (PostgreSQL)
- **Authentication**: Supabase Auth (planned)
- **Analytics**: Vercel Analytics + Speed Insights
- **Hosting**: Vercel

### Development Tools
- **Testing**: Vitest + Testing Library
- **Linting**: ESLint with TypeScript support
- **CI/CD**: GitHub Actions + Vercel automatic deployments

## Project Structure

```
opoch/
├── docs/                    # Documentation
│   ├── architecture.md
│   ├── development_plan.md
│   └── dependencies.md
├── public/                  # Static assets
│   ├── Opoch Assets/       # Brand assets
│   ├── og-image.jpg        # Social media preview
│   └── favicon.svg
├── src/
│   ├── components/         # React components
│   │   ├── ui/            # Base UI components
│   │   ├── ApplyModal.tsx
│   │   ├── AskBox.tsx
│   │   └── OpochLogo.tsx
│   ├── lib/               # Utilities and clients
│   │   ├── supabase.ts
│   │   └── utils.ts
│   ├── OpochLanding.tsx   # Main landing page
│   ├── main.tsx          # App entry point
│   └── index.css         # Global styles
├── supabase/             # Database schema
│   └── schema.sql
└── .github/
    └── workflows/        # CI/CD pipelines
        └── ci.yml
```

## Key Design Decisions

### 1. Single Page Application (SPA)
- Using Vite instead of Next.js for faster development
- Client-side routing with React Router
- Static deployment to Vercel

### 2. Progressive Enhancement
- Site works without JavaScript (static content visible)
- Database features gracefully degrade
- Forms work even without Supabase connection

### 3. Component Architecture
- Atomic design principles
- Reusable UI components in `components/ui/`
- Business logic separated from presentation

### 4. Styling Strategy
- Tailwind CSS for utility-first styling
- Custom Opoch brand colors in config
- Dark theme by default
- CSS variables for theme customization

### 5. Data Flow
- Form submissions go directly to Supabase
- No server middleware required
- RLS policies for security
- Optional Pipedrive integration (future)

## Security Considerations

### Environment Variables
- `VITE_` prefix for client-accessible variables
- Sensitive keys never exposed to client
- `.env.local` for local development
- Environment variables in Vercel dashboard

### Database Security
- Row Level Security (RLS) enabled
- Anonymous users can only INSERT
- No READ access without authentication
- Service role key kept server-side only

### Content Security
- No user-generated content displayed
- All external links use `rel="noopener noreferrer"`
- HTTPS enforced on production

## Performance Optimizations

### Current
- Optimized images (og-image.jpg at 62KB)
- Code splitting with dynamic imports
- Tailwind CSS tree-shaking
- Minimal JavaScript bundle

### Planned
- Lazy loading for Lottie animations
- Route-based code splitting
- Image optimization with WebP
- Font subsetting for Gotham

## Deployment Strategy

### Development
```bash
npm run dev          # Local development
npm run build       # Production build
npm run preview     # Preview production build
```

### Production
- Automatic deployments via Vercel
- Branch deployments for PRs
- Environment variables in Vercel dashboard
- CDN distribution globally

## Monitoring & Analytics

### Current
- Vercel Analytics for user behavior
- Vercel Speed Insights for performance
- Console logging for debugging

### Planned
- Sentry for error tracking
- Custom event tracking
- Conversion funnel analysis
- A/B testing framework

## Future Considerations

### Scalability
- Consider server-side rendering if SEO becomes critical
- Implement caching strategies
- Database connection pooling
- CDN for static assets

### Internationalization
- Prepare component structure for i18n
- URL structure for multiple languages
- Content management system integration

### Mobile App
- React Native code sharing potential
- API-first architecture
- Progressive Web App capabilities