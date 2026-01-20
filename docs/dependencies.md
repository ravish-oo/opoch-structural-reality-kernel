# Dependencies Documentation

## Production Dependencies

### Core Framework
- **react**: ^18.2.0
  - Core React library for building UI
- **react-dom**: ^18.2.0
  - React renderer for web applications

### Routing
- **react-router-dom**: ^7.8.2
  - Client-side routing for SPA
  - Note: Requires Node.js 20+ (currently on 18.14)

### Forms & Validation
- **react-hook-form**: ^7.62.0
  - Performant forms with minimal re-renders
- **@hookform/resolvers**: ^5.2.1
  - Validation resolvers for react-hook-form
- **zod**: ^4.1.4
  - TypeScript-first schema validation

### UI Components & Styling
- **@radix-ui/react-dialog**: ^1.1.15
  - Accessible dialog/modal component
- **class-variance-authority**: ^0.7.0
  - Variant API for component styling
- **clsx**: ^2.0.0
  - Utility for constructing className strings
- **tailwind-merge**: ^2.0.0
  - Merge Tailwind CSS classes without conflicts

### Animations
- **framer-motion**: ^11.0.0
  - Production-ready animation library
- **lottie-react**: ^2.4.1
  - Render After Effects animations

### Icons & Graphics
- **lucide-react**: ^0.263.0
  - Beautiful & consistent icon set
- **recharts**: ^2.8.0
  - Charting library built on React components

### Backend & Analytics
- **@supabase/supabase-js**: ^2.56.0
  - Supabase client for database operations
- **@vercel/analytics**: ^1.5.0
  - Privacy-friendly analytics
- **@vercel/speed-insights**: ^1.2.0
  - Web performance monitoring

## Development Dependencies

### TypeScript & Types
- **typescript**: ^5.0.0
  - TypeScript compiler
- **@types/react**: ^18.2.0
  - TypeScript definitions for React
- **@types/react-dom**: ^18.2.0
  - TypeScript definitions for React DOM

### Build Tools
- **vite**: ^5.0.0
  - Next generation frontend tooling
- **@vitejs/plugin-react**: ^4.0.0
  - Official React plugin for Vite

### CSS Processing
- **tailwindcss**: ^3.3.5
  - Utility-first CSS framework
- **autoprefixer**: ^10.4.14
  - Add vendor prefixes to CSS
- **postcss**: ^8.4.31
  - Tool for transforming CSS

### Testing
- **vitest**: ^3.2.4
  - Unit testing framework for Vite
- **@vitest/ui**: ^3.2.4
  - UI for Vitest test runner
- **@testing-library/react**: ^16.3.0
  - React testing utilities
- **@testing-library/jest-dom**: ^6.8.0
  - Custom jest matchers for DOM
- **jsdom**: ^26.1.0
  - JavaScript implementation of web standards

### Code Quality
- **eslint**: ^9.34.0
  - JavaScript/TypeScript linter
- **@typescript-eslint/eslint-plugin**: ^8.41.0
  - ESLint plugin for TypeScript
- **@typescript-eslint/parser**: ^8.41.0
  - TypeScript parser for ESLint
- **eslint-plugin-react**: ^7.37.5
  - React specific linting rules
- **eslint-plugin-react-hooks**: ^5.2.0
  - ESLint rules for React Hooks

## Version Compatibility Notes

### Node.js Requirements
- Current: v18.14.0
- Recommended: v20.0.0+ (for react-router-dom v7)
- Several ESLint packages require Node.js 18.18.0+

### Browser Support
- Modern browsers with ES2020 support
- CSS Grid and Flexbox support required
- No IE11 support

## Security Audit

### Known Vulnerabilities
- 2 moderate severity vulnerabilities (as of last audit)
- Run `npm audit` for details
- Consider `npm audit fix` for patches

## Bundle Size Analysis

### Current Production Bundle
- Main JS: ~540KB (166KB gzipped)
- Main CSS: ~20KB (4.6KB gzipped)

### Optimization Opportunities
- Implement code splitting for routes
- Lazy load heavy components (charts, animations)
- Tree-shake unused icons from lucide-react

## Upgrade Recommendations

1. **Node.js Upgrade**
   - Upgrade to Node.js 20+ for full compatibility
   - Resolves react-router-dom warnings

2. **Bundle Optimization**
   - Consider dynamic imports for large components
   - Implement route-based code splitting

3. **Type Safety**
   - Generate Supabase types from schema
   - Add stricter TypeScript configuration

## License Information

All dependencies use permissive open-source licenses:
- MIT: Most packages
- Apache-2.0: Some build tools
- ISC: Some utility packages

No GPL or copyleft licenses in production dependencies.