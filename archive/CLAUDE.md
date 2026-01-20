# Claude Code Instructions for Opoch Landing Page

## Project Overview

This is a React-based landing page for Opoch, a consulting service for deep-tech founders, PI-level researchers, and staff+ engineers. The project consists of a single TypeScript React component and a markdown copy deck.

## Tech Stack

- **Frontend Framework**: React with TypeScript
- **Animation**: Framer Motion (`framer-motion`)
- **Icons**: Lucide React (`lucide-react`)
- **UI Components**: Custom UI components from `@/components/ui/` (Button, Card)
- **Styling**: Tailwind CSS (evident from utility classes)

## File Structure

```
/opoch/
├── OpochLanding.tsx    # Main React component with full landing page
└── copy_deck.md        # Marketing copy and content reference
```

## Key Components

The `OpochLanding.tsx` file contains:
- **Nav**: Sticky navigation with branding and menu
- **Hero**: Main hero section with call-to-action
- **Outcomes**: Three key value propositions
- **Process**: 3-step process explanation
- **Who**: Target audience sections
- **Moonshots**: Grid of 15 moonshot projects
- **Pricing**: Membership details ($11,111/mo)
- **SF**: San Francisco location information
- **RBTSection**: Technical details with verifier
- **FAQ**: Common questions
- **Footer**: Standard footer with links

## Development Guidelines

### Component Patterns
- Uses functional components with hooks
- Animations implemented with Framer Motion
- Utility function `classNames()` for conditional CSS classes
- Custom components like `Tag`, `ReceiptBadge`, and `VerifierInline`

### Styling Conventions
- Dark theme with `bg-[#0B0F1A]` as primary background
- Tailwind CSS for all styling
- Glassmorphism effects with `backdrop-blur` and transparent backgrounds
- Gradient text effects using `bg-clip-text`
- Consistent spacing with max-w-7xl containers

### Key Features
1. **Verifier Widget**: Client-side JSON verification using Web Crypto API
2. **Motion Animations**: Staggered animations on scroll
3. **Responsive Design**: Mobile-first with md/lg breakpoints
4. **Interactive Elements**: Expandable RBT section, file upload for verification

## Common Tasks

### Adding a New Section
1. Create a new function component following the existing pattern
2. Add it to the main `OpochLanding` component
3. Use consistent styling: `border-t border-white/10` for sections
4. Maintain the max-w-7xl container pattern

### Modifying Content
- Marketing copy is defined inline in the component
- Key data arrays: `outcomes`, `who`, `moonshots`, `qa`
- Update the `copy_deck.md` file to keep documentation in sync

### Working with the Verifier
- Uses canonical JSON stringification (sorted keys, no spaces)
- SHA-256 hashing via Web Crypto API
- Local-only processing (no server uploads)

## Important Notes

1. **No Package.json**: This appears to be a component file extracted from a larger project
2. **Import Paths**: Uses `@/components/ui/` suggesting a Next.js or similar setup
3. **Missing Dependencies**: The actual UI component library is not included
4. **Build System**: No build configuration present - likely part of a larger project

## Testing Approach

Since no test files or configuration exist:
1. Visual testing through component rendering
2. Verify all interactive elements (buttons, file upload)
3. Check responsive behavior at different breakpoints
4. Test the JSON verifier with valid/invalid files

## Deployment Considerations

This component requires:
- React 18+ (for modern hooks)
- TypeScript configuration
- Tailwind CSS setup
- Framer Motion installation
- Lucide React for icons
- Custom UI component library

The component is production-ready but needs to be integrated into a complete React application with proper build tooling.