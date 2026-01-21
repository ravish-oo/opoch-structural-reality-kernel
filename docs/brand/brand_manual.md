# Opoch Brand Manual

## Design Philosophy
Opoch embodies precision, clarity, and technical excellence. Our visual language reflects our mission: to provide precise technical guidance that actually works.

## Typography

### Primary Font
- **Font Family**: 'Gotham HTF', 'Whitney', Verdana, system-ui, Avenir, Helvetica, Arial, sans-serif
- **Headings**: Medium weight (font-medium), tight tracking
- **Body Text**: Regular weight, comfortable line height

### Font Sizes
- **Hero Title**: text-4xl md:text-5xl lg:text-6xl
- **Section Titles**: text-2xl md:text-3xl
- **Body Text**: text-base md:text-lg
- **Small Text**: text-sm

## Color Palette

### Primary Colors
- **Pure White**: #FFFFFF (rgb(255, 255, 255))
- **Pure Black**: #000000 (rgb(0, 0, 0))

### Opacity Values
- **Primary Text**: text-white (100%)
- **Secondary Text**: text-white/60 (60% opacity)
- **Subtle Text**: text-white/40 (40% opacity)
- **Borders**: border-white/5 (5% opacity)
- **Hover States**: border-white/10 (10% opacity)

### Background Effects
- **Subtle Gradient**: bg-[radial-gradient(50%_50%_at_50%_0%,rgba(120,119,198,0.15),transparent)]
- **Section Backgrounds**: Transparent or subtle white/[0.02]
- **Card Backgrounds**: bg-transparent with borders

## Spacing System

### Section Padding
- **Vertical**: py-20 (80px)
- **Horizontal**: px-4 sm:px-6 lg:px-8

### Component Spacing
- **Card Padding**: p-6 (24px)
- **Button Padding**: px-6 py-3
- **Grid Gaps**: gap-6 to gap-8

## Components

### Navigation
- **Style**: Fixed positioning with backdrop blur
- **Height**: py-4 (16px vertical padding)
- **Background**: bg-black/50 backdrop-blur-md

### Buttons
- **Primary**: bg-white text-black hover:bg-white/90
- **Shape**: Default radius (not rounded-full for primary actions)
- **Size**: px-6 py-3 text-sm font-medium

### Cards
- **Border**: border-white/5 (subtle)
- **Background**: bg-transparent
- **Hover**: hover:border-white/10
- **Radius**: rounded-xl

### Icons
- **Size**: h-5 w-5 (standard), h-4 w-4 (small)
- **Color**: text-white/40 (subtle presence)

## Animation Guidelines

### Motion Principles
- **Subtle**: Minimal motion, purposeful animations
- **Duration**: 200-500ms for transitions
- **Easing**: Default Tailwind transitions

### Hover Effects
- **Border opacity increase**: from /5 to /10
- **No dramatic shadows or transforms
- **Maintain minimalist aesthetic

## Layout Principles

### Content Width
- **Max Width**: max-w-7xl (1280px)
- **Centered**: mx-auto

### Grid Systems
- **Three Column**: md:grid-cols-3
- **Two Column**: md:grid-cols-2
- **Responsive**: Stack on mobile

## Voice & Tone

### Headlines
- **Direct and confident**: "Get unstuck. Build faster. Decide with confidence."
- **No unnecessary embellishment**

### Body Copy
- **Clear and concise**
- **Technical but accessible**
- **Focus on outcomes and value**

## Implementation Notes

### CSS Classes Priority
1. Use Tailwind utilities
2. Maintain consistency across components
3. Avoid custom CSS unless necessary

### Accessibility
- Maintain proper contrast ratios
- Include ARIA labels for interactive elements
- Ensure keyboard navigation works

### Performance
- Use lazy loading for below-fold content
- Optimize animations for 60fps
- Minimize unnecessary re-renders