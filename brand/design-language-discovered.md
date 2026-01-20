# Opoch Design Language (Discovered from Code)

**Source:** Extracted from actual implemented components in `src/components/landing/`

---

## 1. GRADIENT SYSTEM

### **Radial Background Gradient (Hero Pattern)**
```tsx
className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]"
```
- **Color:** opoch-cyan-light rgba(27,205,255,0.28) = #1BCDFF at 28% opacity
- **Shape:** 80% width, 50% height, positioned at 50% horizontal, -10% vertical (above viewport)
- **Usage:** Hero sections, primary focus areas
- **Purpose:** Subtle ambient glow, draws eye upward

### **Gradient Text (Two Variants)**

**Variant 1: Color Gradient (Primary Headlines)**
```tsx
className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent"
```
- Smooth blue → cyan transition
- Usage: Primary hero headline, key CTAs in text

**Variant 2: Fade Gradient (Section Titles)**
```tsx
className="bg-gradient-to-r from-white to-white/60 bg-clip-text text-transparent"
```
- White fading to 60% opacity
- Usage: Section headings ("15 Moonshots", "Common questions")

---

## 2. GLASS MORPHISM SYSTEM

### **Standard Card (Default State)**
```tsx
className="rounded-2xl border border-white/10 bg-white/5"
```
- **Border:** white at 10% opacity (subtle)
- **Background:** white at 5% opacity (glass effect)
- **Corners:** rounded-2xl (16px radius)

### **Card Hover State**
```tsx
className="hover:border-white/20 hover:bg-white/10 transition-all"
```
- **Border:** white at 20% (doubled opacity)
- **Background:** white at 10% (doubled opacity)
- **Transition:** `transition-all` for smooth animation

### **Alternate Card (Solid Background)**
```tsx
className="rounded-2xl border border-white/10 bg-black"
```
- Used when parent has `bg-white/5` to create contrast

---

## 3. FRAMER MOTION ANIMATION PATTERNS

### **Standard Fade-In with Slide**
```tsx
initial={{ opacity: 0, y: 20 }}
whileInView={{ opacity: 1, y: 0 }}
transition={{ duration: 0.5 }}
viewport={{ once: true }}
```
- **Movement:** 20px upward slide
- **Duration:** 0.5s (can be 0.4-0.6s)
- **Viewport:** `once: true` (animate only on first view, no re-trigger)

### **Staggered Children Animation**
```tsx
transition={{ duration: 0.4, delay: index * 0.02 }}
```
- **Delay multiplier:** 0.02 for quick succession (grids)
- **Delay multiplier:** 0.1 for slower succession (FAQs, lists)
- Creates wave effect across items

### **Simple Fade (Headers)**
```tsx
initial={{ opacity: 0, y: 10 }}
animate={{ opacity: 1, y: 0 }}
transition={{ duration: 0.6 }}
```
- Smaller 10px movement for headlines
- No `whileInView`, animates immediately on mount

---

## 4. TYPOGRAPHY SYSTEM

### **Headline Sizes**
- **Hero:** `text-5xl font-semibold md:text-6xl`
- **Section Titles:** `text-3xl font-bold md:text-5xl` OR `text-3xl font-semibold`
- **Card Titles:** `text-lg font-semibold tracking-tight`

### **Special Typography Utility**
```tsx
className="text-balance"
```
- **Usage:** Multi-line headlines for better word wrapping
- Prevents orphan words

### **Body Text Opacity Levels**
- **Primary body:** `text-white/80`
- **Secondary body:** `text-white/70`
- **Tertiary/muted:** `text-white/60`
- **Ultra-muted:** `text-white/40`

### **Line Height**
- Headlines: `leading-tight` (1.25)
- Body: `leading-relaxed` (1.625)

---

## 5. BUTTON SYSTEM

### **Primary Button (White CTA)**
```tsx
className="rounded-2xl bg-white text-black hover:bg-white/90"
```
- Always `rounded-2xl` (never rounded-full)
- White background, black text
- Hover: 90% opacity

### **Outline Button (Secondary)**
```tsx
className="rounded-2xl border-white/20 bg-white/5 text-white hover:bg-white/10"
```
- Subtle border and background
- Hover: Double the background opacity

### **Ghost Button (Tertiary)**
```tsx
variant="ghost" className="text-white/80 hover:text-white p-0"
```
- No background
- Just text color change on hover

---

## 6. ICON SYSTEM

### **Icon Colors**
- **Primary icons:** `text-opoch-cyan-light` (accent color)
- **Secondary icons:** `text-white/60`
- **Muted icons:** `text-white/40`

### **Icon Sizes**
- **Standard:** `h-5 w-5`
- **Small:** `h-4 w-4`
- **Large:** `h-6 w-6` OR `h-7 w-7` (badges)

---

## 7. STATE BADGE SYSTEM

### **Pattern**
```tsx
className="rounded-md px-2 py-0.5 text-xs font-medium"
```

### **Color Variants**
- **PASS (Success):** `bg-emerald-500/20 text-emerald-300`
- **Ø (Warning):** `bg-amber-500/20 text-amber-300`
- **Ø_∞ (Info):** `bg-sky-500/20 text-sky-300`

**Formula:** Background at 20% opacity, text at 300 shade

---

## 8. SECTION LAYOUT SYSTEM

### **Section Dividers**
```tsx
className="border-t border-white/10"
```
- Always 10% opacity border at top

### **Section Background Alternation**
- **Transparent:** No background (black shows through)
- **Subtle fill:** `bg-white/5`
- Pattern: Transparent → White/5 → Transparent → White/5

### **Section Padding**
- **Standard:** `py-16` (64px)
- **Spacious:** `py-24` (96px)
- **Hero:** `py-20` (80px)

### **Container**
```tsx
className="mx-auto max-w-7xl px-4"
```
- Max width: 1280px
- Horizontal padding: 16px (scales with breakpoints)

---

## 9. GRID SYSTEM

### **Spacing**
- **Gap:** `gap-4` (16px), `gap-6` (24px), `gap-8` (32px)
- Standard is `gap-6`

### **Columns**
- **2-col:** `md:grid-cols-2`
- **3-col:** `md:grid-cols-3`
- **4-col:** `sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4`

---

## 10. INTERACTION PATTERNS

### **Hover Transitions**
```tsx
className="transition-all" // OR
className="transition-colors" // More specific
```
- Duration: 200ms (default) OR `duration-200` explicit
- Easing: Default ease

### **Card Hover Pattern**
```tsx
className="group hover:border-white/20 hover:bg-white/10"
```
- Border opacity: 10% → 20%
- Background opacity: 5% → 10%
- Use `group` for child element hover effects

---

## 11. SPECIAL ELEMENTS

### **Numbered Badges (Process Steps)**
```tsx
<div className="flex h-7 w-7 items-center justify-center rounded-full bg-white/10 text-sm">
  {number}
</div>
```

### **Icon + Text Pattern**
```tsx
<div className="flex items-center gap-3">
  <Icon className="h-5 w-5 text-opoch-cyan-light" />
  <p className="font-medium">{title}</p>
</div>
```
- Consistent 12px gap (gap-3)

---

## 12. ACCESSIBILITY PATTERNS

### **Motion Respect**
- All animations use `viewport={{ once: true }}` to prevent re-triggering
- Transitions are smooth (200-600ms) and not jarring

### **Semantic HTML**
- Proper button elements for interactions
- ARIA labels for expandable sections
- Keyboard navigation support

---

## SUMMARY: KEY DIFFERENCES FROM GENERIC DESIGNS

1. ✅ **Radial gradients** with specific opoch-cyan color and positioning
2. ✅ **Gradient text** (two variants: color fade and white fade)
3. ✅ **Framer Motion** with consistent patterns and `viewport: once`
4. ✅ **rounded-2xl** everywhere (never rounded-md or rounded-full for cards/buttons)
5. ✅ **border-white/10 + bg-white/5** combo for glass effect
6. ✅ **Hover states double the opacity** (10% → 20%, 5% → 10%)
7. ✅ **Icon color:** text-opoch-cyan-light for primary actions
8. ✅ **text-balance** for headlines
9. ✅ **State badges** with 20% opacity backgrounds
10. ✅ **Section alternation** between transparent and bg-white/5

---

**Last Updated:** 2025-10-23
**Source:** Actual code from src/components/landing/
