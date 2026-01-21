# Bolt Hero vs Opoch Design Language - Gap Analysis

## What Matches ✅

| Element | Bolt Hero | Opoch Standard | Status |
|---------|-----------|----------------|--------|
| Button radius | No explicit rounded-2xl | `rounded-2xl` | ⚠️ Needs fix |
| Primary CTA color | White bg, black text | `bg-white text-black` | ✅ Matches |
| Nav border | `border-white/5` | `border-white/10` | ⚠️ Different |
| Text opacity | Mixed | Specific levels | ⚠️ Inconsistent |

---

## Critical Differences ❌

### 1. **Background Gradient**

**Bolt:**
```tsx
<div className="absolute inset-0 bg-gradient-radial from-purple-900/10 via-black to-black">
```
- Color: Purple (`purple-900`)
- Opacity: 10%
- Style: Purple radial gradient

**Opoch Standard:**
```tsx
<div className="pointer-events-none absolute inset-0 -z-10 bg-[radial-gradient(80%_50%_at_50%_-10%,rgba(27,205,255,0.28),transparent)]">
```
- Color: Opoch cyan (`rgba(27,205,255,0.28)` = `#1BCDFF`)
- Opacity: 28%
- Style: Precise positioning (80% width, 50% height, -10% top offset)
- **Gap:** Wrong color (purple vs cyan), wrong positioning

---

### 2. **Headline Typography**

**Bolt:**
```tsx
<h1 className="text-5xl md:text-6xl lg:text-7xl font-medium">
```
- No `text-balance`
- No `tracking-tight`
- No `leading-tight` or `leading-[1.1]`

**Opoch Standard:**
```tsx
<h1 className="text-5xl font-semibold leading-tight tracking-tight md:text-6xl">
```
OR (for Hero):
```tsx
<h1 className="text-balance text-5xl font-semibold leading-tight tracking-tight md:text-6xl">
```
- **Gap:** Missing text-balance, tracking-tight, leading-tight

---

### 3. **Gradient Text**

**Bolt:**
- ❌ No gradient text for headline
- Headlines are plain white

**Opoch Standard:**
```tsx
<span className="bg-gradient-to-r from-opoch-blue to-opoch-cyan-light bg-clip-text text-transparent">
```
- Used to emphasize key phrases in hero
- Example: "Decide with confidence" in current hero

---

### 4. **Button Styling**

**Bolt:**
```tsx
<button className="bg-white px-10 py-4 text-lg">
```
- Missing `rounded-2xl`
- No hover state defined
- No `font-medium`

**Opoch Standard:**
```tsx
<Button className="rounded-2xl bg-white text-black hover:bg-white/90">
```
- **Gap:** Missing rounded-2xl, hover state

---

### 5. **Secondary CTA (Falsify Us)**

**Bolt:**
```tsx
<a className="border border-white/20 px-4 py-2 text-sm">
```
- Missing `rounded-2xl`
- Background: none (should be `bg-white/5`)

**Opoch Standard:**
```tsx
<Button variant="outline" className="rounded-2xl border-white/20 bg-white/5 text-white hover:bg-white/10">
```
- **Gap:** Missing rounded-2xl, bg-white/5

---

### 6. **Verification Section**

**Bolt:**
```tsx
<div className="space-y-5">
```
- Simple text links with external link icons
- No card container
- No glass morphism

**Opoch Standard:**
```tsx
<div className="rounded-2xl border border-white/10 bg-white/5 p-6">
```
- Should be in a glass card
- Label: `text-xs uppercase tracking-wide text-white/40`
- **Gap:** Missing glass card container

---

### 7. **Animations**

**Bolt:**
- ❌ No Framer Motion animations
- Static layout

**Opoch Standard:**
```tsx
<motion.h1
  initial={{ opacity: 0, y: 10 }}
  animate={{ opacity: 1, y: 0 }}
  transition={{ duration: 0.6 }}
>
```
- Fade-in with slide animations
- Staggered for different elements
- **Gap:** No animations at all

---

### 8. **Status Indicators**

**Bolt:**
```tsx
<span className="animate-pulse text-amber-400/60">
```
- Custom animation classes
- Not using Opoch's state badge system

**Opoch Standard:**
```tsx
<span className="rounded-md px-2 py-0.5 text-xs font-medium bg-emerald-500/20 text-emerald-300">
```
- **Gap:** Not using standard badge system (though this is more flexible, so maybe OK)

---

### 9. **Icon Usage**

**Bolt:**
- Sparkles icons: `text-amber-400/60`
- External link icons: opacity-40

**Opoch Standard:**
- Primary icons: `text-opoch-cyan-light`
- Secondary icons: `text-white/60`
- **Gap:** Not using opoch-cyan-light for primary icons

---

### 10. **Navigation**

**Bolt:**
```tsx
<nav className="border-b border-white/5">
```

**Opoch Standard:**
```tsx
<nav className="border-b border-white/10">
```
- **Gap:** Border opacity too subtle (5% vs 10%)

---

## Priority Fixes (High → Low)

### **HIGH PRIORITY (Brand Identity)**
1. ✅ **Background gradient:** Change purple to opoch-cyan with correct positioning
2. ✅ **Gradient text:** Add color gradient to key headline phrase
3. ✅ **Button radius:** Add `rounded-2xl` to all buttons
4. ✅ **Glass card for verification:** Wrap in `rounded-2xl border-white/10 bg-white/5`

### **MEDIUM PRIORITY (Polish)**
5. ⚠️ **Typography:** Add `text-balance`, `tracking-tight`, `leading-tight`
6. ⚠️ **Hover states:** Add proper hover transitions to buttons
7. ⚠️ **Nav border:** Change from `border-white/5` to `border-white/10`
8. ⚠️ **Icon colors:** Change Sparkles to `text-opoch-cyan-light`

### **LOW PRIORITY (Nice to Have)**
9. 💡 **Framer Motion:** Add fade-in animations (requires dependency)
10. 💡 **State badges:** Convert status indicators to standard badge system

---

## Recommended Action

**Option 1: Quick Fix (10-15 min)**
- Fix HIGH priority items only
- Maintains Bolt's structure, applies brand identity

**Option 2: Full Alignment (30 min)**
- Fix HIGH + MEDIUM priority items
- Makes hero fully consistent with Opoch design language

**Option 3: Go Back to Bolt**
- Give designer the `design-language-discovered.md` file
- Have them redesign with correct patterns
- Re-integrate when done

---

**Which option do you prefer?**
