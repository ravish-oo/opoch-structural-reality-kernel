# Moonshot Metadata Enhancements

## ✅ Completed Changes

### 1. Header Button Fix
- **Issue**: "Apply for access" text was too long, causing line wrapping
- **Solution**: Changed back to "Apply" for consistent, compact header
- **Files**: `src/pages/MoonshotsPage.tsx`

### 2. Unique Page Titles
Each moonshot now has a unique, descriptive title format:
```
{Title}: {tagline} | {Domain Context} | Age of Truth
```

**Examples**:
- `Alpha Fixed Point: metrology under control | Physics Breakthrough | Age of Truth`
- `Fusion Control: minimum paid work | Energy Innovation | Age of Truth`
- `Robotics Control: Z₄ rhythm | Robotics Advancement | Age of Truth`

### 3. Enhanced Metadata System
- **Rich Descriptions**: Include status, timeline, and complexity
- **Domain Context**: Physics Breakthrough, Energy Innovation, etc.
- **Status Context**: Open Challenge, Active Research, Partnership Opportunity
- **Extended Keywords**: Include taglines, complexity levels, timelines

### 4. Cover Image Infrastructure
- **Data Structure**: Added `coverImage` and `ogImage` fields to Moonshot interface
- **Directory Structure**: `public/moonshots/{slug}/cover.jpg`
- **Automatic Detection**: System uses custom images or falls back to default
- **SEO Integration**: Custom OG images for each moonshot

## 🎯 Metadata Examples

### Alpha Fixed Point
```typescript
{
  title: "Alpha Fixed Point: metrology under control | Physics Breakthrough | Age of Truth",
  description: "metrology under control - Establishing universal measurement standards through recursive calibration. A foundational approach to precision that makes all other measurements possible. Open Challenge in physics. Timeline: 12-18 months. Complexity: 5/5. Join Opoch in the Age of Truth.",
  image: "/moonshots/alpha-fixed-point/cover.jpg", // or default
  keywords: ["physics", "moonshot", "metrology under control", "open challenge", "complexity 5", "12-18 months", ...]
}
```

### Fusion Control
```typescript
{
  title: "Fusion Control: minimum paid work | Energy Innovation | Age of Truth",
  description: "minimum paid work - Optimal control strategies for fusion plasma confinement. Finding the path to net energy gain with minimal input power. Active Research in energy. Timeline: 12-24 months. Complexity: 5/5. Join Opoch in the Age of Truth.",
  image: "/moonshots/fusion-control/cover.jpg", // or default
  keywords: ["energy", "moonshot", "minimum paid work", "active research", "complexity 5", "12-24 months", ...]
}
```

## 📁 Directory Structure
```
public/moonshots/
├── README.md (comprehensive guide)
├── alpha-fixed-point/
│   ├── cover.jpg (1200x630px)
│   └── og-image.jpg (optional)
├── fusion-control/
│   └── cover.jpg
└── [15 other moonshot directories ready]
```

## 🔧 Technical Implementation

### Enhanced Metadata Generation
- **Context-Aware Titles**: Domain and status context
- **Rich Descriptions**: Timeline, complexity, status integration
- **Custom Images**: Automatic detection and fallback
- **SEO Optimization**: Extended keywords, structured data

### Image Support
- **Flexible Paths**: Support for both cover and OG images
- **Automatic Fallback**: Uses default image if custom not available
- **Future-Ready**: Structure supports easy addition of images

## 📋 Next Steps for Images

1. **Create Images**: Design 1200x630px covers for each moonshot
2. **Add to Data**: Update moonshot objects with image paths
3. **Test**: Verify OG images appear correctly in social sharing
4. **Optimize**: Ensure images are < 500KB for fast loading

## 🌟 Benefits

- **SEO**: Unique, descriptive titles and metadata for each page
- **Social Sharing**: Custom OG images for better engagement
- **User Experience**: Compact header with clear "Apply" button
- **Scalability**: Easy to add new moonshots with custom images
- **Brand Consistency**: "Age of Truth" integrated throughout

All changes maintain the existing functionality while significantly enhancing the metadata richness and preparing for custom moonshot imagery.
