# Moonshot Cover Images

This directory contains cover images for individual moonshot pages.

## Image Specifications

### Cover Images
- **Path**: `/moonshots/{slug}/cover.jpg` or `/moonshots/{slug}/cover.png`
- **Dimensions**: 1200x630px (recommended for social sharing)
- **Format**: JPG or PNG
- **File Size**: < 500KB for optimal loading

### OG Images (Optional)
- **Path**: `/moonshots/{slug}/og-image.jpg` or `/moonshots/{slug}/og-image.png`
- **Dimensions**: 1200x630px (Open Graph standard)
- **Format**: JPG or PNG
- **Use**: Specific to social media sharing (if different from cover)

## Directory Structure
```
public/moonshots/
├── alpha-fixed-point/
│   ├── cover.jpg
│   └── og-image.jpg (optional)
├── h-z-cosmic-time/
│   ├── cover.jpg
│   └── og-image.jpg (optional)
├── rotation-lensing/
│   └── cover.jpg
└── ...
```

## Current Moonshot Slugs
- `alpha-fixed-point` - Alpha Fixed Point (Physics)
- `h-z-cosmic-time` - H(z) + Cosmic Time (Physics)
- `rotation-lensing` - Rotation/Lensing (Physics)
- `qec-thresholds` - QEC Thresholds (Physics)
- `fold-bind-explorer` - Fold/Bind Explorer (Bio)
- `fusion-control` - Fusion Control (Energy)
- `solid-state-battery` - Solid-State Battery (Energy)
- `grid-phasor-flow` - Grid Phasor Flow (Infrastructure)
- `robotics-control` - Robotics Control (Robotics)
- `climate-mrv` - Climate MRV (Infrastructure)
- `port-logistics` - Port Logistics (Infrastructure)
- `deep-space-traj` - Deep Space Trajectory (Physics)
- `safety-refuters` - Safety Refuters (Infrastructure)
- `phase-maps-ckl` - Phase Maps (CKL) (Physics)
- `causal-route-finding` - Causal Route-Finding (Infrastructure)

## Implementation Notes

1. **Automatic Detection**: The system automatically looks for cover images at the expected paths
2. **Fallback**: If no custom image is found, the default OG image is used
3. **Data Structure**: Update moonshot data with `coverImage` and `ogImage` fields when adding custom images
4. **SEO Impact**: Each moonshot will have unique OG images for better social sharing

## Adding Images

1. Create a directory for your moonshot: `public/moonshots/{slug}/`
2. Add your cover image: `cover.jpg` or `cover.png`
3. Optionally add a specific OG image: `og-image.jpg` or `og-image.png`
4. Update the moonshot data in `src/data/moonshots.ts` with the image paths:

```typescript
{
  slug: "your-moonshot",
  // ... other properties
  coverImage: "/moonshots/your-moonshot/cover.jpg",
  ogImage: "/moonshots/your-moonshot/og-image.jpg", // optional
}
```

The system will automatically use these images for:
- Social media sharing (Open Graph, Twitter Cards)
- Search engine results
- Internal page headers (future feature)
