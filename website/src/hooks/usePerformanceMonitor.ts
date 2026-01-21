import { useEffect, useRef } from 'react';

export function usePerformanceMonitor(componentName: string) {
  const renderCount = useRef(0);
  const renderStartTime = useRef<number>();

  useEffect(() => {
    renderCount.current += 1;
    const renderEndTime = performance.now();
    
    if (renderStartTime.current) {
      const renderDuration = renderEndTime - renderStartTime.current;
      
      // Only log slow renders in development
      if (process.env.NODE_ENV === 'development' && renderDuration > 16) {
        console.warn(`[Performance] ${componentName} slow render:`, {
          duration: `${renderDuration.toFixed(2)}ms`,
          renderCount: renderCount.current
        });
      }
    }
  });

  // Mark the start of render
  renderStartTime.current = performance.now();

  return {
    renderCount: renderCount.current
  };
}