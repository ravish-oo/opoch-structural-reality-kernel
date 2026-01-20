// Performance monitoring utilities

declare global {
  interface Window {
    gtag?: (...args: any[]) => void
  }
}

export function measurePageLoad() {
  if (typeof window !== 'undefined' && window.performance) {
    window.addEventListener('load', () => {
      const perfData = window.performance.getEntriesByType('navigation')[0] as PerformanceNavigationTiming
      
      if (perfData) {
        const pageLoadTime = perfData.loadEventEnd - perfData.fetchStart
        const dnsTime = perfData.domainLookupEnd - perfData.domainLookupStart
        const tcpTime = perfData.connectEnd - perfData.connectStart
        const ttfb = perfData.responseStart - perfData.fetchStart
        const domContentLoaded = perfData.domContentLoadedEventEnd - perfData.domContentLoadedEventStart
        
        // Log performance metrics
        console.log('Performance Metrics:', {
          pageLoadTime: `${pageLoadTime}ms`,
          dnsTime: `${dnsTime}ms`,
          tcpTime: `${tcpTime}ms`,
          ttfb: `${ttfb}ms`,
          domContentLoaded: `${domContentLoaded}ms`
        })
        
        // Send to analytics if needed
        if (window.gtag) {
          window.gtag('event', 'page_timing', {
            page_load_time: Math.round(pageLoadTime),
            dns_time: Math.round(dnsTime),
            tcp_time: Math.round(tcpTime),
            ttfb: Math.round(ttfb)
          })
        }
      }
    })
  }
}

// Web Vitals monitoring
export function reportWebVitals(onPerfEntry?: (metric: any) => void) {
  if (onPerfEntry && onPerfEntry instanceof Function) {
    import('web-vitals').then(({ onCLS, onINP, onFCP, onLCP, onTTFB }) => {
      onCLS(onPerfEntry)
      onINP(onPerfEntry) // FID is deprecated, use INP instead
      onFCP(onPerfEntry)
      onLCP(onPerfEntry)
      onTTFB(onPerfEntry)
    })
  }
}

// Intersection Observer for lazy loading
export function createLazyLoadObserver(callback: (entry: IntersectionObserverEntry) => void) {
  if ('IntersectionObserver' in window) {
    return new IntersectionObserver(
      (entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            callback(entry)
          }
        })
      },
      {
        rootMargin: '50px 0px',
        threshold: 0.01
      }
    )
  }
  return null
}

// Resource hints
export function addResourceHints() {
  const hints = [
    { rel: 'dns-prefetch', href: 'https://fonts.googleapis.com' },
    { rel: 'dns-prefetch', href: 'https://www.googletagmanager.com' },
    { rel: 'preconnect', href: 'https://fonts.googleapis.com' },
    { rel: 'preconnect', href: 'https://fonts.gstatic.com', crossorigin: true }
  ]
  
  hints.forEach(hint => {
    const link = document.createElement('link')
    link.rel = hint.rel
    link.href = hint.href
    if (hint.crossorigin) {
      link.crossOrigin = 'anonymous'
    }
    document.head.appendChild(link)
  })
}

// Initialize performance monitoring
export function initPerformance() {
  measurePageLoad()
  addResourceHints()
  
  // Report web vitals
  reportWebVitals((metric) => {
    console.log('Web Vital:', metric.name, Math.round(metric.value))
    
    // Send to analytics
    if (window.gtag) {
      window.gtag('event', 'web_vitals', {
        metric_name: metric.name,
        metric_value: Math.round(metric.value),
        metric_id: metric.id
      })
    }
  })
}