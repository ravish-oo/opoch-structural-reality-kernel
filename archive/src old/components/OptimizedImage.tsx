import { useState, useEffect } from 'react'
import { cn } from '../lib/utils'

interface OptimizedImageProps {
  src: string
  alt: string
  className?: string
  width?: number
  height?: number
  loading?: 'lazy' | 'eager'
  priority?: boolean
}

export default function OptimizedImage({ 
  src, 
  alt, 
  className, 
  width, 
  height,
  loading = 'lazy',
  priority = false
}: OptimizedImageProps) {
  const [isLoaded, setIsLoaded] = useState(false)
  const [hasError, setHasError] = useState(false)
  
  // Preload priority images
  useEffect(() => {
    if (priority) {
      const link = document.createElement('link')
      link.rel = 'preload'
      link.as = 'image'
      link.href = src
      document.head.appendChild(link)
      
      return () => {
        document.head.removeChild(link)
      }
    }
  }, [src, priority])
  
  const handleLoad = () => {
    setIsLoaded(true)
  }
  
  const handleError = () => {
    setHasError(true)
    setIsLoaded(true)
  }
  
  if (hasError) {
    return (
      <div 
        className={cn(
          'bg-white/5 flex items-center justify-center',
          className
        )}
        style={{ width, height }}
      >
        <span className="text-white/40 text-sm">Failed to load image</span>
      </div>
    )
  }
  
  return (
    <div className={cn('relative overflow-hidden', className)}>
      {!isLoaded && (
        <div 
          className="absolute inset-0 bg-white/5 animate-pulse" 
          style={{ width, height }}
        />
      )}
      <img
        src={src}
        alt={alt}
        width={width}
        height={height}
        loading={priority ? 'eager' : loading}
        onLoad={handleLoad}
        onError={handleError}
        className={cn(
          'transition-opacity duration-300',
          isLoaded ? 'opacity-100' : 'opacity-0',
          className
        )}
      />
    </div>
  )
}