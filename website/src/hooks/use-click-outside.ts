import { useEffect, useRef, RefObject } from 'react'

/**
 * Hook that detects clicks outside of the passed ref
 */
export function useClickOutside<T extends HTMLElement = HTMLElement>(
  handler: () => void,
  mouseEvent: 'mousedown' | 'mouseup' = 'mousedown'
): RefObject<T> {
  const ref = useRef<T>(null)

  useEffect(() => {
    const listener = (event: MouseEvent | TouchEvent) => {
      const el = ref?.current
      if (!el || el.contains((event?.target as Node) || null)) {
        return
      }

      handler()
    }

    document.addEventListener(mouseEvent, listener)
    document.addEventListener('touchstart', listener)

    return () => {
      document.removeEventListener(mouseEvent, listener)
      document.removeEventListener('touchstart', listener)
    }
  }, [handler, mouseEvent])

  return ref
}