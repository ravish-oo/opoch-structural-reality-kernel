import { useState, useCallback, useRef, useEffect } from 'react'
import type { ApiError } from '../types'
import { handleError } from '../lib/error-handler'

export interface AsyncState<T> {
  data: T | null
  loading: boolean
  error: ApiError | null
}

export interface UseAsyncOptions {
  /**
   * Whether to retry on error
   */
  retry?: boolean
  /**
   * Number of retry attempts
   */
  retryCount?: number
  /**
   * Delay between retries in ms
   */
  retryDelay?: number
  /**
   * Callback when the async operation succeeds
   */
  onSuccess?: <T>(data: T) => void
  /**
   * Callback when the async operation fails
   */
  onError?: (error: ApiError) => void
  /**
   * Whether to show error toast (default: true)
   */
  showErrorToast?: boolean
}

/**
 * Hook for managing async operations with loading, error, and retry states
 */
export function useAsync<T = any>(
  asyncFunction: (...args: any[]) => Promise<T>,
  options: UseAsyncOptions = {}
) {
  const [state, setState] = useState<AsyncState<T>>({
    data: null,
    loading: false,
    error: null,
  })

  const mountedRef = useRef(true)
  const retryCountRef = useRef(0)

  useEffect(() => {
    return () => {
      mountedRef.current = false
    }
  }, [])

  const execute = useCallback(
    async (...args: any[]) => {
      setState(prev => ({ ...prev, loading: true, error: null }))

      try {
        const data = await asyncFunction(...args)
        
        if (mountedRef.current) {
          setState({ data, loading: false, error: null })
          options.onSuccess?.(data)
        }
        
        return data
      } catch (err) {
        const error: ApiError = {
          message: err instanceof Error ? err.message : 'An error occurred',
          code: 'ASYNC_ERROR',
          details: err as Record<string, any>
        }

        if (mountedRef.current) {
          setState({ data: null, loading: false, error })
          
          // Show error toast unless disabled
          if (options.showErrorToast !== false) {
            handleError(err)
          }
          
          options.onError?.(error)

          // Handle retry logic
          if (options.retry && retryCountRef.current < (options.retryCount || 3)) {
            retryCountRef.current++
            setTimeout(() => {
              execute(...args)
            }, options.retryDelay || 1000)
          }
        }

        throw error
      }
    },
    [asyncFunction, options]
  )

  const reset = useCallback(() => {
    setState({
      data: null,
      loading: false,
      error: null,
    })
    retryCountRef.current = 0
  }, [])

  return {
    ...state,
    execute,
    reset,
  }
}