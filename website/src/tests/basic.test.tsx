import { describe, it, expect, vi } from 'vitest'
import { render } from '@testing-library/react'

// Mock all external dependencies
vi.mock('../lib/supabase', () => ({
  supabase: null,
  isSupabaseConfigured: () => false,
}))

vi.mock('../lib/analytics', () => ({
  trackEvent: vi.fn(),
}))

vi.mock('@vercel/analytics', () => ({
  Analytics: () => null,
}))

vi.mock('@vercel/speed-insights', () => ({
  SpeedInsights: () => null,
}))

// Test that our setup is working
describe('Basic Tests', () => {
  it('should have working test environment', () => {
    expect(true).toBe(true)
  })

  it('should have TextEncoder available', () => {
    expect(typeof TextEncoder).toBe('function')
    const encoder = new TextEncoder()
    const result = encoder.encode('test')
    // Check that it's array-like with the correct values
    expect(result[0]).toBe(116) // 't'
    expect(result[1]).toBe(101) // 'e'
    expect(result[2]).toBe(115) // 's'
    expect(result[3]).toBe(116) // 't'
    expect(result.length).toBe(4)
  })

  it('should have working vitest mocks', () => {
    const mockFn = vi.fn()
    mockFn('test')
    expect(mockFn).toHaveBeenCalledWith('test')
  })

  it('should render basic React component', () => {
    const TestComponent = () => <div>Test</div>
    const { container } = render(<TestComponent />)
    expect(container.textContent).toBe('Test')
  })
})