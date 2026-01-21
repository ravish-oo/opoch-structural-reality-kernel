import '@testing-library/jest-dom'
import { vi } from 'vitest'
import { TextEncoder as NodeTextEncoder, TextDecoder as NodeTextDecoder } from 'util'

// Polyfill TextEncoder/TextDecoder with proper Uint8Array
if (!global.TextEncoder) {
  global.TextEncoder = class TextEncoder extends NodeTextEncoder {
    encode(input: string): Uint8Array {
      return new Uint8Array(super.encode(input));
    }
  } as any;
}

if (!global.TextDecoder) {
  global.TextDecoder = NodeTextDecoder as any;
}

// Add ResizeObserver mock
global.ResizeObserver = class ResizeObserver {
  observe() {}
  unobserve() {}
  disconnect() {}
};

// Mock window.matchMedia
Object.defineProperty(window, 'matchMedia', {
  writable: true,
  value: vi.fn().mockImplementation(query => ({
    matches: false,
    media: query,
    onchange: null,
    addListener: vi.fn(), // deprecated
    removeListener: vi.fn(), // deprecated
    addEventListener: vi.fn(),
    removeEventListener: vi.fn(),
    dispatchEvent: vi.fn(),
  })),
});

// Mock IntersectionObserver
global.IntersectionObserver = class IntersectionObserver {
  root: Element | null = null;
  rootMargin: string = '';
  thresholds: ReadonlyArray<number> = [];
  
  constructor() {}
  observe() {}
  unobserve() {}
  disconnect() {}
  takeRecords(): IntersectionObserverEntry[] {
    return [];
  }
} as any;