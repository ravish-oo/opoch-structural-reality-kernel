// Environment-aware configuration
// Uses VITE_ prefixed env vars (available in client)

export const config = {
  // Documentation site URL
  // Local: http://localhost:3003 (set in .env.local)
  // Production: https://docs.opoch.com (default fallback)
  docsUrl: import.meta.env.VITE_DOCS_URL || 'https://docs.opoch.com',

  // Chat app URL
  // DEV: localhost:3001 (different port from docs which uses 3000)
  // PROD: https://chat.opoch.com
  // Override with VITE_CHAT_URL in .env.local if needed
  chatUrl: import.meta.env.VITE_CHAT_URL || (import.meta.env.DEV ? 'http://localhost:3001' : 'https://chat.opoch.com'),

  // Main site URL
  siteUrl: import.meta.env.DEV ? 'http://localhost:5173' : 'https://www.opoch.com',
} as const

// Helper to build docs URLs
export const docsLinks = {
  home: () => config.docsUrl,

  // Introduction
  introduction: () => `${config.docsUrl}/truth/introduction`,

  // Derivations (core-logic)
  opochKernel: () => `${config.docsUrl}/proof/derivations/core-logic/opoch-kernel`,
  kernel: () => `${config.docsUrl}/proof/derivations/core-logic/opoch-kernel`, // Legacy alias
  nullStateLogic: () => `${config.docsUrl}/proof/derivations/core-logic/opoch-kernel`, // Legacy alias
  theDerivation: () => `${config.docsUrl}/proof/derivations/core-logic/the-derivation`,

  // Validation (demos)
  theProof: () => `${config.docsUrl}/proof/validation`,
  critPt: () => `${config.docsUrl}/proof/validation/demos/CritPt`,
  applePuzzles: () => `${config.docsUrl}/proof/validation/demos/apple_puzzles_kernel_demo`,
  reasoningSolved: () => `${config.docsUrl}/proof/validation/apps/intelligence_reasoning_selfimprove_demo`,

  // Resources
  reasoningPrompt: () => `${config.docsUrl}/resources/reasoning_prompt`,
  llmFailures: () => `${config.docsUrl}/resources/llm_reasoning_failures`,
}
