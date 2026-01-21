import { z } from 'zod';

// Browser state types
export interface BrowserState {
  url: string;
  title: string;
  cookies: Array<{
    name: string;
    value: string;
    domain: string;
  }>;
  localStorage: Record<string, string>;
  sessionStorage: Record<string, string>;
}

export interface TabInfo {
  id: string;
  url: string;
  title: string;
  active: boolean;
}

// Tool input schemas
export const NavigateSchema = z.object({
  url: z.string().url().describe('The URL to navigate to'),
  waitUntil: z.enum(['load', 'domcontentloaded', 'networkidle']).optional()
    .describe('When to consider navigation finished')
});

export const ScreenshotSchema = z.object({
  fullPage: z.boolean().optional().describe('Capture full page screenshot'),
  selector: z.string().optional().describe('CSS selector to screenshot specific element')
});

export const ClickSchema = z.object({
  selector: z.string().describe('CSS selector of element to click'),
  timeout: z.number().optional().describe('Maximum time to wait for element (ms)')
});

export const FillSchema = z.object({
  selector: z.string().describe('CSS selector of input element'),
  value: z.string().describe('Value to fill in the input'),
  timeout: z.number().optional().describe('Maximum time to wait for element (ms)')
});

export const WaitForSchema = z.object({
  selector: z.string().optional().describe('CSS selector to wait for'),
  state: z.enum(['visible', 'hidden', 'attached', 'detached']).optional()
    .describe('State to wait for'),
  timeout: z.number().optional().describe('Maximum wait time (ms)')
});

export const ExecuteSchema = z.object({
  script: z.string().describe('JavaScript code to execute in page context'),
  args: z.array(z.any()).optional().describe('Arguments to pass to the script')
});

export const ExtractSchema = z.object({
  selector: z.string().optional().describe('CSS selector to extract from'),
  attribute: z.string().optional().describe('Attribute to extract'),
  all: z.boolean().optional().describe('Extract all matching elements')
});

// Tool response types
export interface ToolResponse<T = any> {
  success: boolean;
  data?: T;
  error?: string;
}