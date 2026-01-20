#!/usr/bin/env node
import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from '@modelcontextprotocol/sdk/types.js';
import { z } from 'zod';
import { BrowserManager } from './browser-manager.js';
import {
  NavigateSchema,
  ScreenshotSchema,
  ClickSchema,
  FillSchema,
  WaitForSchema,
  ExecuteSchema,
  ExtractSchema,
} from './types.js';

// Create server instance
const server = new Server(
  { 
    name: 'chrome-browser', 
    version: '1.0.0'
  },
  {
    capabilities: {
      tools: {}
    }
  }
);

// Create browser manager
const browserManager = new BrowserManager();

// Initialize browser on first use
let browserInitialized = false;
async function ensureBrowserInitialized() {
  if (!browserInitialized) {
    await browserManager.initialize();
    browserInitialized = true;
  }
}

// List available tools
server.setRequestHandler(ListToolsRequestSchema, async () => {
  return {
    tools: [
      {
        name: 'browser_navigate',
        description: 'Navigate to a URL in the browser',
        inputSchema: {
          type: 'object',
          properties: {
            url: { type: 'string', description: 'The URL to navigate to' },
            waitUntil: { 
              type: 'string', 
              enum: ['load', 'domcontentloaded', 'networkidle'],
              description: 'When to consider navigation finished'
            }
          },
          required: ['url']
        }
      },
      {
        name: 'browser_screenshot',
        description: 'Take a screenshot of the current page or element',
        inputSchema: {
          type: 'object',
          properties: {
            fullPage: { type: 'boolean', description: 'Capture full page screenshot' },
            selector: { type: 'string', description: 'CSS selector to screenshot specific element' }
          }
        }
      },
      {
        name: 'browser_click',
        description: 'Click on an element in the page',
        inputSchema: {
          type: 'object',
          properties: {
            selector: { type: 'string', description: 'CSS selector of element to click' },
            timeout: { type: 'number', description: 'Maximum time to wait for element (ms)' }
          },
          required: ['selector']
        }
      },
      {
        name: 'browser_fill',
        description: 'Fill an input field with text',
        inputSchema: {
          type: 'object',
          properties: {
            selector: { type: 'string', description: 'CSS selector of input element' },
            value: { type: 'string', description: 'Value to fill in the input' },
            timeout: { type: 'number', description: 'Maximum time to wait for element (ms)' }
          },
          required: ['selector', 'value']
        }
      },
      {
        name: 'browser_wait_for',
        description: 'Wait for an element or condition',
        inputSchema: {
          type: 'object',
          properties: {
            selector: { type: 'string', description: 'CSS selector to wait for' },
            state: { 
              type: 'string',
              enum: ['visible', 'hidden', 'attached', 'detached'],
              description: 'State to wait for'
            },
            timeout: { type: 'number', description: 'Maximum wait time (ms)' }
          }
        }
      },
      {
        name: 'browser_execute',
        description: 'Execute JavaScript in the page context',
        inputSchema: {
          type: 'object',
          properties: {
            script: { type: 'string', description: 'JavaScript code to execute in page context' },
            args: { type: 'array', description: 'Arguments to pass to the script' }
          },
          required: ['script']
        }
      },
      {
        name: 'browser_get_content',
        description: 'Get the full HTML content of the page',
        inputSchema: {
          type: 'object',
          properties: {}
        }
      },
      {
        name: 'browser_extract_data',
        description: 'Extract data from the page using CSS selectors',
        inputSchema: {
          type: 'object',
          properties: {
            selector: { type: 'string', description: 'CSS selector to extract from' },
            attribute: { type: 'string', description: 'Attribute to extract' },
            all: { type: 'boolean', description: 'Extract all matching elements' }
          }
        }
      },
      {
        name: 'browser_get_state',
        description: 'Get current browser state including URL, title, cookies, and storage',
        inputSchema: {
          type: 'object',
          properties: {}
        }
      },
      {
        name: 'browser_list_tabs',
        description: 'List all open browser tabs',
        inputSchema: {
          type: 'object',
          properties: {}
        }
      },
      {
        name: 'browser_new_tab',
        description: 'Open a new browser tab',
        inputSchema: {
          type: 'object',
          properties: {}
        }
      },
      {
        name: 'browser_switch_tab',
        description: 'Switch to a different browser tab',
        inputSchema: {
          type: 'object',
          properties: {
            tabId: { type: 'string', description: 'The ID of the tab to switch to' }
          },
          required: ['tabId']
        }
      },
      {
        name: 'browser_close_tab',
        description: 'Close a browser tab',
        inputSchema: {
          type: 'object',
          properties: {
            tabId: { type: 'string', description: 'The ID of the tab to close' }
          },
          required: ['tabId']
        }
      }
    ]
  };
});

// Handle tool execution
server.setRequestHandler(CallToolRequestSchema, async (request) => {
  try {
    await ensureBrowserInitialized();
    
    const { name, arguments: args } = request.params;
    
    switch (name) {
      case 'browser_navigate': {
        const parsed = NavigateSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        await browserManager.navigate(parsed.data.url, parsed.data.waitUntil);
        return {
          content: [{ 
            type: 'text', 
            text: `Navigated to ${parsed.data.url}` 
          }]
        };
      }
      
      case 'browser_screenshot': {
        const parsed = ScreenshotSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        const screenshot = await browserManager.screenshot(parsed.data.fullPage, parsed.data.selector);
        const base64 = screenshot.toString('base64');
        return {
          content: [{ 
            type: 'text', 
            text: `Screenshot taken (${screenshot.length} bytes)`,
          }, {
            type: 'image',
            data: base64,
            mimeType: 'image/png'
          }]
        };
      }
      
      case 'browser_click': {
        const parsed = ClickSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        await browserManager.click(parsed.data.selector, parsed.data.timeout);
        return {
          content: [{ 
            type: 'text', 
            text: `Clicked on ${parsed.data.selector}` 
          }]
        };
      }
      
      case 'browser_fill': {
        const parsed = FillSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        await browserManager.fill(parsed.data.selector, parsed.data.value, parsed.data.timeout);
        return {
          content: [{ 
            type: 'text', 
            text: `Filled ${parsed.data.selector} with value` 
          }]
        };
      }
      
      case 'browser_wait_for': {
        const parsed = WaitForSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        await browserManager.waitFor(parsed.data.selector, parsed.data.state, parsed.data.timeout);
        return {
          content: [{ 
            type: 'text', 
            text: 'Wait completed successfully' 
          }]
        };
      }
      
      case 'browser_execute': {
        const parsed = ExecuteSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        const result = await browserManager.execute(parsed.data.script, parsed.data.args);
        return {
          content: [{ 
            type: 'text', 
            text: JSON.stringify(result, null, 2) 
          }]
        };
      }
      
      case 'browser_get_content': {
        const content = await browserManager.getContent();
        return {
          content: [{ 
            type: 'text', 
            text: content 
          }]
        };
      }
      
      case 'browser_extract_data': {
        const parsed = ExtractSchema.safeParse(args);
        if (!parsed.success) {
          throw new Error(`Invalid arguments: ${parsed.error}`);
        }
        const data = await browserManager.extractData(parsed.data.selector, parsed.data.attribute, parsed.data.all);
        return {
          content: [{ 
            type: 'text', 
            text: JSON.stringify(data, null, 2) 
          }]
        };
      }
      
      case 'browser_get_state': {
        const state = await browserManager.getState();
        return {
          content: [{ 
            type: 'text', 
            text: JSON.stringify(state, null, 2) 
          }]
        };
      }
      
      case 'browser_list_tabs': {
        const tabs = await browserManager.listTabs();
        return {
          content: [{ 
            type: 'text', 
            text: JSON.stringify(tabs, null, 2) 
          }]
        };
      }
      
      case 'browser_new_tab': {
        const tabId = await browserManager.newTab();
        return {
          content: [{ 
            type: 'text', 
            text: `New tab created: ${tabId}` 
          }]
        };
      }
      
      case 'browser_switch_tab': {
        const { tabId } = args as { tabId: string };
        if (!tabId) {
          throw new Error('tabId is required');
        }
        await browserManager.switchTab(tabId);
        return {
          content: [{ 
            type: 'text', 
            text: `Switched to tab: ${tabId}` 
          }]
        };
      }
      
      case 'browser_close_tab': {
        const { tabId } = args as { tabId: string };
        if (!tabId) {
          throw new Error('tabId is required');
        }
        await browserManager.closeTab(tabId);
        return {
          content: [{ 
            type: 'text', 
            text: `Closed tab: ${tabId}` 
          }]
        };
      }
      
      default:
        throw new Error(`Unknown tool: ${name}`);
    }
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    return {
      content: [{ 
        type: 'text', 
        text: `Error: ${errorMessage}` 
      }],
      isError: true
    };
  }
});

// Create transport
const transport = new StdioServerTransport();

// Handle cleanup on exit
process.on('SIGINT', async () => {
  console.error('Shutting down browser...');
  await browserManager.cleanup();
  process.exit(0);
});

process.on('SIGTERM', async () => {
  console.error('Shutting down browser...');
  await browserManager.cleanup();
  process.exit(0);
});

// Connect server to transport
server.connect(transport);
console.error('Chrome Browser MCP server started');