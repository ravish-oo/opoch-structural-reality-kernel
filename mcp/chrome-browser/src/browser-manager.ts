import { Browser, BrowserContext, Page, chromium } from 'playwright';
import { BrowserState, TabInfo } from './types.js';
import * as path from 'path';
import * as os from 'os';

export class BrowserManager {
  private browser: Browser | null = null;
  private context: BrowserContext | null = null;
  private pages: Map<string, Page> = new Map();
  private activePageId: string | null = null;

  async initialize(): Promise<void> {
    if (this.browser) return;

    const userDataDir = path.join(os.homedir(), '.opoch-chrome-profile');
    
    this.browser = await chromium.launch({
      headless: process.env.CHROME_HEADLESS === 'true',
      args: [
        '--remote-debugging-port=9222',
        '--no-first-run',
        '--disable-default-apps',
        '--disable-popup-blocking'
      ],
    });

    this.context = await this.browser.newContext({
      userAgent: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36',
      viewport: { width: 1280, height: 720 },
      // Store cookies and local storage in profile
      storageState: undefined
    });

    // Create initial page
    const page = await this.context.newPage();
    const pageId = this.generatePageId();
    this.pages.set(pageId, page);
    this.activePageId = pageId;
  }

  async getActivePage(): Promise<Page> {
    if (!this.activePageId || !this.pages.has(this.activePageId)) {
      throw new Error('No active page available');
    }
    return this.pages.get(this.activePageId)!;
  }

  async navigate(url: string, waitUntil?: 'load' | 'domcontentloaded' | 'networkidle'): Promise<void> {
    const page = await this.getActivePage();
    await page.goto(url, { 
      waitUntil: waitUntil || 'load',
      timeout: 30000 
    });
  }

  async screenshot(fullPage?: boolean, selector?: string): Promise<Buffer> {
    const page = await this.getActivePage();
    
    if (selector) {
      const element = await page.$(selector);
      if (!element) throw new Error(`Element not found: ${selector}`);
      return await element.screenshot();
    }
    
    return await page.screenshot({ fullPage: fullPage || false });
  }

  async click(selector: string, timeout?: number): Promise<void> {
    const page = await this.getActivePage();
    await page.click(selector, { timeout: timeout || 30000 });
  }

  async fill(selector: string, value: string, timeout?: number): Promise<void> {
    const page = await this.getActivePage();
    await page.fill(selector, value, { timeout: timeout || 30000 });
  }

  async waitFor(selector?: string, state?: 'visible' | 'hidden' | 'attached' | 'detached', timeout?: number): Promise<void> {
    const page = await this.getActivePage();
    
    if (selector) {
      await page.waitForSelector(selector, {
        state: state || 'visible',
        timeout: timeout || 30000
      });
    } else {
      // Wait for load state if no selector
      await page.waitForLoadState('networkidle', { timeout: timeout || 30000 });
    }
  }

  async execute(script: string, args?: any[]): Promise<any> {
    const page = await this.getActivePage();
    return await page.evaluate(script, args);
  }

  async getContent(): Promise<string> {
    const page = await this.getActivePage();
    return await page.content();
  }

  async extractData(selector?: string, attribute?: string, all?: boolean): Promise<any> {
    const page = await this.getActivePage();
    
    if (!selector) {
      // Extract all text content
      return await page.evaluate(() => document.body.innerText);
    }
    
    if (all) {
      return await page.evaluate(({ selector, attribute }) => {
        const elements = document.querySelectorAll(selector);
        return Array.from(elements).map(el => 
          attribute ? el.getAttribute(attribute) : el.textContent
        );
      }, { selector, attribute });
    } else {
      return await page.evaluate(({ selector, attribute }) => {
        const element = document.querySelector(selector);
        if (!element) return null;
        return attribute ? element.getAttribute(attribute) : element.textContent;
      }, { selector, attribute });
    }
  }

  async getState(): Promise<BrowserState> {
    const page = await this.getActivePage();
    
    const [url, title, cookies] = await Promise.all([
      page.url(),
      page.title(),
      this.context!.cookies()
    ]);

    const storage = await page.evaluate(() => ({
      localStorage: { ...localStorage },
      sessionStorage: { ...sessionStorage }
    }));

    return {
      url,
      title,
      cookies: cookies.map(c => ({
        name: c.name,
        value: c.value,
        domain: c.domain
      })),
      localStorage: storage.localStorage,
      sessionStorage: storage.sessionStorage
    };
  }

  async listTabs(): Promise<TabInfo[]> {
    const tabs: TabInfo[] = [];
    
    for (const [id, page] of this.pages) {
      tabs.push({
        id,
        url: page.url(),
        title: await page.title(),
        active: id === this.activePageId
      });
    }
    
    return tabs;
  }

  async newTab(): Promise<string> {
    if (!this.context) throw new Error('Browser not initialized');
    
    const page = await this.context.newPage();
    const pageId = this.generatePageId();
    this.pages.set(pageId, page);
    this.activePageId = pageId;
    
    return pageId;
  }

  async switchTab(tabId: string): Promise<void> {
    if (!this.pages.has(tabId)) {
      throw new Error(`Tab not found: ${tabId}`);
    }
    this.activePageId = tabId;
  }

  async closeTab(tabId: string): Promise<void> {
    const page = this.pages.get(tabId);
    if (!page) throw new Error(`Tab not found: ${tabId}`);
    
    await page.close();
    this.pages.delete(tabId);
    
    // Switch to another tab if we closed the active one
    if (this.activePageId === tabId) {
      const remainingTabs = Array.from(this.pages.keys());
      this.activePageId = remainingTabs.length > 0 ? remainingTabs[0] : null;
    }
  }

  async cleanup(): Promise<void> {
    if (this.browser) {
      await this.browser.close();
      this.browser = null;
      this.context = null;
      this.pages.clear();
      this.activePageId = null;
    }
  }

  private generatePageId(): string {
    return `page-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}