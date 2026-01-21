#!/usr/bin/env node
import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

console.log('Testing Chrome Browser MCP Server...\n');

// Start the server
const serverPath = join(__dirname, 'run.js');
const server = spawn('node', [serverPath], {
  stdio: ['pipe', 'pipe', 'pipe']
});

// Send a test request
setTimeout(() => {
  const testRequest = JSON.stringify({
    jsonrpc: '2.0',
    method: 'tools/list',
    id: 1
  }) + '\n';
  
  server.stdin.write(testRequest);
}, 1000);

// Handle responses
server.stdout.on('data', (data) => {
  console.log('Response:', data.toString());
});

server.stderr.on('data', (data) => {
  console.log('Server log:', data.toString());
});

// Clean exit after 5 seconds
setTimeout(() => {
  console.log('\nTest complete. Shutting down...');
  server.kill();
  process.exit(0);
}, 5000);