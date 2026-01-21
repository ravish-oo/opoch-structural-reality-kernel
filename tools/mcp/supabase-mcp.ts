#!/usr/bin/env node
import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { createClient } from "@supabase/supabase-js";
import { z } from "zod";
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from "@modelcontextprotocol/sdk/types.js";

// Load environment from .env.local
import { config } from 'dotenv';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = dirname(fileURLToPath(import.meta.url));
config({ path: join(dirname(__dirname), '.env.local') });

// Environment setup
const SUPABASE_URL = process.env.VITE_SUPABASE_URL;
const SUPABASE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY;

if (!SUPABASE_URL || !SUPABASE_KEY) {
  console.error("Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY");
  process.exit(1);
}

console.error(`Connecting to Supabase at ${SUPABASE_URL}`);

const supabase = createClient(SUPABASE_URL, SUPABASE_KEY, { 
  auth: { persistSession: false } 
});

// Whitelist tables for safety
const ALLOWED_TABLES = new Set([
  "profiles", 
  "leads", 
  "queries", 
  "emails_sent", 
  "email_templates",
  "agent_logs"
]);

function guardTable(table: string) {
  if (!ALLOWED_TABLES.has(table)) {
    throw new Error(`Table '${table}' is not allowed. Add to whitelist if needed.`);
  }
}

// Create server
const server = new Server(
  {
    name: "supabase-mcp",
    version: "1.0.0",
  },
  {
    capabilities: {
      tools: {},
    },
  }
);

// Tool handlers
server.setRequestHandler(ListToolsRequestSchema, async () => {
  return {
    tools: [
      {
        name: "sb_select",
        description: "Select rows from a Supabase table",
        inputSchema: {
          type: "object",
          properties: {
            table: { type: "string", description: "Table name" },
            columns: { type: "string", description: "Columns to select (default: *)" },
            match: { type: "object", description: "Filter conditions" },
            limit: { type: "number", description: "Limit results" },
            order: { 
              type: "object", 
              properties: {
                column: { type: "string" },
                ascending: { type: "boolean" }
              }
            }
          },
          required: ["table"]
        }
      },
      {
        name: "sb_insert",
        description: "Insert rows into a Supabase table",
        inputSchema: {
          type: "object",
          properties: {
            table: { type: "string", description: "Table name" },
            rows: { type: "array", description: "Array of objects to insert" },
            returning: { type: "boolean", description: "Return inserted rows (default: true)" }
          },
          required: ["table", "rows"]
        }
      },
      {
        name: "sb_update",
        description: "Update rows in a Supabase table",
        inputSchema: {
          type: "object",
          properties: {
            table: { type: "string", description: "Table name" },
            match: { type: "object", description: "Filter conditions" },
            patch: { type: "object", description: "Values to update" },
            returning: { type: "boolean", description: "Return updated rows (default: true)" }
          },
          required: ["table", "match", "patch"]
        }
      },
      {
        name: "sb_delete",
        description: "Delete rows from a Supabase table",
        inputSchema: {
          type: "object",
          properties: {
            table: { type: "string", description: "Table name" },
            match: { type: "object", description: "Filter conditions" },
            returning: { type: "boolean", description: "Return deleted rows (default: true)" }
          },
          required: ["table", "match"]
        }
      },
      {
        name: "sb_rpc",
        description: "Call a Postgres function via RPC",
        inputSchema: {
          type: "object",
          properties: {
            fn: { type: "string", description: "Function name" },
            args: { type: "object", description: "Function arguments" }
          },
          required: ["fn"]
        }
      }
    ]
  };
});

// Audit logging
async function logOperation(op: string, payload: any) {
  try {
    await supabase.from("agent_logs").insert({
      actor: "mcp_supabase",
      op,
      payload
    });
  } catch (e) {
    console.error("Failed to log operation:", e);
  }
}

server.setRequestHandler(CallToolRequestSchema, async (request) => {
  try {
    const { name, arguments: args } = request.params;

    switch (name) {
      case "sb_select": {
        const { table, columns = "*", match, limit, order } = args as any;
        guardTable(table);
        
        let query = supabase.from(table).select(columns);
        
        if (match) {
          for (const [key, value] of Object.entries(match)) {
            query = query.eq(key, value);
          }
        }
        
        if (order) {
          query = query.order(order.column, { ascending: order.ascending ?? true });
        }
        
        if (limit) {
          query = query.limit(limit);
        }
        
        const { data, error } = await query;
        
        await logOperation("select", { table, match, limit });
        
        if (error) throw error;
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data, null, 2)
            }
          ]
        };
      }

      case "sb_insert": {
        const { table, rows, returning = true } = args as any;
        guardTable(table);
        
        const query = supabase.from(table).insert(rows);
        const { data, error } = returning ? await query.select() : await query;
        
        await logOperation("insert", { table, rowCount: rows.length });
        
        if (error) throw error;
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data || { success: true }, null, 2)
            }
          ]
        };
      }

      case "sb_update": {
        const { table, match, patch, returning = true } = args as any;
        guardTable(table);
        
        let query = supabase.from(table).update(patch);
        
        for (const [key, value] of Object.entries(match)) {
          query = query.eq(key, value);
        }
        
        const { data, error } = returning ? await query.select() : await query;
        
        await logOperation("update", { table, match });
        
        if (error) throw error;
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data || { success: true }, null, 2)
            }
          ]
        };
      }

      case "sb_delete": {
        const { table, match, returning = true } = args as any;
        guardTable(table);
        
        let query = supabase.from(table).delete();
        
        for (const [key, value] of Object.entries(match)) {
          query = query.eq(key, value);
        }
        
        const { data, error } = returning ? await query.select() : await query;
        
        await logOperation("delete", { table, match });
        
        if (error) throw error;
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data || { success: true }, null, 2)
            }
          ]
        };
      }

      case "sb_rpc": {
        const { fn, args: fnArgs = {} } = args as any;
        
        const { data, error } = await supabase.rpc(fn, fnArgs);
        
        await logOperation("rpc", { function: fn });
        
        if (error) throw error;
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data, null, 2)
            }
          ]
        };
      }

      default:
        throw new Error(`Unknown tool: ${name}`);
    }
  } catch (error: any) {
    return {
      content: [
        {
          type: "text",
          text: `Error: ${error.message || String(error)}`
        }
      ],
      isError: true,
    };
  }
});

// Start server
async function main() {
  const transport = new StdioServerTransport();
  await server.connect(transport);
  console.error("Supabase MCP server is running!");
}

main().catch((error) => {
  console.error("Fatal error:", error);
  process.exit(1);
});