#!/usr/bin/env node
// Supabase MCP Server for Opoch Project
import { createClient, SupabaseClient } from "@supabase/supabase-js";
import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { 
  CallToolRequestSchema, 
  ListToolsRequestSchema,
  Tool
} from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod";

// Environment configuration
const SUPABASE_URL = process.env.SUPABASE_URL;
const SUPABASE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY || process.env.SUPABASE_ANON_KEY;

if (!SUPABASE_URL || !SUPABASE_KEY) {
  console.error("Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY/SUPABASE_ANON_KEY");
  process.exit(1);
}

// Initialize Supabase client
const supabase: SupabaseClient = createClient(SUPABASE_URL, SUPABASE_KEY, {
  auth: {
    persistSession: false,
    autoRefreshToken: false,
  }
});

// Whitelisted tables for safety
const ALLOWED_TABLES = new Set([
  "profiles",
  "leads", 
  "queries",
  "emails_sent",
  "email_templates",
  // Add more tables as needed
]);

// Whitelisted RPC functions
const ALLOWED_RPC_FUNCTIONS = new Set([
  "execute_sql", // If you have this function
  "handle_new_user",
  // Add more functions as needed
]);

function guardTable(table: string): void {
  if (!ALLOWED_TABLES.has(table)) {
    throw new Error(`Table '${table}' is not allowed. Allowed tables: ${Array.from(ALLOWED_TABLES).join(", ")}`);
  }
}

function guardRpcFunction(fn: string): void {
  if (!ALLOWED_RPC_FUNCTIONS.has(fn)) {
    throw new Error(`RPC function '${fn}' is not allowed. Allowed functions: ${Array.from(ALLOWED_RPC_FUNCTIONS).join(", ")}`);
  }
}

// Tool schemas
const SelectSchema = z.object({
  table: z.string().describe("The table name to select from"),
  columns: z.string().optional().default("*").describe("Columns to select (comma-separated)"),
  match: z.record(z.any()).optional().describe("Filter conditions as key-value pairs"),
  limit: z.number().int().positive().optional().describe("Maximum number of rows to return"),
  order: z.object({
    column: z.string().describe("Column to order by"),
    ascending: z.boolean().optional().default(true).describe("Sort ascending (true) or descending (false)"),
    nullsFirst: z.boolean().optional().describe("Put nulls first in ordering")
  }).optional().describe("Ordering configuration")
});

const InsertSchema = z.object({
  table: z.string().describe("The table name to insert into"),
  rows: z.array(z.record(z.any())).min(1).describe("Array of objects to insert")
});

const UpdateSchema = z.object({
  table: z.string().describe("The table name to update"),
  match: z.record(z.any()).describe("Filter conditions to identify rows to update"),
  patch: z.record(z.any()).describe("Fields to update with new values")
});

const DeleteSchema = z.object({
  table: z.string().describe("The table name to delete from"),
  match: z.record(z.any()).describe("Filter conditions to identify rows to delete")
});

const RpcSchema = z.object({
  fn: z.string().describe("The RPC function name to call"),
  args: z.record(z.any()).optional().default({}).describe("Arguments to pass to the function")
});

const ExecuteSqlSchema = z.object({
  query: z.string().describe("SQL query to execute (SELECT only for safety)"),
  params: z.array(z.any()).optional().describe("Query parameters for prepared statements")
});

// Initialize MCP server
const server = new Server(
  {
    name: "supabase-mcp",
    version: "1.0.0",
  },
  {
    capabilities: {
      tools: {}
    }
  }
);

// Tool definitions
const tools: Tool[] = [
  {
    name: "sb_select",
    description: "Select rows from a Supabase table with optional filtering, ordering, and limiting",
    inputSchema: {
      type: "object",
      properties: {
        table: { type: "string", description: "The table name to select from" },
        columns: { type: "string", description: "Columns to select (comma-separated)", default: "*" },
        match: { type: "object", description: "Filter conditions as key-value pairs", additionalProperties: true },
        limit: { type: "number", description: "Maximum number of rows to return" },
        order: {
          type: "object",
          properties: {
            column: { type: "string", description: "Column to order by" },
            ascending: { type: "boolean", description: "Sort ascending (true) or descending (false)", default: true },
            nullsFirst: { type: "boolean", description: "Put nulls first in ordering" }
          },
          required: ["column"]
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
        table: { type: "string", description: "The table name to insert into" },
        rows: {
          type: "array",
          items: { type: "object", additionalProperties: true },
          description: "Array of objects to insert",
          minItems: 1
        }
      },
      required: ["table", "rows"]
    }
  },
  {
    name: "sb_update",
    description: "Update rows in a Supabase table based on matching conditions",
    inputSchema: {
      type: "object",
      properties: {
        table: { type: "string", description: "The table name to update" },
        match: { type: "object", description: "Filter conditions to identify rows to update", additionalProperties: true },
        patch: { type: "object", description: "Fields to update with new values", additionalProperties: true }
      },
      required: ["table", "match", "patch"]
    }
  },
  {
    name: "sb_delete",
    description: "Delete rows from a Supabase table based on matching conditions",
    inputSchema: {
      type: "object",
      properties: {
        table: { type: "string", description: "The table name to delete from" },
        match: { type: "object", description: "Filter conditions to identify rows to delete", additionalProperties: true }
      },
      required: ["table", "match"]
    }
  },
  {
    name: "sb_rpc",
    description: "Call a Postgres function (RPC) with arguments",
    inputSchema: {
      type: "object",
      properties: {
        fn: { type: "string", description: "The RPC function name to call" },
        args: { type: "object", description: "Arguments to pass to the function", additionalProperties: true }
      },
      required: ["fn"]
    }
  },
  {
    name: "sb_sql",
    description: "Execute a SELECT SQL query (read-only for safety)",
    inputSchema: {
      type: "object",
      properties: {
        query: { type: "string", description: "SQL query to execute (SELECT only)" },
        params: {
          type: "array",
          items: { type: ["string", "number", "boolean", "null"] },
          description: "Query parameters for prepared statements"
        }
      },
      required: ["query"]
    }
  }
];

// Request handlers
server.setRequestHandler(ListToolsRequestSchema, async () => {
  return {
    tools
  };
});

server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name, arguments: args } = request.params;

  try {
    switch (name) {
      case "sb_select": {
        const params = SelectSchema.parse(args);
        guardTable(params.table);
        
        let query = supabase.from(params.table).select(params.columns);
        
        if (params.match) {
          for (const [key, value] of Object.entries(params.match)) {
            query = query.eq(key, value);
          }
        }
        
        if (params.order) {
          query = query.order(params.order.column, {
            ascending: params.order.ascending,
            nullsFirst: params.order.nullsFirst
          });
        }
        
        if (params.limit) {
          query = query.limit(params.limit);
        }
        
        const { data, error } = await query;
        
        if (error) throw new Error(`Select error: ${error.message}`);
        
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
        const params = InsertSchema.parse(args);
        guardTable(params.table);
        
        const { data, error } = await supabase
          .from(params.table)
          .insert(params.rows)
          .select();
        
        if (error) throw new Error(`Insert error: ${error.message}`);
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data, null, 2)
            }
          ]
        };
      }

      case "sb_update": {
        const params = UpdateSchema.parse(args);
        guardTable(params.table);
        
        let query = supabase.from(params.table).update(params.patch);
        
        for (const [key, value] of Object.entries(params.match)) {
          query = query.eq(key, value);
        }
        
        const { data, error } = await query.select();
        
        if (error) throw new Error(`Update error: ${error.message}`);
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data, null, 2)
            }
          ]
        };
      }

      case "sb_delete": {
        const params = DeleteSchema.parse(args);
        guardTable(params.table);
        
        let query = supabase.from(params.table).delete();
        
        for (const [key, value] of Object.entries(params.match)) {
          query = query.eq(key, value);
        }
        
        const { data, error } = await query.select();
        
        if (error) throw new Error(`Delete error: ${error.message}`);
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data, null, 2)
            }
          ]
        };
      }

      case "sb_rpc": {
        const params = RpcSchema.parse(args);
        guardRpcFunction(params.fn);
        
        const { data, error } = await supabase.rpc(params.fn, params.args);
        
        if (error) throw new Error(`RPC error: ${error.message}`);
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(data, null, 2)
            }
          ]
        };
      }

      case "sb_sql": {
        const params = ExecuteSqlSchema.parse(args);
        
        // Safety check: only allow SELECT queries
        const normalizedQuery = params.query.trim().toUpperCase();
        if (!normalizedQuery.startsWith("SELECT")) {
          throw new Error("Only SELECT queries are allowed for safety. Use other tools for mutations.");
        }
        
        // This would require an RPC function to execute arbitrary SQL
        // For now, return an informative message
        return {
          content: [
            {
              type: "text",
              text: "Direct SQL execution requires setting up an RPC function in Supabase. Use the other tools (sb_select, sb_insert, etc.) for now."
            }
          ]
        };
      }

      default:
        throw new Error(`Unknown tool: ${name}`);
    }
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    return {
      content: [
        {
          type: "text",
          text: `Error: ${errorMessage}`
        }
      ],
      isError: true
    };
  }
});

// Start the server
async function main() {
  const transport = new StdioServerTransport();
  await server.connect(transport);
  console.error("Supabase MCP server running...");
}

main().catch((error) => {
  console.error("Server error:", error);
  process.exit(1);
});