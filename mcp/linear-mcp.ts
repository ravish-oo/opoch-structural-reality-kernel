#!/usr/bin/env node
import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from '@modelcontextprotocol/sdk/types.js';
import axios from 'axios';

// Linear API configuration
const LINEAR_API_URL = 'https://api.linear.app/graphql';
const LINEAR_API_KEY = process.env.LINEAR_API_KEY;

if (!LINEAR_API_KEY) {
  console.error('LINEAR_API_KEY environment variable is required');
  process.exit(1);
}

console.error(`Connecting to Linear API`);

// Create axios instance with auth header
const linearClient = axios.create({
  baseURL: LINEAR_API_URL,
  headers: {
    'Authorization': LINEAR_API_KEY,
    'Content-Type': 'application/json',
  },
});

// GraphQL query helper
async function graphql(query: string, variables?: any) {
  try {
    const response = await linearClient.post('', {
      query,
      variables,
    });
    
    if (response.data.errors) {
      throw new Error(response.data.errors[0].message);
    }
    
    return response.data.data;
  } catch (error: any) {
    console.error('Linear API error:', error.message);
    throw error;
  }
}

// Create server
const server = new Server(
  {
    name: 'linear-mcp',
    version: '1.0.0',
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
        name: 'linear_list_issues',
        description: 'List issues from Linear with optional filters',
        inputSchema: {
          type: 'object',
          properties: {
            teamId: { type: 'string', description: 'Team ID to filter issues' },
            assigneeId: { type: 'string', description: 'User ID to filter by assignee' },
            stateId: { type: 'string', description: 'State ID to filter issues' },
            includeArchived: { type: 'boolean', description: 'Include archived issues' },
            limit: { type: 'number', description: 'Number of issues to return' },
            orderBy: { type: 'string', description: 'Field to order by' },
          },
        },
      },
      {
        name: 'linear_create_issue',
        description: 'Create a new issue in Linear',
        inputSchema: {
          type: 'object',
          properties: {
            title: { type: 'string', description: 'Issue title' },
            description: { type: 'string', description: 'Issue description in Markdown' },
            teamId: { type: 'string', description: 'Team ID for the issue' },
            priority: { type: 'number', description: 'Priority (0=No priority, 1=Urgent, 2=High, 3=Medium, 4=Low)' },
            assigneeId: { type: 'string', description: 'User ID to assign the issue' },
            labelIds: { type: 'array', items: { type: 'string' }, description: 'Array of label IDs' },
            projectId: { type: 'string', description: 'Project ID to add issue to' },
          },
          required: ['title', 'teamId'],
        },
      },
      {
        name: 'linear_update_issue',
        description: 'Update an existing issue in Linear',
        inputSchema: {
          type: 'object',
          properties: {
            issueId: { type: 'string', description: 'Issue ID to update' },
            title: { type: 'string', description: 'New title' },
            description: { type: 'string', description: 'New description' },
            stateId: { type: 'string', description: 'New state ID' },
            priority: { type: 'number', description: 'New priority' },
            assigneeId: { type: 'string', description: 'New assignee ID' },
            labelIds: { type: 'array', items: { type: 'string' }, description: 'New label IDs' },
          },
          required: ['issueId'],
        },
      },
      {
        name: 'linear_get_issue',
        description: 'Get details of a specific issue',
        inputSchema: {
          type: 'object',
          properties: {
            issueId: { type: 'string', description: 'Issue ID to fetch' },
          },
          required: ['issueId'],
        },
      },
      {
        name: 'linear_create_comment',
        description: 'Add a comment to an issue',
        inputSchema: {
          type: 'object',
          properties: {
            issueId: { type: 'string', description: 'Issue ID to comment on' },
            body: { type: 'string', description: 'Comment body in Markdown' },
          },
          required: ['issueId', 'body'],
        },
      },
      {
        name: 'linear_list_teams',
        description: 'List all teams in the workspace',
        inputSchema: {
          type: 'object',
          properties: {},
        },
      },
      {
        name: 'linear_list_projects',
        description: 'List projects for a team',
        inputSchema: {
          type: 'object',
          properties: {
            teamId: { type: 'string', description: 'Team ID to filter projects' },
          },
        },
      },
      {
        name: 'linear_search_issues',
        description: 'Search issues by query',
        inputSchema: {
          type: 'object',
          properties: {
            query: { type: 'string', description: 'Search query' },
            includeArchived: { type: 'boolean', description: 'Include archived issues' },
            limit: { type: 'number', description: 'Number of results' },
          },
          required: ['query'],
        },
      },
      {
        name: 'linear_list_views',
        description: 'List all custom views',
        inputSchema: {
          type: 'object',
          properties: {
            teamId: { type: 'string', description: 'Filter by team ID' },
            shared: { type: 'boolean', description: 'Filter by shared status' },
          },
        },
      },
      {
        name: 'linear_create_view',
        description: 'Create a new custom view',
        inputSchema: {
          type: 'object',
          properties: {
            name: { type: 'string', description: 'View name' },
            description: { type: 'string', description: 'View description' },
            teamId: { type: 'string', description: 'Team ID for the view' },
            icon: { type: 'string', description: 'Icon name (e.g., "Code", "Alert", "Users")' },
            color: { type: 'string', description: 'Icon color (hex format)' },
            shared: { type: 'boolean', description: 'Share with team' },
            filterData: { type: 'object', description: 'Issue filter object' },
          },
          required: ['name'],
        },
      },
      {
        name: 'linear_update_view',
        description: 'Update an existing custom view',
        inputSchema: {
          type: 'object',
          properties: {
            viewId: { type: 'string', description: 'Custom view ID to update' },
            name: { type: 'string', description: 'New name' },
            description: { type: 'string', description: 'New description' },
            icon: { type: 'string', description: 'New icon' },
            color: { type: 'string', description: 'New color' },
            shared: { type: 'boolean', description: 'New sharing status' },
            filterData: { type: 'object', description: 'New filter data' },
          },
          required: ['viewId'],
        },
      },
      {
        name: 'linear_delete_view',
        description: 'Delete a custom view',
        inputSchema: {
          type: 'object',
          properties: {
            viewId: { type: 'string', description: 'Custom view ID to delete' },
          },
          required: ['viewId'],
        },
      },
    ],
  };
});

// Handle tool execution
server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name, arguments: args } = request.params;

  try {
    let result: any;

    switch (name) {
      case 'linear_list_issues': {
        const filters: string[] = [];
        if (args.teamId) filters.push(`team: { id: { eq: "${args.teamId}" } }`);
        if (args.assigneeId) filters.push(`assignee: { id: { eq: "${args.assigneeId}" } }`);
        if (args.stateId) filters.push(`state: { id: { eq: "${args.stateId}" } }`);
        if (!args.includeArchived) filters.push('archivedAt: { null: true }');

        const filterString = filters.length > 0 ? `filter: { ${filters.join(', ')} }` : '';
        
        const query = `
          query ListIssues($limit: Int!, $orderBy: PaginationOrderBy!) {
            issues(first: $limit, orderBy: $orderBy, ${filterString}) {
              nodes {
                id
                identifier
                title
                description
                priority
                priorityLabel
                state { name type }
                assignee { name email }
                labels { nodes { name color } }
                createdAt
                updatedAt
              }
            }
          }
        `;
        
        result = await graphql(query, {
          limit: args.limit || 20,
          orderBy: args.orderBy || 'createdAt',
        });
        break;
      }

      case 'linear_create_issue': {
        const mutation = `
          mutation CreateIssue($input: IssueCreateInput!) {
            issueCreate(input: $input) {
              success
              issue {
                id
                identifier
                title
                url
              }
            }
          }
        `;
        
        const input: any = {
          title: args.title,
          teamId: args.teamId,
        };
        
        if (args.description) input.description = args.description;
        if (args.priority !== undefined) input.priority = args.priority;
        if (args.assigneeId) input.assigneeId = args.assigneeId;
        if (args.labelIds) input.labelIds = args.labelIds;
        if (args.projectId) input.projectId = args.projectId;
        
        result = await graphql(mutation, { input });
        break;
      }

      case 'linear_update_issue': {
        const mutation = `
          mutation UpdateIssue($id: String!, $input: IssueUpdateInput!) {
            issueUpdate(id: $id, input: $input) {
              success
              issue {
                id
                identifier
                title
                description
                state { name }
              }
            }
          }
        `;
        
        const input: any = {};
        if (args.title) input.title = args.title;
        if (args.description) input.description = args.description;
        if (args.stateId) input.stateId = args.stateId;
        if (args.priority !== undefined) input.priority = args.priority;
        if (args.assigneeId) input.assigneeId = args.assigneeId;
        if (args.labelIds) input.labelIds = args.labelIds;
        
        result = await graphql(mutation, { id: args.issueId, input });
        break;
      }

      case 'linear_get_issue': {
        const query = `
          query GetIssue($id: String!) {
            issue(id: $id) {
              id
              identifier
              title
              description
              priority
              priorityLabel
              state { name type }
              assignee { name email }
              team { name key }
              labels { nodes { name color } }
              comments { nodes { body createdAt user { name } } }
              createdAt
              updatedAt
              url
            }
          }
        `;
        
        result = await graphql(query, { id: args.issueId });
        break;
      }

      case 'linear_create_comment': {
        const mutation = `
          mutation CreateComment($issueId: String!, $body: String!) {
            commentCreate(input: { issueId: $issueId, body: $body }) {
              success
              comment {
                id
                body
                createdAt
              }
            }
          }
        `;
        
        result = await graphql(mutation, {
          issueId: args.issueId,
          body: args.body,
        });
        break;
      }

      case 'linear_list_teams': {
        const query = `
          query ListTeams {
            teams {
              nodes {
                id
                name
                key
                description
              }
            }
          }
        `;
        
        result = await graphql(query);
        break;
      }

      case 'linear_list_projects': {
        const query = `
          query ListProjects($teamId: String) {
            projects(filter: { team: { id: { eq: $teamId } } }) {
              nodes {
                id
                name
                description
                state
                team { name key }
              }
            }
          }
        `;
        
        result = await graphql(query, { teamId: args.teamId });
        break;
      }

      case 'linear_search_issues': {
        const query = `
          query SearchIssues($query: String!, $includeArchived: Boolean!, $limit: Int!) {
            issueSearch(query: $query, includeArchived: $includeArchived, first: $limit) {
              nodes {
                id
                identifier
                title
                description
                state { name }
                assignee { name }
                priority
                createdAt
              }
            }
          }
        `;
        
        result = await graphql(query, {
          query: args.query,
          includeArchived: args.includeArchived || false,
          limit: args.limit || 20,
        });
        break;
      }

      case 'linear_list_views': {
        const filters: string[] = [];
        if (args.teamId) filters.push(`team: { id: { eq: "${args.teamId}" } }`);
        if (args.shared !== undefined) filters.push(`shared: { eq: ${args.shared} }`);
        
        const filterString = filters.length > 0 ? `filter: { ${filters.join(', ')} }` : '';
        
        const query = `
          query ListCustomViews {
            customViews(${filterString}) {
              nodes {
                id
                name
                description
                icon
                color
                shared
                team { id name }
                filterData
                createdAt
                updatedAt
              }
            }
          }
        `;
        
        result = await graphql(query);
        break;
      }

      case 'linear_create_view': {
        const mutation = `
          mutation CreateCustomView($input: CustomViewCreateInput!) {
            customViewCreate(input: $input) {
              success
              customView {
                id
                name
                description
                icon
                color
                shared
                team { name }
              }
            }
          }
        `;
        
        const input: any = {
          name: args.name,
        };
        
        if (args.description) input.description = args.description;
        if (args.teamId) input.teamId = args.teamId;
        if (args.icon) input.icon = args.icon;
        if (args.color) input.color = args.color;
        if (args.shared !== undefined) input.shared = args.shared;
        if (args.filterData) input.filterData = args.filterData;
        
        result = await graphql(mutation, { input });
        break;
      }

      case 'linear_update_view': {
        const mutation = `
          mutation UpdateCustomView($id: String!, $input: CustomViewUpdateInput!) {
            customViewUpdate(id: $id, input: $input) {
              success
              customView {
                id
                name
                description
                icon
                color
                shared
              }
            }
          }
        `;
        
        const input: any = {};
        if (args.name) input.name = args.name;
        if (args.description) input.description = args.description;
        if (args.icon) input.icon = args.icon;
        if (args.color) input.color = args.color;
        if (args.shared !== undefined) input.shared = args.shared;
        if (args.filterData) input.filterData = args.filterData;
        
        result = await graphql(mutation, { id: args.viewId, input });
        break;
      }

      case 'linear_delete_view': {
        const mutation = `
          mutation DeleteCustomView($id: String!) {
            customViewDelete(id: $id) {
              success
            }
          }
        `;
        
        result = await graphql(mutation, { id: args.viewId });
        break;
      }

      default:
        throw new Error(`Unknown tool: ${name}`);
    }

    return {
      content: [
        {
          type: 'text',
          text: JSON.stringify(result, null, 2),
        },
      ],
    };
  } catch (error: any) {
    return {
      content: [
        {
          type: 'text',
          text: `Error: ${error.message}`,
        },
      ],
      isError: true,
    };
  }
});

// Start server
async function main() {
  const transport = new StdioServerTransport();
  await server.connect(transport);
  console.error('Linear MCP server started');
}

main().catch((error) => {
  console.error('Server error:', error);
  process.exit(1);
});