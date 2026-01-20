# Security Guidelines

## Environment Variables

### Client-side Variables (Safe to expose)
- `VITE_SUPABASE_URL`: Your Supabase project URL
- `VITE_SUPABASE_ANON_KEY`: Public anonymous key for client-side authentication

### Server-side Variables (NEVER expose to client)
- `SUPABASE_SERVICE_ROLE_KEY`: Service role key with full database access
- `SUPABASE_JWT_SECRET`: JWT secret for token verification
- `DATABASE_URL`: Direct database connection string
- `DIRECT_URL`: Direct database URL without connection pooling

## Setup Instructions

1. Copy `.env.example` to `.env.local`
2. Fill in your Supabase credentials.
3. Never commit `.env.local` or any file containing real credentials
4. Use environment variables from hosting providers for production

## Best Practices

1. **Row Level Security (RLS)**: Always enable RLS on Supabase tables
2. **API Keys**: Use the anon key for client-side, service role only for server-side
3. **Authentication**: Implement proper user authentication before database operations
4. **Validation**: Always validate user input on both client and server
5. **HTTPS**: Ensure all API calls use HTTPS in production

## Reporting Security Issues

If you discover a security vulnerability, please email security@opoch.com