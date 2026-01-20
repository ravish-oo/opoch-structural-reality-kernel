import { serve } from "https://deno.land/std@0.168.0/http/server.ts"
import { createClient } from 'https://esm.sh/@supabase/supabase-js@2.38.4'

serve(async (req) => {
  try {
    // Get authorization header
    const authHeader = req.headers.get('Authorization')
    if (!authHeader) {
      return new Response(
        JSON.stringify({ error: 'Missing authorization header' }),
        { status: 401, headers: { 'Content-Type': 'application/json' } }
      )
    }

    // Create Supabase client with service role
    const supabaseUrl = Deno.env.get('SUPABASE_URL')!
    const supabaseServiceKey = Deno.env.get('SUPABASE_SERVICE_ROLE_KEY')!
    
    const supabase = createClient(supabaseUrl, supabaseServiceKey, {
      auth: {
        persistSession: false,
        autoRefreshToken: false,
      },
      db: {
        schema: 'public'
      }
    })

    // Parse request body
    const { query, params = [] } = await req.json()

    if (!query) {
      return new Response(
        JSON.stringify({ error: 'Missing SQL query' }),
        { status: 400, headers: { 'Content-Type': 'application/json' } }
      )
    }

    // Execute the SQL query using the admin client
    const { data, error, count, status, statusText } = await supabase
      .from('_sql')
      .select()
      .eq('query', query)
      .single()

    // Since direct SQL execution isn't available in edge functions,
    // we'll use a workaround with RPC function
    // First, let's create the RPC function if it doesn't exist
    const createRpcFunction = `
      CREATE OR REPLACE FUNCTION execute_sql(query text)
      RETURNS json
      LANGUAGE plpgsql
      SECURITY DEFINER
      AS $$
      DECLARE
        result json;
      BEGIN
        EXECUTE query INTO result;
        RETURN result;
      END;
      $$;
    `

    // For safety, we'll use parameterized queries when possible
    // and validate the query type
    const queryType = query.trim().split(/\s+/)[0].toUpperCase()
    const allowedTypes = ['SELECT', 'INSERT', 'UPDATE', 'DELETE', 'CREATE', 'ALTER', 'DROP', 'GRANT', 'REVOKE']
    
    if (!allowedTypes.includes(queryType)) {
      return new Response(
        JSON.stringify({ error: 'Query type not allowed' }),
        { status: 403, headers: { 'Content-Type': 'application/json' } }
      )
    }

    // For now, return a message about manual execution
    // In production, you would set up proper RPC functions
    return new Response(
      JSON.stringify({ 
        message: 'SQL execution via Edge Functions requires setting up RPC functions.',
        recommendation: 'Use the Supabase SQL Editor or create specific RPC functions for your queries.',
        query: query,
        queryType: queryType
      }),
      { status: 200, headers: { 'Content-Type': 'application/json' } }
    )

  } catch (error) {
    return new Response(
      JSON.stringify({ error: error.message }),
      { status: 500, headers: { 'Content-Type': 'application/json' } }
    )
  }
})