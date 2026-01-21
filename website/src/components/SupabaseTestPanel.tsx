import { useState } from 'react'
import { testSupabaseConnection, isSupabaseConfigured, supabase } from '../lib/supabase'
import { Button } from './ui/button'
import { Card, CardContent } from './ui/card'

export default function SupabaseTestPanel() {
  const [testResult, setTestResult] = useState<any>(null)
  const [isLoading, setIsLoading] = useState(false)

  const runTest = async () => {
    setIsLoading(true)
    setTestResult(null)

    try {
      const result = await testSupabaseConnection()
      setTestResult(result)
    } catch (error) {
      setTestResult({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      })
    } finally {
      setIsLoading(false)
    }
  }

  const testInsert = async () => {
    setIsLoading(true)
    try {
      const { data, error } = await supabase
        .from('leads')
        .insert({
          name: 'Test User',
          email: 'test@example.com',
          designation: 'Developer',
          organization: 'Test Org',
          reason: 'Testing connection',
          source: 'test-panel'
        })
        .select()

      setTestResult({
        success: !error,
        message: error ? error.message : 'Insert successful',
        data: data
      })
    } catch (error) {
      setTestResult({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      })
    } finally {
      setIsLoading(false)
    }
  }

  const configured = isSupabaseConfigured()

  return (
    <Card className="w-full max-w-2xl mx-auto mt-8">
      <CardContent className="p-6">
        <h3 className="text-lg font-semibold mb-4">🔧 Supabase Connection Test</h3>
        
        <div className="space-y-4">
          <div className="flex items-center gap-2">
            <span className={configured ? "text-green-600" : "text-red-600"}>
              {configured ? "✅" : "❌"}
            </span>
            <span>Configuration: {configured ? "Ready" : "Missing environment variables"}</span>
          </div>

          {configured && (
            <div className="space-y-2">
              <div className="flex gap-2">
                <Button 
                  onClick={runTest} 
                  disabled={isLoading}
                  size="sm"
                >
                  {isLoading ? "Testing..." : "Test Connection"}
                </Button>
                
                <Button 
                  onClick={testInsert} 
                  disabled={isLoading}
                  size="sm"
                  variant="outline"
                >
                  {isLoading ? "Testing..." : "Test Insert"}
                </Button>
              </div>
            </div>
          )}

          {testResult && (
            <div className={`p-4 rounded-md border ${
              testResult.success 
                ? 'bg-green-50 border-green-200 text-green-800' 
                : 'bg-red-50 border-red-200 text-red-800'
            }`}>
              <div className="font-semibold">
                {testResult.success ? "✅ Success" : "❌ Error"}
              </div>
              <div className="text-sm mt-1">
                {testResult.message || testResult.error}
              </div>
              {testResult.data && (
                <pre className="text-xs mt-2 bg-gray-100 p-2 rounded overflow-auto">
                  {JSON.stringify(testResult.data, null, 2)}
                </pre>
              )}
            </div>
          )}

          {!configured && (
            <div className="bg-yellow-50 border border-yellow-200 p-4 rounded-md">
              <h4 className="font-semibold text-yellow-800">Setup Required:</h4>
              <ol className="text-sm text-yellow-700 mt-2 space-y-1">
                <li>1. Copy your Supabase URL and anon key from the dashboard</li>
                <li>2. Create a .env.local file in the project root</li>
                <li>3. Add the environment variables</li>
                <li>4. Restart the dev server</li>
              </ol>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
