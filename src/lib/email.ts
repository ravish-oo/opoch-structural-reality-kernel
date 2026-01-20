// Email utility functions for client-side email triggering

interface SendEmailParams {
  templateName: string
  to: string
  variables: Record<string, string>
  leadId?: string
}

export async function sendEmail(params: SendEmailParams) {
  try {
    // In development, just log the email
    if (import.meta.env.DEV) {
      console.log('📧 Email would be sent:', params)
      return { success: true, id: 'dev-' + Date.now() }
    }

    // In production, call the Vercel serverless function
    const response = await fetch('/api/send-email', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(params),
    })

    if (!response.ok) {
      throw new Error(`Failed to send email: ${response.statusText}`)
    }

    const result = await response.json()
    return { success: true, ...result }
  } catch (error) {
    console.error('Failed to send email:', error)
    return { success: false, error }
  }
}

// Helper function to send welcome email
export async function sendWelcomeEmail(name: string, email: string, leadId?: string) {
  return sendEmail({
    templateName: 'welcome',
    to: email,
    variables: { name },
    leadId,
  })
}

// Helper function to send application received email
export async function sendApplicationReceivedEmail(
  name: string,
  email: string,
  researchQuery: string,
  leadId?: string
) {
  return sendEmail({
    templateName: 'apply-received',
    to: email,
    variables: {
      name,
      research_query: researchQuery || 'Not specified',
      lead_id: leadId || '',
    },
    leadId,
  })
}