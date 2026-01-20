import type { VercelRequest, VercelResponse } from '@vercel/node'
import { Resend } from 'resend'
import { createClient } from '@supabase/supabase-js'

const resend = new Resend(process.env.RESEND_API_KEY)
const supabase = createClient(
  process.env.VITE_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_ROLE_KEY!
)

interface SendEmailRequest {
  templateName: string
  to: string
  variables: Record<string, string>
  leadId?: string
}

// Email validation regex
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  // Set CORS headers for all requests
  const origin = req.headers.origin
  const allowedOrigins = ['https://opoch.com', 'https://www.opoch.com', 'http://localhost:5173', 'http://localhost:3000']
  
  if (origin && allowedOrigins.includes(origin)) {
    res.setHeader('Access-Control-Allow-Origin', origin)
  } else if (process.env.ALLOWED_ORIGIN) {
    res.setHeader('Access-Control-Allow-Origin', process.env.ALLOWED_ORIGIN)
  } else {
    res.setHeader('Access-Control-Allow-Origin', 'https://opoch.com')
  }
  
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS')
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type')
  res.setHeader('Access-Control-Max-Age', '86400')

  // Handle OPTIONS request for CORS preflight
  if (req.method === 'OPTIONS') {
    return res.status(200).end()
  }

  // Only allow POST requests for actual email sending
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' })
  }

  try {
    const { templateName, to, variables, leadId } = req.body as SendEmailRequest

    // Validate input
    if (!templateName || typeof templateName !== 'string') {
      return res.status(400).json({ error: 'Invalid template name' })
    }

    if (!to || !emailRegex.test(to)) {
      return res.status(400).json({ error: 'Invalid email address' })
    }

    if (!variables || typeof variables !== 'object') {
      return res.status(400).json({ error: 'Invalid variables' })
    }

    // Sanitize variables to prevent injection
    const sanitizedVariables: Record<string, string> = {}
    for (const [key, value] of Object.entries(variables)) {
      if (typeof value === 'string') {
        sanitizedVariables[key] = value.slice(0, 1000) // Limit length
      }
    }

    // Fetch email template from Supabase
    const { data: template, error: templateError } = await supabase
      .from('email_templates')
      .select('*')
      .eq('name', templateName)
      .single()

    if (templateError || !template) {
      return res.status(404).json({ error: 'Template not found' })
    }

    // Replace variables in subject and body using sanitized values
    let subject = template.subject
    let htmlBody = template.html_body
    let textBody = template.text_body

    Object.entries(sanitizedVariables).forEach(([key, value]) => {
      const regex = new RegExp(`{{${key}}}`, 'g')
      subject = subject.replace(regex, value)
      htmlBody = htmlBody.replace(regex, value)
      textBody = textBody?.replace(regex, value) || ''
    })

    // Send email via Resend
    const { data: emailData, error: sendError } = await resend.emails.send({
      from: 'Opoch <hello@opoch.com>',
      to,
      reply_to: 'hello@opoch.com',
      subject,
      html: htmlBody,
      text: textBody,
    })

    if (sendError) {
      throw sendError
    }

    // Log email sent to database
    const { error: logError } = await supabase.from('emails_sent').insert({
      to_email: to,
      template_name: templateName,
      subject,
      status: 'sent',
      provider: 'resend',
      provider_id: emailData?.id,
      metadata: { variables: sanitizedVariables },
      lead_id: leadId,
    })

    if (logError) {
      console.error('Failed to log email:', logError)
    }

    return res.status(200).json({ success: true, id: emailData?.id })
  } catch (error) {
    console.error('Email send error:', error)
    
    // Log failed email
    try {
      await supabase.from('emails_sent').insert({
        to_email: req.body.to,
        template_name: req.body.templateName,
        subject: 'Failed to send',
        status: 'failed',
        provider: 'resend',
        error: error instanceof Error ? error.message : 'Unknown error',
        metadata: { variables: req.body.variables },
        lead_id: req.body.leadId,
      })
    } catch (logError) {
      console.error('Failed to log error:', logError)
    }

    return res.status(500).json({ error: 'Failed to send email' })
  }
}