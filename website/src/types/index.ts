// Component Props Types
export interface BaseComponentProps {
  className?: string
  children?: React.ReactNode
}

// Form Types
export interface FormField {
  name: string
  label: string
  type: 'text' | 'email' | 'tel' | 'textarea' | 'select'
  required?: boolean
  placeholder?: string
  validation?: Record<string, any>
}

export interface FormValues {
  [key: string]: any
}

// API Response Types
export interface ApiResponse<T = any> {
  data?: T
  error?: ApiError
  success: boolean
}

export interface ApiError {
  message: string
  code?: string
  details?: Record<string, any>
}

// Lead Types
export interface Lead {
  id: string
  name: string
  email: string
  phone?: string
  designation: string
  organization: string
  reason: string
  research_query?: string
  source?: string
  utm?: Record<string, any>
  created_at: string
  updated_at: string
}

// Query Types
export interface Query {
  id: string
  text: string
  context?: Record<string, any>
  created_at: string
  user_id?: string
}

// Update/Article Types
export interface Update {
  id: string
  title: string
  slug: string
  excerpt: string
  content: string
  publishedAt: string
  tags: string[]
  featured?: boolean
  author?: string
}

// Moonshot Types
export interface Moonshot {
  id: string
  category: string
  problem: string
  approach: string
  impact: string
  scale: string
  timeline: string
  rbt_overlap: number
}

// Auth Types
export interface AuthUser {
  id: string
  email?: string
  user_metadata?: {
    full_name?: string
    avatar_url?: string
  }
}

export interface Profile {
  id: string
  email: string
  role: 'user' | 'admin'
  full_name?: string
  created_at: string
  updated_at: string
}

// Email Types
export interface EmailLog {
  id: string
  to: string
  subject: string
  template: string
  status: 'sent' | 'failed' | 'pending'
  error?: string
  metadata?: Record<string, any>
  created_at: string
}

// SEO Types
export interface BreadcrumbItem {
  name: string
  url: string
}

export interface ArticleMetadata {
  publishedTime: string
  author?: string
  section?: string
  tags?: string[]
}

export interface PageMetadata {
  title: string
  description: string
  image: string
  url: string
  type: 'website' | 'article'
  keywords?: string[]
  article?: ArticleMetadata
  breadcrumbs?: BreadcrumbItem[]
}

// Component State Types
export interface LoadingState {
  isLoading: boolean
  error?: ApiError
}

export interface PaginationState {
  page: number
  limit: number
  total: number
  hasMore: boolean
}

// Event Handler Types
export type FormSubmitHandler<T = FormValues> = (values: T) => void | Promise<void>
export type ClickHandler = (event: React.MouseEvent<HTMLElement>) => void
export type ChangeHandler<T = HTMLInputElement> = (event: React.ChangeEvent<T>) => void