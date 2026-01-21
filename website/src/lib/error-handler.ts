import { toastError } from '../hooks/use-toast';

export class AppError extends Error {
  constructor(
    message: string,
    public code?: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'AppError';
  }
}

export function handleError(error: unknown): void {
  console.error('Error caught by handler:', error);
  
  if (error instanceof AppError) {
    toastError(error.message, error.details);
  } else if (error instanceof Error) {
    // Handle specific error types
    if (error.message.includes('Failed to fetch')) {
      toastError('Network error', 'Please check your internet connection');
    } else if (error.message.includes('401') || error.message.includes('Unauthorized')) {
      toastError('Authentication required', 'Please sign in to continue');
    } else if (error.message.includes('403') || error.message.includes('Forbidden')) {
      toastError('Access denied', 'You do not have permission to perform this action');
    } else if (error.message.includes('404') || error.message.includes('Not found')) {
      toastError('Not found', 'The requested resource could not be found');
    } else if (error.message.includes('500') || error.message.includes('Internal server')) {
      toastError('Server error', 'Something went wrong on our end. Please try again later');
    } else {
      toastError('An error occurred', error.message);
    }
  } else {
    toastError('An unexpected error occurred', 'Please try again');
  }
}

export function createAsyncHandler<T extends (...args: any[]) => Promise<any>>(
  fn: T
): T {
  return (async (...args: Parameters<T>) => {
    try {
      return await fn(...args);
    } catch (error) {
      handleError(error);
      throw error;
    }
  }) as T;
}

export function handleSupabaseError(error: any): void {
  if (error?.code === 'PGRST116') {
    handleError(new AppError('No data found', 'SUPABASE_NOT_FOUND', 404));
  } else if (error?.code === '23505') {
    handleError(new AppError('This item already exists', 'SUPABASE_DUPLICATE', 409));
  } else if (error?.code === '42501') {
    handleError(new AppError('Permission denied', 'SUPABASE_PERMISSION', 403));
  } else if (error?.message) {
    handleError(new AppError(error.message, error.code));
  } else {
    handleError(error);
  }
}