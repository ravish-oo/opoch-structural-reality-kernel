import { type ClassValue, clsx } from "clsx"
import { twMerge } from "tailwind-merge"

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}

// Legacy classNames function for backward compatibility
export function classNames(...xs: (string | false | undefined)[]) {
  return xs.filter(Boolean).join(" ");
}