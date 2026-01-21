import * as React from 'react'
import PNI from 'react-phone-number-input'
import type { Value } from 'react-phone-number-input'
import 'react-phone-number-input/style.css'
import { cn } from '../../lib/utils'

interface PhoneInputProps {
  value?: Value
  onChange?: (value: Value | undefined) => void
  placeholder?: string
  className?: string
  disabled?: boolean
  error?: boolean
}

// Stable, top-level input component so the library doesn't remount
const StyledPhoneField = React.forwardRef<HTMLInputElement, React.InputHTMLAttributes<HTMLInputElement>>(
  (props, ref) => (
    <input
      ref={ref}
      {...props}
      className={cn(
        "flex h-10 w-full rounded-md border border-white/10 bg-white/5 px-3 py-2 text-sm text-white placeholder:text-white/50 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-white/20 disabled:cursor-not-allowed disabled:opacity-50",
        props.className
      )}
    />
  )
)
StyledPhoneField.displayName = 'StyledPhoneField'

export const PhoneInput = React.memo(
  React.forwardRef<HTMLDivElement, PhoneInputProps>(function PhoneInput(
    { value, onChange, placeholder = "Phone number", className, disabled, error },
    ref
  ) {
    return (
      <div ref={ref}>
        <PNI
          international
          defaultCountry="US"
          value={value}
          onChange={onChange || (() => {})}
          placeholder={placeholder}
          disabled={disabled}
          inputComponent={StyledPhoneField as any}
          className={cn(
            "phone-input-container",
            error && "[&_input]:border-red-500/50 [&_input]:focus-visible:ring-red-500/20",
            className
          )}
        />
      </div>
    )
  })
)