import { describe, it, expect } from 'vitest'
import { render, screen } from '@testing-library/react'
import OpochLogo from './OpochLogo'

describe('OpochLogo', () => {
  it('renders with default props', () => {
    render(<OpochLogo />)
    const logo = screen.getByAltText('Opoch')
    expect(logo).toBeInTheDocument()
    expect(logo).toHaveAttribute('width', '120')
    expect(logo).toHaveAttribute('height', '40')
  })

  it('renders with custom dimensions', () => {
    render(<OpochLogo width={200} height={60} />)
    const logo = screen.getByAltText('Opoch')
    expect(logo).toHaveAttribute('width', '200')
    expect(logo).toHaveAttribute('height', '60')
  })

  it('applies custom className', () => {
    render(<OpochLogo className="custom-class" />)
    const logo = screen.getByAltText('Opoch')
    expect(logo).toHaveClass('custom-class')
  })
})