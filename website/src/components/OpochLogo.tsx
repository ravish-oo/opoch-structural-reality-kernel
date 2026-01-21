
interface OpochLogoProps {
  className?: string;
  width?: number;
  height?: number;
}

export default function OpochLogo({ className = "", width = 120, height = 40 }: OpochLogoProps) {
  return (
    <img 
      src="/Opoch Assets/Opoch Typelogo.svg" 
      alt="Opoch" 
      width={width} 
      height={height}
      className={className}
    />
  );
}