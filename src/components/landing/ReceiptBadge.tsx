import React from "react";
import { classNames } from "../../lib/utils";

interface ReceiptBadgeProps {
  state?: "PASS" | "Ø" | "Ø_∞";
}

const ReceiptBadge = React.memo(({ state = "PASS" }: ReceiptBadgeProps) => {
  const colorMap = {
    "PASS": "bg-emerald-500/20 text-emerald-300",
    "Ø": "bg-amber-500/20 text-amber-300",
    "Ø_∞": "bg-sky-500/20 text-sky-300"
  };
  
  const color = colorMap[state];
  
  return (
    <span className={classNames("rounded-md px-2 py-0.5 text-xs font-medium", color)}>
      {state}
    </span>
  );
});

ReceiptBadge.displayName = "ReceiptBadge";

export default ReceiptBadge;