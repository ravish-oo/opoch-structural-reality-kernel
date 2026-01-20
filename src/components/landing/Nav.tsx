import React from "react";
import { Link } from "react-router-dom";
import { Sparkles } from "lucide-react";
import { Button } from "../ui/button";
import OpochLogo from "../OpochLogo";
import UserMenu from "../UserMenu";
import Tag from "../Tag";
import { useAuth } from "../../contexts/AuthContext";

interface NavProps {
  openApply: () => void;
}

const Nav = React.memo(({ openApply }: NavProps) => {
  const { user } = useAuth();
  
  return (
    <div className="sticky top-0 z-40 border-b border-white/10 bg-black/80 backdrop-blur">
      <div className="mx-auto flex max-w-7xl items-center justify-between px-4 py-4">
        <div className="flex items-center gap-3">
          <Link to="/" className="transition-colors duration-200 hover:opacity-80">
            <OpochLogo width={100} height={32} />
          </Link>
          <Tag><Sparkles className="h-3 w-3" /> Age of Truth</Tag>
        </div>
        <div className="hidden items-center gap-6 md:flex">
          <a className="text-sm text-white/80 hover:text-white" href="#process">How it works</a>
          <a className="text-sm text-white/80 hover:text-white" href="#who">Who it's for</a>
          <Link to="/moonshots" className="text-sm text-white/80 hover:text-white">Moonshots</Link>
          <a className="text-sm text-white/80 hover:text-white" href="#rbt">RBT</a>
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" onClick={openApply}>Apply — $11,111/mo</Button>
          {user && <UserMenu />}
        </div>
        <div className="flex items-center gap-2 md:hidden">
          {user && <UserMenu />}
          <Button className="rounded-2xl bg-white text-black hover:bg-white/90" size="sm" onClick={openApply}>Apply</Button>
        </div>
      </div>
    </div>
  );
});

Nav.displayName = "Nav";

export default Nav;