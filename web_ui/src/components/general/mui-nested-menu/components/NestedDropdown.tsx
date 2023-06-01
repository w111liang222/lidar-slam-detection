import React from "react";
import { nestedMenuItemsFromObject } from "./nestedMenuItemsFromObject";
import Button, { ButtonProps } from "@mui/material/Button";
import Menu, { MenuProps } from "@mui/material/Menu";
import { ChevronDown } from "../icons/ChevronDown";
import { MenuItemData } from "..";

interface NestedDropdownProps {
  children?: React.ReactNode;
  menuItemsData?: any;
  onClick?: (e: React.MouseEvent<HTMLButtonElement>) => void;
  ButtonProps?: Partial<ButtonProps>;
  MenuProps?: Partial<MenuProps>;
}

export const NestedDropdown = React.forwardRef<HTMLDivElement | null, NestedDropdownProps>(function NestedDropdown(
  props,
  ref
) {
  const [anchorEl, setAnchorEl] = React.useState<any>(null);
  const open = Boolean(anchorEl);

  const { menuItemsData: data, onClick, ButtonProps, MenuProps, ...rest } = props;

  const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    setAnchorEl(e.currentTarget);
    onClick && onClick(e);
  };
  const handleClose = () => setAnchorEl(null);

  const menuItems = nestedMenuItemsFromObject({
    menuItemsData: data.items,
    isOpen: open,
    handleClose,
  });

  return (
    <div ref={ref} {...rest}>
      <Button onClick={handleClick} endIcon={<ChevronDown />} {...ButtonProps}>
        {data?.label}
      </Button>
      <Menu anchorEl={anchorEl} open={open} onClose={handleClose} {...MenuProps}>
        {menuItems}
      </Menu>
    </div>
  );
});
