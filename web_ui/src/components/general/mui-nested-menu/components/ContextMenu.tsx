import React, { useState, forwardRef, useRef } from "react";
import Menu from "@mui/material/Menu";
import { nestedMenuItemsFromObject } from "./nestedMenuItemsFromObject";
import { MenuItemData } from "../definitions";

export interface ContextMenuProps {
  children?: React.ReactNode;
  menuItems?: React.ReactNode[];
  menuItemsData?: MenuItemData[];
}

interface Position {
  top: number;
  left: number;
}

const ContextMenu = forwardRef<HTMLDivElement, ContextMenuProps>(({ children, menuItems, menuItemsData }, ref) => {
  const wrapperRef = (ref as React.MutableRefObject<HTMLDivElement>) ?? useRef<HTMLDivElement>(null);

  const [menuPosition, setMenuPosition] = useState<Position>();

  const [mouseDownPosition, setMouseDownPosition] = useState<Position>();

  const handleItemClick = () => setMenuPosition(undefined);

  const handleMouseDown = (e: React.MouseEvent) => {
    if (menuPosition !== null) setMenuPosition(undefined);

    if (e.button !== 2) return;

    const wrapperBounds = wrapperRef.current.getBoundingClientRect();

    // Check mouse coordinates are inside the rect
    if (
      e.clientX < wrapperBounds.left ||
      e.clientX > wrapperBounds.right ||
      e.clientY < wrapperBounds.top ||
      e.clientY > wrapperBounds.bottom
    ) {
      return;
    }

    setMouseDownPosition({ top: e.clientY, left: e.clientX });
  };

  const handleMouseUp = (e: React.MouseEvent) => {
    const top = e.clientY;
    const left = e.clientX;

    if (mouseDownPosition == null) return;

    if (mouseDownPosition.top === top && mouseDownPosition.left === left) {
      setMenuPosition({ top: e.clientY, left: e.clientX });
    }
  };

  const menuContents =
    menuItems ??
    (menuItemsData &&
      nestedMenuItemsFromObject({
        menuItemsData: menuItemsData,
        isOpen: !!menuPosition,
        handleClose: handleItemClick,
      }));

  return (
    <div
      ref={wrapperRef}
      onContextMenu={(e) => e.preventDefault()}
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}>
      <Menu
        onContextMenu={(e) => e.preventDefault()}
        open={!!menuPosition}
        onClose={() => setMenuPosition(undefined)}
        anchorReference="anchorPosition"
        anchorPosition={menuPosition}>
        {menuContents}
      </Menu>
      {children}
    </div>
  );
});

ContextMenu.displayName = "ContextMenu";
export { ContextMenu };
