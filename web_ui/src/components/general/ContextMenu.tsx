import { MenuItem, MenuList, Popover } from "@mui/material";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";

export type Props = {
  onStart?: () => void;
  enabled?: boolean;
  menuItems: { [index: string]: () => void };
};

export default function ContextMenu({ onStart, enabled, menuItems }: Props) {
  const { t } = useTranslation();

  const [menuPosition, setMenuPosition] = useState({ left: 0, top: 0 });
  const [menuVisibility, setMenuVisibility] = useState(false);

  useEffect(() => {
    const showMenu = (ev: MouseEvent) => {
      setMenuVisibility(true);
      setMenuPosition({
        left: ev.clientX,
        top: ev.clientY,
      });
    };
    document.body.oncontextmenu = (ev) => {
      onStart && onStart();
      if (enabled) {
        showMenu(ev);
      }
    };
    return () => {
      document.body.oncontextmenu = null;
    };
  }, [menuItems]);

  return (
    <Popover
      anchorReference="anchorPosition"
      anchorPosition={menuPosition}
      open={menuVisibility}
      onClose={() => setMenuVisibility(false)}>
      <MenuList>
        {Object.entries(menuItems).map((kv) => (
          <MenuItem
            key={kv[0]}
            onClick={() => {
              kv[1]();
              setMenuVisibility(false);
            }}>
            {t(kv[0])}
          </MenuItem>
        ))}
      </MenuList>
    </Popover>
  );
}
