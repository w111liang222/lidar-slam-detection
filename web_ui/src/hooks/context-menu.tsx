import { MenuItem, MenuList, Popover } from "@mui/material";
import React, { useEffect, useState } from "react";

export type Props = {
  menuItems: { [index: string]: () => void };
  t?: (x: string) => string;
};

export default function useContextMenu({
  menuItems,
  t = (x) => x,
}: Props): [React.ReactNode, { show: (ev: MouseEvent) => void }] {
  const [menuPosition, setMenuPosition] = useState({ left: 0, top: 0 });
  const [menuVisibility, setMenuVisibility] = useState(false);

  const showMenu = (ev: MouseEvent) => {
    setMenuVisibility(true);
    setMenuPosition({
      left: ev.clientX,
      top: ev.clientY,
    });
  };

  return [
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
    </Popover>,
    { show: showMenu },
  ];
}
