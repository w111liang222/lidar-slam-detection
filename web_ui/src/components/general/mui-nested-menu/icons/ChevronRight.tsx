import React from "react";
import SvgIcon, { SvgIconProps } from "@mui/material/SvgIcon";

export const ChevronRight = (props: SvgIconProps) => {
  return (
    <SvgIcon {...props}>
      <path d="M9.29 6.71c-.39.39-.39 1.02 0 1.41L13.17 12l-3.88 3.88c-.39.39-.39 1.02 0 1.41.39.39 1.02.39 1.41 0l4.59-4.59c.39-.39.39-1.02 0-1.41L10.7 6.7c-.38-.38-1.02-.38-1.41.01z"></path>
    </SvgIcon>
  );
};
