import React, { forwardRef, RefObject } from "react";
import MenuItem, { MenuItemProps } from "@mui/material/MenuItem";
import Typography from "@mui/material/Typography";
import Box from "@mui/system/Box";
import styled from "@mui/material/styles/styled";
import { ListItemIcon, ListItemText } from "@mui/material";

const StyledMenuItem = styled(MenuItem)({
  paddingLeft: "4px",
  paddingRight: "4px",
  display: "flex",
  justifyContent: "space-between",
});

const StyledTypography = styled(Typography)({
  paddingLeft: "8px",
  paddingRight: "8px",
  textAlign: "left",
});

const FlexBox = styled(Box)({
  display: "flex",
});

interface IconMenuItemProps {
  leftIcon?: React.ReactNode;
  // rightIcon?: React.ReactNode;
  onClick?: () => void;
  label?: string;
  className?: string;
  MenuItemProps?: MenuItemProps;
  ref?: RefObject<HTMLLIElement>;
}

const IconMenuItem = forwardRef<HTMLLIElement, IconMenuItemProps>(
  ({ leftIcon, label, MenuItemProps, className, ...props }, ref) => {
    return (
      <MenuItem
        {...MenuItemProps}
        ref={ref}
        // className={className}
        {...props}>
        {/* <FlexBox> */}
        <ListItemIcon>{leftIcon}</ListItemIcon>
        <ListItemText>{label}</ListItemText>
        {/* </FlexBox> */}
        {/* {rightIcon} */}
      </MenuItem>
    );
  }
);

IconMenuItem.displayName = "IconMenuItem";
export { IconMenuItem };
