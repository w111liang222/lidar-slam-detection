import React from "react";
import { Box, Fab } from "@mui/material";

export default function AimButton({ onClick, disable, children }) {
  return (
    <Box position="fixed" bottom="2rem" right="1rem">
      <Fab color={"primary"} size={"small"} onClick={onClick} disabled={disable}>
        {children}
      </Fab>
    </Box>
  );
}
