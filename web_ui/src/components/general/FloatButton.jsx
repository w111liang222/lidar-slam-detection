import React from "react";
import { Box, Fab } from "@mui/material";

export default function FloatButton({ onClick, children }) {
  return (
    <Box position="fixed" bottom="2rem" right="2rem">
      <Fab onClick={onClick}>{children}</Fab>
    </Box>
  );
}
