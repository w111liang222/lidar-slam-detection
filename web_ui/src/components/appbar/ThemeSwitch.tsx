import React from "react";
import { Button } from "@mui/material";
import { Brightness4, Brightness7 } from "@mui/icons-material";

export interface Props {
  value: "light" | "dark";
  onChange: (value: "light" | "dark") => void;
}

export default function ThemeSwitch({ value, onChange }: Props) {
  return (
    <Button onClick={() => onChange(value === "light" ? "dark" : "light")} color="inherit">
      {value === "light" ? <Brightness7 /> : <Brightness4 />}
    </Button>
  );
}
