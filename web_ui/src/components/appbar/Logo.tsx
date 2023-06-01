import React, { useState, useEffect, useRef } from "react";
import { Button } from "@mui/material";

interface Props {
  toggle: () => void;
  children: React.ReactNode;
}

interface Ref {
  queue: number[];
}

export default function DeveloperModeToggler({ toggle, children }: Props) {
  const ref = useRef<Ref>({ queue: [] });

  const handleLogoClick = () => {
    const { queue } = ref.current;
    const t = performance.now();
    queue.push(t);
    if (queue[0] < t - 5000) queue.shift();
    if (queue.length >= 5) {
      ref.current.queue = [];
      toggle();
    }
  };

  return (
    <Button color="inherit" onClick={handleLogoClick}>
      {children}
    </Button>
  );
}
