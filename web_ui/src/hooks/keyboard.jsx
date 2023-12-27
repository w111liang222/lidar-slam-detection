import { useEffect, useRef, useState } from "react";

const CONTROL_KEY = navigator.platform === "MacIntel" ? "Meta" : "Control";
const SHIFT_KEY = "Shift";

export function useCtrlKey() {
  const [pressed, setPressed] = useState(false);

  useEffect(() => {
    const set = (event) => {
      if (event.key === CONTROL_KEY) {
        setPressed(true);
      }
    };
    const unset = (event) => {
      if (event.key === CONTROL_KEY) {
        setPressed(false);
      }
    };
    window.addEventListener("keydown", set);
    window.addEventListener("keyup", unset);
    return () => {
      window.removeEventListener("keydown", set);
      window.removeEventListener("keyup", unset);
    };
  }, []);
  return pressed;
}

export function useShiftKey() {
  const [pressed, setPressed] = useState(false);

  useEffect(() => {
    const set = (event) => {
      if (event.key === SHIFT_KEY) {
        setPressed(true);
      }
    };
    const unset = (event) => {
      if (event.key === SHIFT_KEY) {
        setPressed(false);
      }
    };
    window.addEventListener("keydown", set);
    window.addEventListener("keyup", unset);
    return () => {
      window.removeEventListener("keydown", set);
      window.removeEventListener("keyup", unset);
    };
  }, []);
  return pressed;
}

export function useLeftKey() {
  const [pressed, setPressed] = useState(false);

  useEffect(() => {
    const set = (event) => {
      if (event.key === "ArrowLeft" || event.key === "a") {
        setPressed(true);
      }
    };
    const unset = (event) => {
      if (event.key === "ArrowLeft" || event.key === "a") {
        setPressed(false);
      }
    };
    window.addEventListener("keydown", set);
    window.addEventListener("keyup", unset);
    return () => {
      window.removeEventListener("keydown", set);
      window.removeEventListener("keyup", unset);
    };
  }, []);
  return pressed;
}

export function useRightKey() {
  const [pressed, setPressed] = useState(false);

  useEffect(() => {
    const set = (event) => {
      if (event.key === "ArrowRight" || event.key === "d") {
        setPressed(true);
      }
    };
    const unset = (event) => {
      if (event.key === "ArrowRight" || event.key === "d") {
        setPressed(false);
      }
    };
    window.addEventListener("keydown", set);
    window.addEventListener("keyup", unset);
    return () => {
      window.removeEventListener("keydown", set);
      window.removeEventListener("keyup", unset);
    };
  }, []);
  return pressed;
}
