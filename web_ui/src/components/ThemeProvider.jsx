import React from "react";
import { ThemeProvider, StyledEngineProvider, createTheme } from "@mui/material/styles";
import { CssBaseline } from "@mui/material";
import { useEffect, useState } from "react";
const tpDarkStyle = `
:root {
    --tp-base-background-color: hsla(230, 7%, 20%, 1.00);
    --tp-base-shadow-color: hsla(0, 0%, 0%, 0.2);
    --tp-button-background-color: hsla(230, 7%, 70%, 1.00);
    --tp-button-background-color-active: hsla(230, 7%, 85%, 1.00);
    --tp-button-background-color-focus: hsla(230, 7%, 80%, 1.00);
    --tp-button-background-color-hover: hsla(230, 7%, 75%, 1.00);
    --tp-button-foreground-color: hsla(230, 7%, 20%, 1.00);
    --tp-container-background-color: hsla(230, 7%, 75%, 0.10);
    --tp-container-background-color-active: hsla(230, 7%, 75%, 0.25);
    --tp-container-background-color-focus: hsla(230, 7%, 75%, 0.20);
    --tp-container-background-color-hover: hsla(230, 7%, 75%, 0.15);
    --tp-container-foreground-color: hsla(230, 0%, 100%, 1.00);
    --tp-groove-foreground-color: hsla(230, 0%, 0%, 0.20);
    --tp-input-background-color: hsla(230, 7%, 0%, 0.20);
    --tp-input-background-color-active: hsla(230, 7%, 0%, 0.35);
    --tp-input-background-color-focus: hsla(230, 7%, 0%, 0.30);
    --tp-input-background-color-hover: hsla(230, 7%, 0%, 0.25);
    --tp-input-foreground-color: hsla(230, 7%, 75%, 1.00);
    --tp-label-foreground-color: hsla(230, 0%, 100%, 1.00);
    --tp-monitor-background-color: hsla(230, 7%, 0%, 0.20);
    --tp-monitor-foreground-color: hsla(230, 7%, 75%, 0.70);
  }
`;
const tpLightStyle = `
:root {
    --tp-base-background-color: hsla(230, 5%, 90%, 1.00);
    --tp-base-shadow-color: hsla(0, 0%, 0%, 0.10);
    --tp-button-background-color: hsla(230, 7%, 75%, 1.00);
    --tp-button-background-color-active: hsla(230, 7%, 60%, 1.00);
    --tp-button-background-color-focus: hsla(230, 7%, 65%, 1.00);
    --tp-button-background-color-hover: hsla(230, 7%, 70%, 1.00);
    --tp-button-foreground-color: hsla(230, 10%, 30%, 1.00);
    --tp-container-background-color: hsla(230, 15%, 30%, 0.20);
    --tp-container-background-color-active: hsla(230, 15%, 30%, 0.32);
    --tp-container-background-color-focus: hsla(230, 15%, 30%, 0.28);
    --tp-container-background-color-hover: hsla(230, 15%, 30%, 0.24);
    --tp-container-foreground-color: hsla(230, 10%, 30%, 1.00);
    --tp-groove-foreground-color: hsla(230, 15%, 30%, 0.10);
    --tp-input-background-color: hsla(230, 15%, 30%, 0.10);
    --tp-input-background-color-active: hsla(230, 15%, 30%, 0.22);
    --tp-input-background-color-focus: hsla(230, 15%, 30%, 0.18);
    --tp-input-background-color-hover: hsla(230, 15%, 30%, 0.14);
    --tp-input-foreground-color: hsla(230, 10%, 30%, 1.00);
    --tp-label-foreground-color: hsla(230, 10%, 30%, 0.70);
    --tp-monitor-background-color: hsla(230, 15%, 30%, 0.10);
    --tp-monitor-foreground-color: hsla(230, 10%, 30%, 0.50);
  }
`;

const defaultTheme = {
  components: {
    MuiTextField: {
      defaultProps: {
        variant: "outlined",
        fullWidth: true,
      },
    },
    MuiGrid: {
      defaultProps: {
        spacing: 1,
        alignItems: "center",
      },
      styleOverrides: {
        container: {
          marginTop: "0.5rem",
        },
      },
    },
    MuiSwitch: {
      defaultProps: {
        color: "primary",
      },
    },
    MuiFormControl: {
      defaultProps: {
        variant: "outlined",
        fullWidth: true,
      },
    },
  },
  palette: {
    mode: "light",
  },
  typography: {
    subtitle1: {
      fontSize: "1rem",
      fontWeight: "bold",
    },
    subtitle2: {
      fontSize: "0.8rem",
      fontWeight: "bold",
    },
    h6: {
      fontSize: "1.2rem",
      fontWeight: "bold",
    },
  },
};

export default function MyThemeProvider({ isDarkMode, children }) {
  const [theme, setTheme] = useState(defaultTheme);
  useEffect(() => {
    setTheme((theme) => ({
      ...theme,
      palette: { mode: isDarkMode ? "dark" : "light" },
    }));

    // https://stackoverflow.com/questions/28386125/dynamically-load-a-stylesheet-with-react
    var head = document.head;
    var style = document.createElement("style");
    style.textContent = isDarkMode ? tpDarkStyle : tpLightStyle;
    head.appendChild(style);
    return () => {
      head.removeChild(style);
    };
  }, [isDarkMode]);
  return (
    <StyledEngineProvider injectFirst>
      <ThemeProvider theme={createTheme(theme)}>
        <CssBaseline />
        {children}
      </ThemeProvider>
    </StyledEngineProvider>
  );
}
