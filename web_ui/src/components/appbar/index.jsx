import Logo from "./Logo";
import LinkList from "./LinkList";
import LocaleMenu from "./LocaleMenu";
import ThemeSwitch from "./ThemeSwitch";
import Power from "./Power";
import BackendSetter from "./BackendSetter";
import React, { useCallback, useEffect } from "react";
import { AppBar, Toolbar, Typography, Box } from "@mui/material";

export default function MyAppBar({
  isDeveloperMode = false,
  setIsDeveloperMode,
  isDarkMode = false,
  setIsDarkMode,
  userLinks = [],
  menuBars,
  developerLinks = [],
}) {
  const toggleDeveloperMode = useCallback(() => setIsDeveloperMode((x) => !x), []);

  return (
    <AppBar position="sticky">
      <Toolbar>
        <Logo {...{ toggle: toggleDeveloperMode }}>
          <Typography variant="h4">{"LSD"}</Typography>
        </Logo>
        <LinkList links={userLinks} />
        {menuBars}
        {isDeveloperMode && <LinkList links={developerLinks} />}
        <Box flexGrow={1} />
        {isDeveloperMode && <BackendSetter />}
        <Box mr="1rem">{isDeveloperMode && import.meta.env.VITE_COMMIT_HASH}</Box>
        <LocaleMenu />
        <ThemeSwitch
          value={isDarkMode ? "dark" : "light"}
          onChange={(val) => {
            val === "dark" ? setIsDarkMode(true) : setIsDarkMode(false);
          }}
        />
        {/* <Power /> */}
      </Toolbar>
    </AppBar>
  );
}
