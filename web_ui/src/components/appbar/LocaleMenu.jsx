import React, { useState, useEffect } from "react";
import { Button, Box, Popover, MenuItem } from "@mui/material";
import { Translate, ExpandMore } from "@mui/icons-material";

import { useTranslation } from "react-i18next";
import { resources } from "@plugins/i18n.js";
import { useLocalStorageState } from "ahooks";

const localeSet = Object.keys(resources);

export default function LocaleSwitch() {
  const { t, i18n } = useTranslation();
  const [locale, setLocale] = useLocalStorageState("locale", localeSet[0]);
  useEffect(() => {
    i18n.changeLanguage(locale);
  }, [locale, i18n]);

  const [anchorEl, setAnchorEl] = useState(null);
  const handleButtonClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handItemClick = (key) => {
    setLocale(key);
    setAnchorEl(null);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <div>
      <Button color="inherit" onClick={handleButtonClick}>
        <Translate />
        <Box ml="0.5rem">{t(locale)}</Box>
        <ExpandMore />
      </Button>
      <Popover
        anchorEl={anchorEl}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "center",
        }}
        transformOrigin={{
          vertical: "top",
          horizontal: "center",
        }}
        open={Boolean(anchorEl)}
        onClose={handleClose}>
        {localeSet.map((locale) => {
          return (
            <MenuItem onClick={() => handItemClick(locale)} key={locale}>
              {t(locale)}
            </MenuItem>
          );
        })}
      </Popover>
    </div>
  );
}
