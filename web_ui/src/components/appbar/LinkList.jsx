import React from "react";
import { Button } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { NavLink } from "react-router-dom";
import { useTranslation } from "react-i18next";

const useStyles = makeStyles((theme) => ({
  buttonSelected: {
    "& div": {
      borderBottom: "1px solid",
    },
  },
  link: {
    // color: theme.palette.light,
    color: "white",
    textDecoration: "none",
  },
}));

export default function LinkList({ links = [] }) {
  const { t } = useTranslation();

  const classes = useStyles();
  return (
    <div>
      {links &&
        links.map((tabName) => (
          <NavLink to={`/${tabName}`} activeClassName={classes.buttonSelected} className={classes.link} key={tabName}>
            <Button key={tabName} color="inherit">
              <div>{t(tabName)}</div>
            </Button>
          </NavLink>
        ))}
    </div>
  );
}
