import SelectFunctions from "./SelectFunctions";
import BoardConfig from "./BoardConfig";
import React, { useState } from "react";
import { Box, Backdrop, CircularProgress, Grid, FormControl, InputLabel, Select, MenuItem } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { useTranslation } from "react-i18next";

const useStyles = makeStyles((theme) => ({
  backdrop: {
    zIndex: theme.zIndex.appBar - 1,
    color: "#fff",
  },
}));

export default function DevConfig() {
  const [isLoaded, setIsLoaded] = useState<boolean>(true);
  const classes = useStyles();
  const { t } = useTranslation();
  const [devConfig, setDev] = useState("BoardConfig" || "SelectFunctions");

  return (
    <>
      <Box>
        {isLoaded || (
          <Backdrop open className={classes.backdrop}>
            <CircularProgress color="inherit" />
          </Backdrop>
        )}
        <div style={{ display: "flex" }}>
          <div style={{ width: "15%", marginRight: 0 }}>
            <Grid style={{ marginTop: "1rem", marginLeft: "1rem" }} container>
              <Grid item xs={12} zeroMinWidth>
                <FormControl>
                  <InputLabel>{t("Mode")}</InputLabel>
                  <Select
                    label={t("mode")}
                    name={`mode`}
                    value={devConfig}
                    onChange={(e) => {
                      setDev(e.target.value as string);
                    }}>
                    <MenuItem value={"BoardConfig"}>{t("BoardConfig")}</MenuItem>
                    <MenuItem value={"SelectFunctions"}>{t("SelectFunctions")}</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
            </Grid>
          </div>
          <div style={{ width: "70%", marginTop: "1rem" }}>
            <div hidden={devConfig != "BoardConfig"}>
              <BoardConfig setIsLoaded={setIsLoaded} />
            </div>
            <div hidden={devConfig != "SelectFunctions"}>
              <SelectFunctions setIsLoaded={setIsLoaded} />
            </div>
          </div>
        </div>
      </Box>
    </>
  );
}
