import { Paper, Typography, Table, TableBody, TableCell, TableRow, Switch, FormControlLabel } from "@mui/material";
import { getConfig, postConfig } from "@rpc/http";
import makeStyles from "@mui/styles/makeStyles";
import React from "react";
import { useState, useEffect, useRef } from "react";
import { useRequest } from "ahooks";
import Loading from "@components/general/Loading";

const useStyles = makeStyles((theme) => ({
  root: {
    paddingLeft: "1rem",
    paddingRight: "1rem",
    paddingBottom: "0.5rem",
  },
}));

export type Props = {
  t: any;
  normal?: boolean;
};

export default function Status({ t, normal, ...props }: Props) {
  const classes = useStyles();
  const [isOnline, setIsOnline] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState<boolean>(false);

  const { data, run, cancel } = useRequest(getConfig, {
    pollingInterval: 1000,
    manual: true,
    onSuccess: (data: LSD.Config) => {
      cancel();
      if (data.input.mode == "online") {
        setIsOnline(true);
      } else {
        setIsOnline(false);
      }
      setIsLoading(false);
    },
  });
  const config = data;

  useEffect(() => {
    run();
  }, []);

  useEffect(() => {
    if (!normal) {
      run();
    }
  }, [normal]);

  const handleModeChange = () => {
    getConfig().then((cfg) => {
      let mode = cfg.input;
      if (isOnline) {
        mode.mode = "offline";
      } else {
        mode.mode = "online";
      }
      cfg.input = { ...mode };
      setIsLoading(true);
      postConfig(cfg).then(() => {
        run();
      });
    });
  };

  return (
    <>
      {isLoading && <Loading />}
      <Paper className={classes.root} elevation={3}>
        <Table size="small">
          <TableBody>
            {Object.entries(props).map((kv) => {
              const [key, val] = kv;
              return (
                <TableRow key={key}>
                  <TableCell>
                    <Typography variant="h6">{t(key)}</Typography>
                  </TableCell>
                  <TableCell align="right">
                    <Typography variant="body1">{val}</Typography>
                  </TableCell>
                </TableRow>
              );
            })}
            {normal && (
              <TableRow>
                <TableCell>
                  <Typography variant="h6">{t("mode")}</Typography>
                </TableCell>
                <TableCell align="right">
                  <FormControlLabel
                    control={<Switch onChange={handleModeChange} checked={isOnline} />}
                    label={isOnline ? t("online") : t("offline")}
                    labelPlacement="start"
                  />
                </TableCell>
              </TableRow>
            )}
          </TableBody>
        </Table>
      </Paper>
    </>
  );
}
