import React from "react";
import {
  Container,
  Box,
  Paper,
  Table,
  TableBody,
  TableRow,
  TableCell,
  Typography,
  Switch,
  FormControlLabel,
} from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { useTranslation } from "react-i18next";

import { getUsers, addBlackList, removeBlackList } from "@rpc/http";
import { useRequest } from "ahooks";

const useStyles = makeStyles((theme) => ({
  root: {
    paddingLeft: "3rem",
    paddingRight: "-1rem",
    paddingTop: "0.5rem",
    paddingBottom: "0.5rem",
  },
}));

const style = {
  container: {
    marginTop: "1rem",
  },
};

export default function UserManager() {
  const classes = useStyles();
  const { t } = useTranslation();

  const { data } = useRequest(getUsers, {
    pollingInterval: 1000,
    loadingDelay: 1000,
  });

  const onHandleChange = (enable: boolean, ip: string) => {
    if (enable) {
      removeBlackList(ip);
    } else {
      addBlackList(ip);
    }
  };

  return (
    <Container maxWidth="md" style={style.container}>
      <Box mt="1rem" />
      <Paper className={classes.root} elevation={3}>
        <Table size="small">
          <TableBody>
            <TableRow>
              <TableCell>
                <Typography variant="h6">{t("ClientIP")}</Typography>
              </TableCell>
              <TableCell>
                <Typography variant="h6">{t("LastConnectTime")}</Typography>
              </TableCell>
              <TableCell>
                <Typography variant="h6">{t("RequestCount")}</Typography>
              </TableCell>
              <TableCell>
                <Typography variant="h6">{t("Action")}</Typography>
              </TableCell>
            </TableRow>
            {data &&
              Object.entries(data.users).map((kv) => {
                const [ip, status] = kv;
                const connectTime = new Date();
                connectTime.setTime(status.time * 1000);
                return (
                  <TableRow>
                    <TableCell>
                      <Typography variant="subtitle1">
                        {data.client_ip == ip ? t("currentDevice") : ip.substr(7)}
                      </Typography>
                    </TableCell>
                    <TableCell>
                      <Typography variant="subtitle1">{formatDate(connectTime)}</Typography>
                    </TableCell>
                    <TableCell>
                      <Typography variant="subtitle1">{status.requests}</Typography>
                    </TableCell>
                    <TableCell>
                      <FormControlLabel
                        control={
                          <Switch
                            onChange={() => {
                              onHandleChange(status.disable, ip);
                            }}
                            checked={!status.disable}
                          />
                        }
                        label={status.disable ? t("Deny") : t("Allow")}
                        labelPlacement="end"
                      />
                    </TableCell>
                  </TableRow>
                );
              })}
          </TableBody>
        </Table>
      </Paper>
    </Container>
  );
}

function formatDate(date: Date) {
  const mm = date.getMonth() + 1; // getMonth() is zero-based
  const dd = date.getDate();
  const h = date.getHours();
  const m = date.getMinutes();
  const s = date.getSeconds();

  return (
    [date.getFullYear(), (mm > 9 ? "" : "0") + mm, (dd > 9 ? "" : "0") + dd].join("-") +
    " " +
    [(h > 9 ? "" : "0") + h, (m > 9 ? "" : "0") + m, (s > 9 ? "" : "0") + s].join(":")
  );
}
