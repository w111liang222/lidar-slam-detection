import React, { useEffect, useState } from "react";
import { Container, Box } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { useTranslation } from "react-i18next";

import Status from "./Status";
import Disk from "./Disk";
import { getStatus } from "@rpc/http";
import Loading from "@components/general/Loading";
import { useRequest } from "ahooks";

const style = {
  container: {
    marginTop: "1rem",
  },
};

export default function Home({
  onNetworkError,
  onSuccess,
}: {
  onNetworkError?: (err: Error) => void;
  onSuccess?: () => void;
}) {
  const { t } = useTranslation();

  const [isStatusNormal, setIsStatusNormal] = useState<boolean>(false);

  const { data, loading, error } = useRequest(getStatus, {
    pollingInterval: 1000,
    loadingDelay: 1000,
    onSuccess,
  });
  if (error) onNetworkError && onNetworkError(error);
  const boardStatus = {
    hostTime: formatDate(new Date()),
    deviceTime: data?.time,
    status: t(data?.status || error?.message || ""),
  };
  const diskStatus = {
    name: data?.disk.disk_name,
    total: data?.disk.total,
    usage: data?.disk.used_percent,
    frame_success: data?.disk.frame_success,
    frame_lost: data?.disk.frame_lost,
    frame_error: data?.disk.frame_error,
  };

  useEffect(() => {
    if (data && (data.status == "Paused" || data.status == "Running")) {
      setIsStatusNormal(true);
    } else {
      setIsStatusNormal(false);
    }
  }, [data]);

  return (
    <Container maxWidth="sm" style={style.container}>
      <Status {...boardStatus} t={t} normal={isStatusNormal} />
      <Box mt="1rem" />
      {data?.disk.has_disk && <Disk {...diskStatus} isRecording={data.disk.status} t={t} />}
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
