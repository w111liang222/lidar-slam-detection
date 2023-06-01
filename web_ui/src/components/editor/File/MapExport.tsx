import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import IosShareIcon from "@mui/icons-material/IosShare";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { axios, setExportMapConfig } from "@rpc/http";
import Loading from "@components/general/Loading";

export interface Props {
  config: any;
  onFinish: any;
}

export default function MapExport({ config, onFinish }: Props) {
  const { t } = useTranslation();
  const [isLoading, setIsLoading] = useState<boolean>(false);

  const onClickMenu = () => {
    if (config) {
      setIsLoading(true);
      setExportMapConfig(config.map.zRange.min / 10.0, config.map.zRange.max / 10.0, config.map.color)
        .then(() => {
          setIsLoading(false);
          window.open(axios.defaults.baseURL + "v1/map-export-pcd");
        })
        .finally(() => {
          onFinish();
        });
    }
  };

  return (
    <>
      {isLoading && <Loading />}
      <MenuItem onClick={onClickMenu}>
        <ListItemIcon>
          <IosShareIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("MapExport")}</ListItemText>
      </MenuItem>
    </>
  );
}
