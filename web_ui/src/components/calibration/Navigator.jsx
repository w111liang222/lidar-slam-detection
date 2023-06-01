import React, { useEffect, useState } from "react";
import LidarCalibration from "./lidar";
import LidarCameraCalibration from "./lidar-camera";
import CameraCalibrator from "./camera";
import InsCalibrator from "./lidar-ins";
import ImuCalibrator from "./lidar-imu";
import RadarLidarCalicration from "./lidar-radar";
import PanoramaCameraCalibrator from "./panorama-camera";
import { NavLink, Routes, Route, useMatch, useLocation } from "react-router-dom";
import { Button, Container, Grid } from "@mui/material";
import { styled } from "@mui/material/styles";
import { useTranslation } from "react-i18next";
import "./index.css";
import { getAvfunsConfig } from "@components/store/avfunSlice";
import WEB_STORE from "@rpc/sample/webstore.json";
import { useSelector, useDispatch } from "react-redux";

export const ROUTE = {
  calibrate_lidar: {
    name: "calibrate_lidar",
    component: LidarCalibration,
  },
  calibrate_lidar_camera: {
    name: "calibrate_lidar_camera",
    component: LidarCameraCalibration,
  },
  calibrate_camera: {
    name: "calibrate_camera",
    component: CameraCalibrator,
  },
  calibrate_lidar_ins: {
    name: "calibrate_lidar_ins",
    component: InsCalibrator,
  },
  calibrate_lidar_radar: {
    name: "calibrate_lidar_radar",
    component: RadarLidarCalicration,
  },
  calibrate_lidar_imu: {
    name: "calibrate_lidar_imu",
    component: ImuCalibrator,
  },
  // calibrate_panorama_camera: {
  //   name: "calibrate_panorama_camera",
  //   component: PanoramaCameraCalibrator,
  // },
};

const NavButton = styled(Button)({
  height: "10rem",
  fontSize: "1.5rem",
  textTransform: "none",
  color: "black",
  backgroundColor: "#E0E0E0",
  "&:hover": {
    backgroundColor: "#D5D5D5",
  },
});

const LinkDiv = styled("div")({
  maxWidth: "1200px",
  marginTop: "1rem",
  position: "absolute",
  left: 0,
  right: 0,
  marginLeft: "auto",
  marginRight: "auto",
});

export default function CalibNavigator() {
  const { t } = useTranslation();
  let url = useMatch("/:item");
  let location = useLocation();

  const avfuns = useSelector((state) => state.avfuns);
  const dispatch = useDispatch();
  const [show, setShow] = useState(WEB_STORE.avfuns.calibration);
  useEffect(() => {
    const action = getAvfunsConfig();
    dispatch(action);
  }, []);

  useEffect(() => {
    if (typeof avfuns == "object") {
      setShow(avfuns.calibration);
    }
  }, [avfuns]);

  if (url?.pathname === "/") url.pathname = "";
  return (
    <>
      <LinkDiv>
        {location.pathname === url?.pathname && (
          <Grid container spacing={2}>
            {Object.entries(ROUTE).map(([linkName, tab]) => {
              if (show[linkName]) {
                return (
                  <Grid item xs={12} sm={6} key={linkName}>
                    <NavLink to={linkName} style={{ textDecoration: "none" }}>
                      <NavButton fullWidth variant="contained">
                        {t(tab.name)}
                      </NavButton>
                    </NavLink>
                  </Grid>
                );
              }
            })}
          </Grid>
        )}
      </LinkDiv>
      <Routes>
        {Object.entries(ROUTE).map(([linkName, tab]) => {
          if (!tab.component) return;
          const Component = tab.component;
          return <Route path={linkName} key={linkName} element={<Component />} />;
        })}
      </Routes>
    </>
  );
}
