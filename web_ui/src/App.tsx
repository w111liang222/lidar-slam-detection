import React, { useState, useEffect, useRef } from "react";
import {
  Route,
  useNavigate,
  useMatch,
  Routes,
  // useLocation
} from "react-router-dom";

import ThemeProvider from "@components/ThemeProvider";
import AppBar from "@components/appbar";
import Home from "@components/home";
import Config from "@components/config";
import Preview from "@components/preview";
import CalibrationNavigator, { ROUTE as CALIB_ROUTE } from "@components/calibration/Navigator";
import DevConfig from "@components/dev";
import Log from "@components/dev/Log";
import FileUploader from "@components/upgrade";
import MapEditor from "@components/editor";
import AskMoney from "@components/general/AskMoney";
import UserManager from "@components/general/UserManager";
import TViz from "@components/tviz";
import log from "loglevel";
import { useLocalStorageState, useRequest, useSessionStorageState } from "ahooks";
import _ from "lodash";
import { isPermit } from "@rpc/http";

const USER_TABS = {
  config: Config,
  preview: Preview,
  calibration: CalibrationNavigator,
  tviz: TViz,
};
const UPGRADE_TABS = {
  // upgrade: FileUploader,
};
const DEVELOPER_TABS = {
  configDev: DevConfig,
  // log: Log,
};

export default function App() {
  const [isDeveloperMode, setIsDeveloperMode] = useSessionStorageState("devMode", false);
  const [isDarkMode, setIsDarkMode] = useLocalStorageState("theme", false);
  const [menuBars, setMenuBar] = useState(<div />);
  const [error, setError] = useState<Error>();
  const editorRef = useRef<any>();
  const navigate = useNavigate();
  let url = useMatch("/:item");
  const { data, run, cancel } = useRequest(isPermit, {
    pollingInterval: 10000,
    loadingDelay: 1000,
    onSuccess: (data) => {
      if (data.licence == "registered") {
        cancel();
      } else if (data.licence == "error" && url?.pathname != "/licence") {
        navigate("/licence", { replace: true });
      }
    },
  });

  const userLinks =
    url?.pathname == "/editor"
      ? []
      : error
      ? ["home", "upgrade"]
      : _.concat("home", Object.keys(USER_TABS), Object.keys(UPGRADE_TABS));

  const developerLinks = url?.pathname == "/editor" ? [] : Object.keys(DEVELOPER_TABS);

  useEffect(() => {
    if (isDeveloperMode) log.enableAll();
    // change in render function won't do, why?
    else log.setLevel(log.levels.WARN); // default level
  }, [isDeveloperMode]);

  useEffect(() => {
    if (editorRef.current && url?.pathname == "/editor") {
      setMenuBar(editorRef.current.menuBar());
    } else {
      setMenuBar(<div />);
    }
  }, [editorRef, url]);

  let mapEditor = React.useMemo(() => <MapEditor ref={editorRef} />, [editorRef]);

  return (
    <ThemeProvider isDarkMode={isDarkMode}>
      <AppBar
        {...{
          isDeveloperMode,
          setIsDeveloperMode,
          isDarkMode,
          setIsDarkMode,
          userLinks: userLinks,
          menuBars: menuBars,
          developerLinks: developerLinks,
        }}
      />
      <Routes>
        <Route path="/">
          <Route index element={<Home onSuccess={() => setError(undefined)} />} />
          <Route path="home" element={<Home onSuccess={() => setError(undefined)} />} />
          <Route path="licence" element={<AskMoney />} />
          <Route path="users" element={<UserManager />} />
          <Route path="editor" element={mapEditor} />
          {Object.entries({
            ...USER_TABS,
            ...UPGRADE_TABS,
            ...DEVELOPER_TABS,
          }).map((kv) => {
            const [name, EL] = kv;
            return <Route key={name} path={`/${name}/*`} element={<EL />} />;
          })}
        </Route>
      </Routes>
    </ThemeProvider>
  );
}
