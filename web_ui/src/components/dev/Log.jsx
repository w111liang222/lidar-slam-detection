import { getLogFiles, getLogContent } from "@rpc/http-upgrade";

import React, { useState, useEffect } from "react";
import { Container, Button, Select, MenuItem } from "@mui/material";
import { HOSTNAME, PORT } from "@rpc/http-upgrade";

export default function Log() {
  const [logFiles, setLogFiles] = useState([]);
  const [currentLogFileIndex, setCurrentLogFileIndex] = useState("");
  const [logContent, setLogContent] = useState("");

  useEffect(() => {
    async function init() {
      const logFiles = await getLogFiles();
      logFiles.sort().reverse();
      setLogFiles(logFiles);
    }
    init();
  }, []);

  const handleChange = async (i) => {
    setCurrentLogFileIndex(i);
    setLogContent(await getLogContent(logFiles[i]));
  };

  const downloadLog = () => {
    window.open(`http://${HOSTNAME}:${PORT}/v1/log-download`);
  };

  return (
    <Container maxWidth="lg">
      <Select variant="standard" value={currentLogFileIndex} onChange={(e) => handleChange(e.target.value)}>
        {logFiles.slice(0, 10).map((fn, i) => {
          return (
            <MenuItem key={i} value={i}>
              {fn}
            </MenuItem>
          );
        })}
      </Select>
      <Button variant="contained" onClick={downloadLog}>
        download
      </Button>
      <div style={{ maxWidth: "1200px", whiteSpace: "pre-wrap" }}>{logContent}</div>
    </Container>
  );
}
