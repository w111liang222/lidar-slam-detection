import {
  Grid,
  FormControlLabel,
  Switch,
  Typography,
  Container,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Box,
  Button,
} from "@mui/material";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { setAvfuns } from "@rpc/http";
import React, { useState, useEffect } from "react";
import { useTranslation } from "react-i18next";
import { useFormik } from "formik";

import WEB_STORE from "@rpc/sample/webstore.json";
import { getAvfunsConfig } from "@components/store/avfunSlice";
import { useSnackbar } from "@hooks/index";
import { useSelector, useDispatch } from "react-redux";

function SelecteItem({ values, handleChange, t = (x) => x }) {
  return Object.entries(values).map(([funName, funItem], funIndex) => {
    return (
      <Container maxWidth="md">
        <Accordion style={{ marginTop: "1px" }} key={funIndex}>
          <AccordionSummary expandIcon={<ExpandMoreIcon />} id="selectConfig">
            <Typography variant="subtitle2" className="subtitle-no-mt">
              {t(funName)}
            </Typography>
          </AccordionSummary>
          <AccordionDetails style={{ paddingTop: "0px" }}>
            {typeof Object.values(funItem)[0] !== "boolean" ? (
              <SelecteItem values={funItem} handleChange={handleChange} t={t} />
            ) : (
              <Container maxWidth="md" style={{ marginLeft: "0rem" }}>
                <Grid container spacing={0}>
                  {Object.entries(funItem).map(([configItem, tag], index) => {
                    return (
                      <Grid item xs="auto" key={index}>
                        <FormControlLabel
                          control={
                            <Switch
                              checked={values[funName][configItem]}
                              name={funName + "." + configItem}
                              onChange={handleChange}
                            />
                          }
                          label={t(configItem)}
                        />
                      </Grid>
                    );
                  })}
                </Grid>
              </Container>
            )}
          </AccordionDetails>
        </Accordion>
      </Container>
    );
  });
}

export default function SelecteFunctions({ setIsLoaded }) {
  const avfuns = useSelector((state) => state.avfuns);
  const dispatch = useDispatch();

  useEffect(() => {
    setIsLoaded(false);
    dispatch(getAvfunsConfig());
    setIsLoaded(true);
  }, []);

  useEffect(() => {
    if (typeof avfuns == "object") {
      setValues(avfuns);
    }
  }, [avfuns]);

  const { t } = useTranslation();
  const formik = useFormik({
    initialValues: WEB_STORE.avfuns,
    onSubmit: (values) => {
      console.log(values);
    },
  });

  const { values, handleChange, errors, touched, setValues } = formik;

  const [snackbar, showMessage] = useSnackbar();
  const handleUpload = async () => {
    try {
      setIsLoaded(false);
      const configUpdateResponse = await setAvfuns(values);
      setIsLoaded(true);
      console.log(configUpdateResponse);
      showMessage(t("configUpdated"), 2000);
    } catch (e) {
      console.log(e);
    }
  };

  return (
    <div>
      <Container maxWidth="md">
        <SelecteItem values={values} handleChange={handleChange} t={t} />
        <Box display="flex">
          <Box flexGrow={1} />
          {/* <Button onClick={handleDownload} color="primary" variant="contained">
                                {t("reset")}
                            </Button> */}
          <Button onClick={handleUpload} color="primary" variant="contained">
            {t("update")}
          </Button>
        </Box>
        {snackbar}
      </Container>
    </div>
  );
}
