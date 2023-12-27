import React, { memo, useEffect, useMemo, useState } from "react";
import {
  Paper,
  Select,
  MenuItem,
  Stepper,
  Step,
  Button,
  StepButton,
  Box,
  CircularProgress,
  Typography,
} from "@mui/material";

import makeStyles from "@mui/styles/makeStyles";
import produce from "immer";

export enum STEP {
  BEGIN = 0,
  CALIB_GROUND,
  CALIB_POSITION,
  FINETUNE,
  RECALIB,
}

const STEP_NAME = ["stepNameSelectLidar", "stepNameCalibGroud", "stepNameCalibFront", "stepNameFinetune"];

const HELP_MESSAGE = ["helpMsgSelectLidar", "helpMsgCalibGroud", "helpMsgCalibFront", "helpMsgFinetune"];

const useStyles = makeStyles((theme) => ({
  container: {
    paddingBottom: "0.5rem",
    padding: "2rem",
    marginTop: "-1rem",
    marginLeft: "1rem",
    marginRight: "1rem",
    borderRadius: "0.5rem",
  },
}));

export interface Props {
  activeStep: STEP;
  setActiveStep: any;
  onNext?: any;
  children?: any;
  t?: (x: string | undefined) => string;
  calibEnable?: boolean;
}

function CalibStepper({ activeStep, setActiveStep, children, onNext, t = (x) => x || "", calibEnable }: Props) {
  const [completed, setCompleted] = useState<{ [key: number]: boolean }>({});
  const [processing, setProcessing] = useState(false);
  const handleStep = (step: STEP) => {
    setCompleted(
      produce((x) => {
        for (let index = 0; index < STEP_NAME.length; index++) {
          x[index] = index < step;
        }
      })
    );
    setActiveStep(step);
  };
  const handleNext = async () => {
    setProcessing(true);
    if (onNext) await onNext();
    handleStep(activeStep + 1);
    setProcessing(false);
  };
  const handleSkip = () => {
    handleStep(activeStep + 1);
  };
  const handleReset = () => {
    handleStep(0);
  };

  const handlePrevious = () => {
    handleStep(activeStep - 1);
  };

  useEffect(() => {
    handleStep(activeStep);
  }, [activeStep]);

  const getExtraProps = (index: number) => {
    if (activeStep === index && processing)
      return {
        icon: <CircularProgress size="1.5rem" />,
      };
    return {};
  };

  const classes = useStyles();
  return (
    <Paper className={classes.container}>
      <Stepper nonLinear activeStep={activeStep}>
        {STEP_NAME.map((label, index) => {
          return (
            <Step key={label} completed={completed[index]}>
              <StepButton onClick={() => handleStep(index)} {...getExtraProps(index)}>
                {t(label)}
              </StepButton>
            </Step>
          );
        })}
      </Stepper>

      <Box px="2rem" display="flex" alignItems="center" style={{ whiteSpace: "pre-wrap", marginTop: "1rem" }}>
        <Typography variant="body1">{t(HELP_MESSAGE[activeStep])}</Typography>
        <span> </span>
        {children}
      </Box>

      <Box px="1rem" display="flex">
        {activeStep === 0 ? undefined : (
          <Button color="primary" onClick={handlePrevious}>
            {t("previous")}
          </Button>
        )}
        <Box flexGrow={1} />
        <Box>
          {activeStep >= STEP_NAME.length || (
            <Button color="primary" onClick={handleSkip}>
              {t("skip")}
            </Button>
          )}
          <Button
            variant="contained"
            color="primary"
            disabled={!calibEnable}
            onClick={activeStep < STEP_NAME.length ? handleNext : handleReset}
            onKeyDown={(event) => {
              event.preventDefault();
            }}
            onKeyUp={(event) => {
              event.preventDefault();
            }}>
            {activeStep == STEP_NAME.length - 1
              ? t("calibFinish")
              : activeStep < STEP_NAME.length
              ? t("next")
              : t("resetCalibration")}
          </Button>
        </Box>
      </Box>
    </Paper>
  );
}

export default memo(CalibStepper);
