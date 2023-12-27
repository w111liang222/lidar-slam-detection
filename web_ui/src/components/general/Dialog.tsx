import React, { useState } from "react";
import { Button, Dialog, DialogActions, DialogContent } from "@mui/material";
import { useTranslation } from "react-i18next";
import { useImperativeHandle } from "react";

type Props = {
  content: string;
  onConfirm?: () => void;
  onCancel?: () => void;
};

function MyDialog({ content, onConfirm, onCancel }: Props, ref: any) {
  useImperativeHandle(ref, () => ({
    open: () => setOpen(true),
  }));
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);
  return (
    <Dialog open={open}>
      <DialogContent style={{ textAlign: "center" }}>
        <b>{content}</b>
      </DialogContent>
      <DialogActions>
        <Button
          onClick={() => {
            setOpen(false);
            onCancel && onCancel();
          }}>
          {t("no")}
        </Button>
        <Button
          onClick={() => {
            setOpen(false);
            onConfirm && onConfirm();
          }}>
          {t("yes")}
        </Button>
      </DialogActions>
    </Dialog>
  );
}

export default React.forwardRef(MyDialog);
