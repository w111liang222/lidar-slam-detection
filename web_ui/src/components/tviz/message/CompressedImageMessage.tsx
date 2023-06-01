import { useRequest } from "ahooks";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { getMessageData } from "@rpc/http";
import ImageViewer from "@components/preview/ImageViewer";

export const CompressedImageDataProps = [""];

export type Props = {
  name: string;
  index: number;
};

function CompressedImageMessage({ name, index }: Props) {
  const { t } = useTranslation();
  const [image, setImage] = useState<Uint8Array | undefined>(undefined);

  useRequest(() => getMessageData(name, "CompressedImage"), {
    pollingInterval: 50,
    onSuccess: (data) => {
      if (data != undefined && data !== "") {
        setImage(data as Uint8Array);
      }
    },
  });

  return (
    <>
      {image && (
        <div style={{ position: "absolute", right: "48%", top: 60 + 220 * index }}>
          <ImageViewer name={name} imageData={image} numCamera={4} />
        </div>
      )}
    </>
  );
}
export default React.memo(CompressedImageMessage);
