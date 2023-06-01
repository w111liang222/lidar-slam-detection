import { getVertexData } from "@rpc/http";
import React, { useEffect, useState } from "react";
import { ImageList } from "./ImageList";

type Props = {
  id: string;
  config?: any;
  isSelect?: boolean;
};

export default function ImageVertexView({ id, config, isSelect }: Props) {
  const [vetexId, setVetexId] = useState<string>();
  const [images, setImages] = useState<LSD.MapKeyframe["images"]>({});

  useEffect(() => {
    if (id != vetexId && isSelect) {
      getVertexData(id, "i").then((kf) => {
        setImages(kf.images);
      });
      setVetexId(id);
    }
  }, [id]);

  useEffect(() => {
    if (isSelect && (images == undefined || id != vetexId)) {
      getVertexData(id, "i").then((kf) => {
        setImages(kf.images);
      });
      setVetexId(id);
    }
  }, [isSelect]);

  return <>{isSelect && images && <ImageList imageData={images} />}</>;
}
