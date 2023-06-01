import React, { useEffect, useState, useRef } from "react";
import Draggable from "react-draggable";
import { usePopover } from "@hooks/index";
import "./ImageList.css";

export function ImageList({ imageData, onLoad }: any) {
  const numCamera = Object.keys(imageData).length;
  const appBarEle = document.getElementsByClassName("MuiToolbar-root")[0] as HTMLElement;
  const offsetHeight = appBarEle ? appBarEle.offsetHeight : 0;
  const height = Math.max(100, Math.min(280, (window.innerHeight - offsetHeight) / numCamera));
  return (
    <div className="container">
      {Object.keys(imageData)
        .sort()
        .map((key, index) => {
          return (
            <div style={{ position: "absolute", left: 0, top: height * index }}>
              <ImageViewer key={key} name={key} imageData={imageData[key]} onLoad={onLoad} numCamera={numCamera} />
            </div>
          );
        })}
    </div>
  );
}

type Props = {
  name?: string;
  imageData?: Uint8Array;
  onLoad?: () => void;
  numCamera?: number;
};

function ImageViewer({ name = "", imageData, onLoad, numCamera = 1 }: Props) {
  const imgRef = useRef<HTMLImageElement>(null);
  const [url, setUrl] = useState<string | undefined>(undefined);
  const [popover, showPopover, hidePopover] = usePopover();

  useEffect(() => {
    if (imageData) {
      const blob = new Blob([imageData.buffer], {
        type: "application/octet-stream",
      });
      setUrl(URL.createObjectURL(blob));
    }
  }, [imageData]);

  useEffect(() => {
    if (imgRef.current && url) {
      imgRef.current.onload = () => {
        URL.revokeObjectURL(url);
        if (imgRef.current) {
          const appBarEle = document.getElementsByClassName("MuiToolbar-root")[0] as HTMLElement;
          const offsetHeight = appBarEle ? appBarEle.offsetHeight : 0;
          const height = Math.max(100, Math.min(280, (window.innerHeight - offsetHeight) / numCamera));
          const width = (imgRef.current.naturalWidth / imgRef.current.naturalHeight) * height;
          setImgWidth(width);
          setImgHeight(height);
        }
      };
    }
  }, [imgRef.current]);

  const [imgZoom, setImgZoom] = useState(false);
  const [imgScale, setImgScale] = useState(1.0);
  const [imgWidth, setImgWidth] = useState(0);
  const [imgHeight, setImgHeight] = useState(0);
  const [ZIndex, setZIndex] = useState(false);

  return (
    <>
      {popover}
      <Draggable>
        <img
          draggable="false"
          style={{
            width: (imgZoom ? 2 : 1) * imgWidth * imgScale,
            height: (imgZoom ? 2 : 1) * imgHeight * imgScale,
            position: "fixed",
          }}
          className={ZIndex ? "maxIndex" : undefined}
          src={url}
          ref={imgRef}
          onMouseMove={(ev) => {
            showPopover(ev as any as MouseEvent, name, {
              anchorOrigin: {
                vertical: "top",
                horizontal: "left",
              },
              transformOrigin: {
                vertical: "top",
                horizontal: "left",
              },
            });
            setZIndex(true);
          }}
          onWheel={(ev) => {
            setImgScale(Math.min(3.0, Math.max(0.5, imgScale + ev.deltaY * -0.001)));
          }}
          onMouseLeave={(ev) => {
            hidePopover();
            setZIndex(false);
          }}
          onDoubleClick={(ev) => {
            if (imgZoom || Math.abs(imgScale - 1.0) > 1e-4) {
              setImgScale(1.0);
              setImgZoom(false);
            } else {
              setImgZoom(!imgZoom);
            }
          }}
        />
      </Draggable>
    </>
  );
}
