import React, { useEffect, useState, useRef } from "react";
import Draggable from "react-draggable"; // The default
import { usePopover } from "@hooks/index";
import "./ImageViewer.css";
import { useCounter } from "ahooks";

export function ImageList({ imageData, onLoad }: any) {
  const numCamera = Object.keys(imageData).length;
  var appBarEle = document.getElementsByClassName("MuiToolbar-root")[0] as HTMLElement;
  const offsetHeight = appBarEle ? appBarEle.offsetHeight : 0;
  const height = Math.max(100, Math.min(280, (window.innerHeight - offsetHeight) / numCamera));
  // https://stackoverflow.com/questions/5467129/sort-javascript-object-by-key
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

export default function ImageViewer({ name = "", imageData, onLoad, numCamera = 1 }: Props) {
  const imgRef = useRef<HTMLImageElement>(null);
  const ref = useRef({ lock: false });
  const [popover, showPopover, hidePopover] = usePopover();
  // const [cnt, { inc }] = useCounter(0);
  useEffect(() => {
    if (!imageData) return;

    if (ref.current.lock) return;

    if (!imgRef.current) return;

    ref.current.lock = true;

    var appBarEle = document.getElementsByClassName("MuiToolbar-root")[0] as HTMLElement;
    const offsetHeight = appBarEle ? appBarEle.offsetHeight : 0;

    const height = Math.max(100, Math.min(280, (window.innerHeight - offsetHeight) / numCamera));
    const width = (imgRef.current.naturalWidth / imgRef.current.naturalHeight) * height;

    setImgWidth(width);
    setImgHeight(height);

    const blob = new Blob([imageData.buffer], {
      type: "application/octet-stream",
    });
    const url = URL.createObjectURL(blob); //possibly `webkitURL` or another vendor prefix for old browsers.
    imgRef.current.src = url;
    imgRef.current.onload = () => {
      URL.revokeObjectURL(url);
      ref.current.lock = false;
      onLoad && onLoad();
      // console.log(cnt);
      // inc();
    };
  }, [imageData]);

  const [imgZoom, setImgZoom] = useState(false);
  const [imgScale, setImgScale] = useState(1.0);
  const [imgWidth, setImgWidth] = useState(0);
  const [imgHeight, setImgHeight] = useState(0);
  const [ZIndex, setZIndex] = useState(false);

  return (
    <>
      {popover}
      {
        <Draggable
        // disabled={!draggable}
        // position={position}
        // onDrag={handleDrag}
        // grid={[50, 50]}
        >
          {/* https://github.com/react-grid-layout/react-draggable/issues/69 */}

          <img
            draggable="false"
            style={{
              width: (imgZoom ? 2 : 1) * imgWidth * imgScale,
              height: (imgZoom ? 2 : 1) * imgHeight * imgScale,
              position: "fixed",
            }}
            className={ZIndex ? "maxIndex" : undefined}
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
            // onMouseDownCapture={hidePopover}
          />
        </Draggable>
      }
    </>
  );
}
