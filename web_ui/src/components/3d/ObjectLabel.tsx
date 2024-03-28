import car from "@assets/icons8-car-white.svg";
import ped from "@assets/icons8-ped.svg";
import cyc from "@assets/icons8-cyc-white.svg";
import cone from "@assets/icons8-cone.svg";

import React, { useRef, useEffect, useState } from "react";

import * as THREE from "three";
import * as proto from "@proto/detection";

const FONT_SIZE = 48;
const LINE_NUM = 2.5;
const ICON_SIZE = 64;
const LABEL_WIDTH = 1000;
const ASPECT = (FONT_SIZE * LINE_NUM + ICON_SIZE) / LABEL_WIDTH;

function drawBubble(
  ctx: CanvasRenderingContext2D,
  color: string,
  x: number,
  y: number,
  width: number,
  height: number,
  radius: number
) {
  ctx.beginPath();
  ctx.moveTo(x, y + radius);

  ctx.lineTo(x, y + height);

  ctx.lineTo(x + width - radius, y + height);
  ctx.arcTo(x + width, y + height, x + width, y + height - radius, radius);

  ctx.lineTo(x + width, y + radius);
  ctx.arcTo(x + width, y, x + width - radius, y, radius);

  ctx.lineTo(x + radius, y);
  ctx.arcTo(x, y, x, y + radius, radius);

  ctx.fillStyle = color;
  ctx.fill();
}

let icons: { color: string; src: string; canvas?: HTMLCanvasElement }[] = [
  { color: "#4E91F0", src: car },
  { color: "#FFC6B1", src: ped },
  { color: "#967FCB", src: cyc },
  { color: "#DDCC00", src: cone },
];
let numLoad = 0;
for (let icon of icons) {
  let img = new Image();
  img.src = icon.src;
  img.onload = () => {
    const canvas = document.createElement("canvas");
    let context = canvas.getContext("2d");

    let icon_size = ICON_SIZE;
    canvas.width = icon_size;
    canvas.height = icon_size;

    // draw icon
    drawBubble(context!, icon.color, 0, 0, icon_size, icon_size, icon_size / 3);
    let size = icon_size * 0.7;
    let offset = -size / 2 + icon_size / 2;
    context?.drawImage(img, offset, offset, size, size);

    icon.canvas = canvas;
    numLoad += 1;
  };
}

export type Props = {
  object: proto.IObject;
  showInfo?: boolean;
  scale?: number;
};
export default function ObjectLabel({ object, showInfo = false, scale = 1 }: Props) {
  const [ctx, setCtx] = useState<CanvasRenderingContext2D>();
  const [texture, setTexture] = useState<THREE.Texture>();
  const spriteRef = useRef<THREE.Sprite>();
  const res = (
    <sprite
      ref={spriteRef}
      scale={[(2 / ASPECT) * scale, 2 * scale, 1]}
      center={[0, 0] as any as THREE.Vector2}
      visible={false}>
      <spriteMaterial
        {...{
          sizeAttenuation: true,
          depthTest: false,
          map: texture,
        }}
      />
    </sprite>
  );
  useEffect(() => {
    const canvas = document.createElement("canvas");
    canvas.height = FONT_SIZE * LINE_NUM + ICON_SIZE;
    canvas.width = LABEL_WIDTH;
    setCtx(canvas.getContext("2d")!);
    setTexture(new THREE.CanvasTexture(canvas));
  }, []);
  useEffect(() => {
    const sprite = spriteRef.current;
    if (object && numLoad >= icons.length && sprite && ctx && texture) {
      sprite.visible = true;
      sprite.position.z = object.box.height / 2;

      object.type > 0 && ctx.drawImage(icons[object.type - 1].canvas!, 0, FONT_SIZE * LINE_NUM);
      const info = [
        `(${object.id},` +
          `${object.confidence.toFixed(2)},` +
          `${object.box.center.x.toFixed(2)},` +
          `${object.box.center.y.toFixed(2)},` +
          `${object.box.length.toFixed(2)},` +
          `${object.box.width.toFixed(2)},` +
          `${object.box.heading.toFixed(2)})`,
        `(${object.velocityX.toFixed(2)},` +
          `${object.velocityY.toFixed(2)},` +
          `${object.angleRate.toFixed(2)},` +
          `${object.accelX.toFixed(2)},` +
          `${object.age},` +
          `${object.valid},` +
          `${object.status})`,
      ];
      ctx.clearRect(0, 0, LABEL_WIDTH, FONT_SIZE * LINE_NUM);
      if (showInfo && info) {
        ctx.fillStyle = "rgba(240, 240, 240, 1.0)";
        ctx.font = `Bold ${FONT_SIZE}px Arial`;
        for (let index = 0; index < info.length; index++) {
          const line = info[index];
          ctx.fillText(line, 0, FONT_SIZE * (index + 1));
        }
      }
      texture.needsUpdate = true;
    }
  });
  return res;
}
