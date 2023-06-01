import React, { useEffect, useMemo, useRef, useState } from "react";
import Controls, { CONTROL_TYPE } from "./Controls";
import { TextureLoader } from "three";
import produce from "immer";
import { useCreation } from "ahooks";
import * as THREE from "three";

export function useTexture(src?: string, onLoad?: () => void) {
  const texLoader = useCreation(() => new TextureLoader(), []);
  const [texture, setTexture] = useState<THREE.Texture>(); //new THREE.Texture()
  const [width, setWidth] = useState(640);
  const [height, setHeight] = useState(480);

  useEffect(() => {
    if (src) {
      texLoader.load(src, (tex) => {
        setTexture(tex);
        setWidth(tex.image.width);
        setHeight(tex.image.height);
        onLoad && onLoad();
      });
    }
  }, [src]);
  return { texture, width, height };
}

export interface Props {
  src?: string;
  pointsPicked: THREE.Vector3[];
  onChange: (pts: THREE.Vector3[]) => void;
  enabled: boolean;
  removing: boolean;
  onTextureLoad?: () => void;
}

export default function ImagePicker({ src, pointsPicked, onChange, enabled, removing = false, onTextureLoad }: Props) {
  const ref = useRef({ width: null, height: null });

  const { texture, width, height } = useTexture(src, onTextureLoad);
  const ratio = height / width;
  const [canvasIndexes] = useState(new Array(10).fill(0).map((x, i) => document.createElement("canvas")));
  useEffect(() => {
    canvasIndexes.forEach((canvas, i) => {
      const context = canvas.getContext("2d");
      let fontsize = 32;
      canvas.width = fontsize * 10;
      canvas.height = fontsize;
      // draw text
      if (context) {
        context.fillStyle = "rgba(240, 0, 0, 1.0)";
        context.font = `Bold ${fontsize}px Arial`;
        context.fillText(i + 1 + "", 0, fontsize);
      }
    });
  }, []);

  const addPoint = (pt: THREE.Vector3) => {
    console.log(pt);
    if (enabled && !removing) {
      onChange(
        produce(pointsPicked, (pts) => {
          pt.x += 0.5;
          pt.y = 0.5 - pt.y / ratio;
          pts.push(pt);
        })
      );
    }
  };

  const removePoint = (i: number) => {
    if (enabled && removing) {
      onChange(
        produce(pointsPicked, (pts) => {
          pts.splice(i, 1);
        })
      );
    }
  };
  return (
    <>
      <Controls type={CONTROL_TYPE.IMAGE} height={1.2} />
      <mesh
        onClick={(e) => {
          addPoint(e.point);
        }}
        scale={[1, 1 * ratio, 1]}>
        <planeBufferGeometry args={[1, 1]} />
        {texture ? <meshBasicMaterial map={texture} /> : null}
      </mesh>
      {pointsPicked.map((pt, i) => {
        return (
          <group position={[pt.x - 0.5, (0.5 - pt.y) * ratio, 0]} key={i} onClick={() => removePoint(i)}>
            <mesh>
              <sphereGeometry args={[0.003, 32, 32]} />
              <meshBasicMaterial color={0xff0000} />
            </mesh>
            <sprite
              position-x={0.005}
              position-y={0.005}
              scale={[0.5, 0.05, 1]}
              center={[0, 0] as any as THREE.Vector2}>
              <spriteMaterial sizeAttenuation={false} depthTest={false}>
                <canvasTexture attach={"map"} args={[canvasIndexes[i]]} />
              </spriteMaterial>
            </sprite>
          </group>
        );
      })}
    </>
  );
}
