import * as THREE from "three";
import React, { useEffect, useRef, useState } from "react";
import vertexShader from "@components/3d/shader/roi_vertex_shader.glsl?raw";
import fragmentShader from "@components/3d/shader/roi_fragment_shader.glsl?raw";

import { FontLoader } from "three/examples/jsm/loaders/FontLoader";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
import helvetiker from "three/examples/fonts/helvetiker_regular.typeface.json";
import { extend, ThreeEvent, useThree } from "@react-three/fiber";

export interface Props {
  meta?: LSD.MapMeta;
  enable: boolean;
  selectArea: string | undefined;
  onAreaClick: any;
  onCheckClickElement: any;
}

export const AreaView = React.memo(({ meta, enable, selectArea, onAreaClick, onCheckClickElement }: Props) => {
  return (
    <>
      {meta &&
        meta.area &&
        Object.keys(meta.area).map((id, idx) => {
          return (
            <AreaComponent
              key={"AreaView" + id}
              id={id}
              area={meta.area[id]}
              enable={enable}
              isSelect={id == selectArea}
              onAreaClick={onAreaClick}
              onCheckClickElement={onCheckClickElement}
            />
          );
        })}
    </>
  );
});

type IProps = {
  id: string;
  area: any;
  enable: boolean;
  isSelect: boolean;
  onAreaClick: any;
  onCheckClickElement: any;
};

function AreaComponent({ id, area, enable, isSelect, onAreaClick, onCheckClickElement }: IProps) {
  extend({ TextGeometry });
  const { camera } = useThree();
  const [center, setCenter] = useState<THREE.Vector3>(new THREE.Vector3(0, 0, 0));
  const helvetikerRegular = new FontLoader().parse(helvetiker);
  const uniforms = {
    color1: {
      value: new THREE.Color("white"),
    },
    color2: {
      value: new THREE.Color("black"),
    },
  };

  let color = 0x000000;
  if (area && area["type"]) {
    if (isSelect) {
      color = 0xa00000;
    } else if (area["type"] == "indoor") {
      color = 0xa0a000;
    } else if (area["type"] == "outdoor") {
      color = 0x00a000;
    }
  }

  const colorMaterial = new THREE.MeshBasicMaterial({
    color: color,
    transparent: true,
    opacity: 0.8,
    side: THREE.DoubleSide,
  });

  const areaContour0 = useRef<THREE.Mesh>();
  const areaContour1 = useRef<THREE.Mesh>();
  const areaMeshRef = useRef<THREE.Mesh>();

  const onHandleClick = (e: ThreeEvent<MouseEvent>) => {
    if (enable) {
      e.stopPropagation();
      onAreaClick(id, e);
    }
  };

  const onHanleContextMenu = (e: ThreeEvent<MouseEvent>) => {
    if (enable) {
      e.stopPropagation();
      onCheckClickElement(e, camera);
    }
  };

  useEffect(() => {
    if (area) {
      const contour = area["polygon"];
      let vertices: number[] = [];
      let uvs: number[] = [];
      let vec2s = [];
      if (contour.length > 0) {
        let centerSum = new THREE.Vector3(0, 0, 0);
        for (const pt of contour) {
          centerSum.x = centerSum.x + pt[0];
          centerSum.y = centerSum.y + pt[1];
          centerSum.z = centerSum.z + pt[2];
          vertices = vertices.concat([pt[0], pt[1], pt[2] + 0]);
          vertices = vertices.concat([pt[0], pt[1], pt[2] + 1]);
          uvs = uvs.concat([0, 0]);
          uvs = uvs.concat([0, 1]);
          vec2s.push(new THREE.Vector2(pt[0], pt[1]));
        }
        centerSum.x = centerSum.x / contour.length;
        centerSum.y = centerSum.y / contour.length;
        centerSum.z = centerSum.z / contour.length;
        setCenter(centerSum);
        vertices = vertices.concat([contour[0][0], contour[0][1], contour[0][2] + 0]);
        vertices = vertices.concat([contour[0][0], contour[0][1], contour[0][2] + 1]);
        uvs = uvs.concat([0, 0]);
        uvs = uvs.concat([0, 1]);
      }
      for (let areaContour of [areaContour0, areaContour1]) {
        if (areaContour.current) {
          areaContour.current.geometry.setAttribute(
            "position",
            new THREE.BufferAttribute(new Float32Array(vertices), 3)
          );
          areaContour.current.geometry.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(uvs), 2));

          let indexs: number[] = [];
          for (let i = 0; i < contour.length; i++) {
            let j = i * 2;
            indexs = indexs.concat([j, j + 1, j + 2, j + 2, j + 1, j + 3]);
          }
          areaContour.current.geometry.setIndex(indexs);
        }
      }

      const areaMesh = areaMeshRef.current;
      if (!areaMesh) return;
      if (vec2s.length > 0) {
        areaMesh.geometry = new THREE.ShapeGeometry(new THREE.Shape(vec2s));
        areaMesh.visible = true;
        areaMesh.material = colorMaterial;
      }
    }
  }, [id, area, isSelect]);

  return (
    <group>
      <mesh position={center} rotation={[0, 0, -3.14 / 2.0]}>
        <textGeometry args={[area.name, { font: helvetikerRegular, size: 3, height: 0.1 }]} />
      </mesh>
      <mesh ref={areaMeshRef} onDoubleClick={onHandleClick} onContextMenu={onHanleContextMenu}>
        <shapeGeometry />
      </mesh>
      <mesh ref={areaContour0}>
        <bufferGeometry />
        <shaderMaterial
          {...{
            uniforms,
            vertexShader,
            fragmentShader,
            transparent: true,
            opacity: 0.4,
            side: THREE.BackSide,
            renderOrder: 0,
          }}
        />
      </mesh>
      <mesh ref={areaContour1}>
        <bufferGeometry />
        <shaderMaterial
          {...{
            uniforms,
            vertexShader,
            fragmentShader,
            transparent: true,
            opacity: 0.4,
            side: THREE.FrontSide,
            renderOrder: 1,
          }}
        />
      </mesh>
    </group>
  );
}
