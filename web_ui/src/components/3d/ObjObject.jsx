import * as React from "react";
import { useEffect, useRef } from "react";
import * as THREE from "three";
import { useGLTF } from "@react-three/drei";
import car from "@assets/car.glb";

export default function ObjObject({ visible = false, props }) {
  const { nodes, materials } = useGLTF(car);
  const groupRef = useRef();

  const res = (
    <group ref={groupRef} dispose={null} visible={visible}>
      <group rotation={[0, 0, Math.PI / 2]} scale={0.009}>
        <group position={[0, 0, 90]} scale={100}>
          <mesh
            geometry={nodes.Plane029_Material001_0.geometry}
            material={new THREE.MeshBasicMaterial({ color: "orange" })}
          />
          <mesh geometry={nodes.Plane029_Material002_0.geometry} material={materials["Material.002"]} />
          <mesh geometry={nodes.Plane029_Material003_0.geometry} material={materials["Material.003"]} />
          <mesh geometry={nodes.Plane029_Material004_0.geometry} material={materials["Material.004"]} />
          <mesh geometry={nodes.Plane029_Material005_0.geometry} material={materials["Material.005"]} />
          <mesh geometry={nodes.Plane029_Material006_0.geometry} material={materials["Material.006"]} />
          <mesh geometry={nodes.Plane029_Material007_0.geometry} material={materials["Material.007"]} />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane030_Material002_0.geometry} material={materials["Material.002"]} />
          <mesh geometry={nodes.Plane030_Material_0.geometry} material={materials.Material} />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane031_Material002_0.geometry} material={materials["Material.002"]} />
          <mesh geometry={nodes.Plane031_Material_0.geometry} material={materials.Material} />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane032_Material002_0.geometry} material={materials["Material.002"]} />
          <mesh geometry={nodes.Plane032_Material_0.geometry} material={materials.Material} />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane033_Material003_0.geometry} material={materials["Material.003"]} />
          <mesh
            geometry={nodes.Plane033_Material009_0.geometry}
            material={new THREE.MeshBasicMaterial({ color: "red" })}
          />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane034_Material003_0.geometry} material={materials["Material.003"]} />
          <mesh
            geometry={nodes.Plane034_Material009_0.geometry}
            material={new THREE.MeshBasicMaterial({ color: "red" })}
          />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane035_Material002_0.geometry} material={materials["Material.002"]} />
          <mesh geometry={nodes.Plane035_Material_0.geometry} material={materials.Material} />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane036_Material003_0.geometry} material={materials["Material.003"]} />
          <mesh
            geometry={nodes.Plane036_Material009_0.geometry}
            material={new THREE.MeshBasicMaterial({ color: "red" })}
          />
        </group>
        <group position={[0, 0, 90]} scale={100}>
          <mesh geometry={nodes.Plane037_Material003_0.geometry} material={materials["Material.003"]} />
          <mesh
            geometry={nodes.Plane037_Material009_0.geometry}
            material={new THREE.MeshBasicMaterial({ color: "red" })}
          />
        </group>
      </group>
    </group>
  );

  useEffect(() => {
    if (groupRef.current) {
      if (props) {
        groupRef.current.rotation.set(props["rotation"][0], props["rotation"][1], props["rotation"][2]);
        groupRef.current.position.set(props["position"][0], props["position"][1], props["position"][2]);
      } else {
        groupRef.current.rotation.set(0, 0, 0);
        groupRef.current.rotateZ(0);
        groupRef.current.position.set(0, 0, 0);
      }
    }
  }, [props]);
  return res;
}

useGLTF.preload(car);
