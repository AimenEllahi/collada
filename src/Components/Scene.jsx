import React, { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { Environment } from "@react-three/drei";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader";
import { useLoader } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import Loader from "./Loader";

const Model = ({ model, modelKey }) => {
  const models = model.paths.map((path, index) => {
    const { nodes } = useLoader(ColladaLoader, path);

    return <primitive key={index} object={nodes ? nodes.Scene : null} />;
  });

  return (
    <group position={model.position} scale={2} key={modelKey}>
      {models}
    </group>
  );
};

export default function Scene() {
  const models = {
    SPN: {
      paths: [
        "/SPN/meshes/Base_mesh.dae",
        "/SPN/meshes/Top_mesh.dae",
        "/SPN/meshes/Middle_mesh.dae",
        "/SPN/meshes/Faceplate_mesh.dae",
      ],
      key: 1,
      position: [-2, 0, 0],
    },
    Single: {
      paths: ["/Single/meshes/Driver_Base_P1.dae"],
      key: 2,
      position: [0, 0, 0],
    },
    Dual: {
      paths: ["/Dual/meshes/AB0.dae", "/Dual/meshes/AB1.dae"],
      key: 3,
      position: [2, 0, 0],
    },
    HD: {
      paths: ["/HD/meshes/Pillar_Base.dae"],
      key: 4,
      position: [5, 0, 0],
    },
  };

  return (
    <div>
      <div className="fixed top-0 left-0 w-screen h-screen">
        <Canvas className="z-10">
          <ambientLight intensity={1} />
          <Suspense fallback={<Loader />}>
            <Environment preset="apartment" />
            {Object.values(models).map((model) => (
              <group key={model.key}>
                {model.paths.map((path, index) => (
                  <Model
                    key={index}
                    model={{
                      paths: [path],
                      position: model.position,
                      key: model.key,
                    }}
                    modelKey={model.key}
                  />
                ))}
              </group>
            ))}
          </Suspense>
          <OrbitControls />
        </Canvas>
      </div>
    </div>
  );
}
