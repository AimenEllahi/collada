import React, { Suspense, useRef } from "react";
import { Canvas } from "@react-three/fiber";
import { Environment, ContactShadows, useGLTF } from "@react-three/drei";
import { OrbitControls } from "@react-three/drei";
import SPN from "./Models/SPN";
import Loader from "./Loader";
import Single from "./Models/Single";
import Dual from "./Models/Dual";
import HD from "./Models/HD";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter.js";

const Models = () => {
  const { scene } = useGLTF("/positioner.glb");
  const { scene: scene2 } = useGLTF("/positioner.glb");
  const model2 = scene.clone();

  return (
    <>
      <primitive object={scene} scale={0.0005} />
      <primitive object={model2} scale={0.0005} position={[2, 0, 0]} />
    </>
  );
};

const Lights = () => {
  const ref = useRef();

  return (
    <>
      <ambientLight color={"#ffffff"} intensity={1.0} />
      <directionalLight
        color='#ffffff'
        intensity={3}
        position={[-100, 0, 100]}
      />
      <directionalLight
        color='#ffffff'
        intensity={4}
        position={[100, 0, 100]}
      />
      <directionalLight
        ref={ref}
        color={"#ffffff"}
        intensity={3}
        position={[100, 0, -100]}
      />
    </>
  );
};

export default function Scene() {
  const canvasRef = useRef();

  const downloadSceneGLTF = () => {
    const exporter = new GLTFExporter();
    let object = canvasRef.current;

    exporter.parse(
      object, // Access the group
      async function (result) {
        let binaryData;
        if (result instanceof ArrayBuffer) {
          binaryData = result;
        } else {
          // Convert the JSON result
          const output = JSON.stringify(result, null, 2);
          binaryData = new TextEncoder().encode(output).buffer;
        }

        await saveArrayBuffer(
          binaryData,
          `scene-${new Date()
            .toTimeString()
            .split("(")[0]
            .replace(" ", "-")
            .replaceAll(":", "-")
            .replace("+", "-")
            .trim()}.gltf`
        );
      },
      { binary: true }
    );
  };

  // Define the saveArrayBuffer and saveString functions
  const saveArrayBuffer = async (data, filename) => {
    const blob = new Blob([data], { type: "model/octet-stream" });
    //download
    const link = document.createElement("a");
    link.style.display = "none";
    document.body.appendChild(link);
    link.href = URL.createObjectURL(blob);
    link.download = filename;
    link.click();
  };

  return (
    <div>
      <div className='fixed top-0 left-0 w-screen h-screen'>
        <Canvas shadows camera={{ fov: 45 }} className='z-10'>
          <Lights />
          <Environment preset='sunset' />

          <color attach='background' args={["#FFFFFF"]} />
          <ContactShadows
            scale={100}
            position={[0, 0, 0]}
            blur={1}
            far={100}
            opacity={1}
          />

          <Suspense fallback={<Loader />}>
            {/* <group rotation={[-Math.PI / 2, 0, 0]}>
              <SPN position={[-3, 0, 0]} scale={0.4} />
              <Single scale={0.4} position={[-2, 0, 0]} />
              <Dual scale={0.4} />
              <HD scale={0.4} position={[2, 0, 0]} />
            </group> */}
            <group ref={canvasRef}>
              <Models />
            </group>
          </Suspense>
          <OrbitControls />
        </Canvas>

        <button
          className='z-30 absolute text-black top-0'
          onClick={downloadSceneGLTF}
        >
          Download Model
        </button>
      </div>
    </div>
  );
}
