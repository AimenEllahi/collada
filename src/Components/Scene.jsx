import React, { Suspense, useRef } from "react";
import { Canvas } from "@react-three/fiber";
import { Environment, ContactShadows } from "@react-three/drei";
import { OrbitControls } from "@react-three/drei";
import SPN from "./Models/SPN";
import Loader from "./Loader";
import Single from "./Models/Single";
import Dual from "./Models/Dual";
import HD from "./Models/HD";

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
            <group rotation={[-Math.PI / 2, 0, 0]}>
              <SPN position={[-3, 0, 0]} scale={0.4} />
              <Single scale={0.4} position={[-2, 0, 0]} />
              <Dual scale={0.4} />
              <HD scale={0.4} position={[2, 0, 0]} />
            </group>
          </Suspense>
          <OrbitControls />
        </Canvas>
      </div>
    </div>
  );
}
