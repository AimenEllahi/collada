import React, { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { Environment } from "@react-three/drei";
import { OrbitControls } from "@react-three/drei";
import SPN from "./Models/SPN";
import Loader from "./Loader";
import Single from "./Models/Single";
import Dual from "./Models/Dual";
import HD from "./Models/HD";

export default function Scene() {
  return (
    <div>
      <div className='fixed top-0 left-0 w-screen h-screen'>
        <Canvas className='z-10'>
          <ambientLight intensity={1} />
          <directionalLight intensity={0.5} />
          <Suspense fallback={<Loader />}>
            <Environment preset='sunset' />

            <SPN position={[-3, 0, 0]} scale={0.4} />
            <Single scale={0.4} position={[-2, 0, 0]} />
            <Dual scale={0.4} />
            <HD scale={0.4} position={[2, 0, 0]} />
          </Suspense>
          <OrbitControls />
        </Canvas>
      </div>
    </div>
  );
}
