import React from "react";

import URDFLoader from "urdf-loader";
import { exporter } from "./model.urdf";

export default function index({ position = [0, 0, 0], scale = 1 }) {
  const urdf = exporter();

  const loader = new URDFLoader();
  const robot = loader.parse(urdf.trim());

  return <primitive object={robot} position={position} scale={scale} />;
}
