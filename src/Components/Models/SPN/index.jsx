import React from "react";
import MeshRendeder from "../../Shared/MeshRenderer";
import presets from "./preset.json";

const PATHS = [
  "/SPN/meshes/Base_mesh.dae",
  "/SPN/meshes/Top_mesh.dae",
  "/SPN/meshes/Middle_mesh.dae",
  "/SPN/meshes/Faceplate_mesh.dae",
];

export default function index() {
  console.log(presets);
  return (
    <group>
      {PATHS.map((path, index) => (
        <MeshRendeder key={index} path={path} />
      ))}
    </group>
  );
}
