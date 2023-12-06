import React from "react";
import { useLoader } from "@react-three/fiber";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader";
const Model = ({ path, key }) => {
  const { nodes } = useLoader(ColladaLoader, path);

  console.log(nodes);

  return <primitive key={key} object={nodes ? nodes.Scene : null} />;
};

export default Model;
