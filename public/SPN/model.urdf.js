export const exporter = () => {
  let PL = parameters.payload; // Preset input parameters
  let new_height = parameters.height * 0.01;

  let XY_scale = 1; // Mesh scaling parameters
  let Z_scale = 1;
  let faceplate_scale = 1;

  let default_height = 650; // Mesh placement parameters
  let base_height = 25;
  let middle_height = 513;
  let height_difference = 0;
  let top_placement = 0;
  let faceplate_placement = 0;

  type = PL <= 1000 ? 0 : 1; // Set type based on payload input
  type = PL <= 2000 && PL > 1000 ? 1 : type;
  type = PL <= 3000 && PL > 2000 ? 2 : type;
  type = PL <= 5000 && PL > 3000 ? 3 : type;
  type = PL <= 10000 && PL > 5000 ? 4 : type;
  type = PL <= 20000 && PL > 10000 ? 5 : type;
  type = PL <= 40000 && PL > 20000 ? 6 : type;

  switch (
    type // Set XY scaling values for mesh files based on payload class
  ) {
    case 0:
      XY_scale = 0.632;
      faceplate_scale = 0.666;
      break;
    case 1:
      XY_scale = 0.649;
      faceplate_scale = 0.646;
      break;
    case 2:
      XY_scale = 0.979;
      faceplate_scale = 0.866;
      break;
    case 4:
      XY_scale = 1.204;
      faceplate_scale = 1.28;
      break;
    case 5:
      XY_scale = 1.448;
      faceplate_scale = 1.466;
      break;
    case 6:
      XY_scale = 1.561;
      faceplate_scale = 1.673;
      break;
  }

  height_difference = new_height - default_height; // Set vertical scaling and placement parameters
  Z_scale = (height_difference + middle_height) / middle_height;
  top_placement = base_height + middle_height + height_difference;
  faceplate_placement = default_height + height_difference;

  let createJoints = function () {
    // URDF code to create joints

    return `
		<joint name="positioner_joint" type="fixed">
			<parent link="base_link" />
			<child link="positioner_link" />
		</joint>
		<joint name="faceplate_joint" type="revolute">
			<parent link="positioner_link" />
			<child link="faceplate_link" />
			<origin xyz="0 0 ${faceplate_placement}" rpy="0 0 0" />
			<axis xyz="0 0 1.00000" />
			<limit lower="0" upper="6.2831853071795862" effort="30" velocity="6283185.307179586" />
		</joint>
		<joint name="workpiece_joint" type="fixed">
			<parent link="faceplate_link" />
			<child link="workpiece_link" />
		</joint>
		`;
  };

  let createLinks = function () {
    // URDF code to create links

    return `
		<link name="base_link" />
		<link name="positioner_link">
			<visual>
				<geometry>
					<mesh filename="meshes/Base_mesh.dae" scale="${XY_scale} ${XY_scale} 1.00000" />
				</geometry>
			</visual>
			<visual>
				<geometry>
					<mesh filename="meshes/Middle_mesh.dae" scale="${XY_scale} ${XY_scale} ${Z_scale}" />
				</geometry>
				<origin xyz="0 0 ${base_height}" rpy="0 0 0" />
			</visual>
			<visual>
				<geometry>
					<mesh filename="meshes/Top_mesh.dae" scale="${XY_scale} ${XY_scale} 1.00000" />
				</geometry>
				<origin xyz="0 0 ${top_placement}" rpy="0 0 0" />
			</visual>
			
		</link>
		<link name="faceplate_link">
			<visual>
				<geometry>
					<mesh filename="meshes/Faceplate_mesh.dae" scale="${faceplate_scale} ${faceplate_scale} 1.00000" />
				</geometry>
			</visual>
		</link>
		<link name="workpiece_link" />
		`;
  };
  // URDF code
  return `															
	<?xml version = "1.0"?>
	<!-- Generated with Verbotics Cell Editor -->
	<robot name="SPN-V Single Axis Positioner V2.0">
		${createJoints()}
		${createLinks()}
	</robot>
	`;
};
