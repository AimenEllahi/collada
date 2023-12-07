const parameters = {
  payload: 5000,
  height: 750,
  diameter: 500,
  tilt: 1500,
  toggle: 0,
};
const scaling_factor = 0.001;
//fix model paths and scaling
export const exporter = () => {
  // ******************************************************************************************************
  // Determine required model based on payload, workpiece diameter and tilt axis height OR workpiece height
  // ******************************************************************************************************

  const payload = [500, 1500, 2000, 3000, 5000, 7500, 10000, 15000, 20000]; // Payload class
  const min_tilt_axis_height = [
    750, 750, 1000, 1000, 1250, 1250, 1250, 1250, 1250,
  ]; // Min tilt axis / loading height
  const max_tilt_axis_height = [
    1250, 1500, 1750, 1750, 2000, 2000, 2250, 2250, 2250,
  ]; // Max tilt axis height
  const max_input_fixture = [
    1750, 2250, 2500, 2500, 2750, 2750, 2750, 2750, 2750,
  ]; // Max fixture distance (exclduing the min loading height)
  const max_workpiece_diameter = [
    2000, 3000, 3000, 3000, 3500, 3500, 4000, 4000, 4000,
  ]; // Max diameter of the workpiece
  const min_rotation_axis_height = [
    400, 750, 750, 750, 1000, 1000, 1250, 1250, 1250,
  ]; // Min horizontal distance between arm and rotational axis
  let tilt_axis_height = 0;
  let rotational_height = 0;
  let index = 0;

  let limitCheck = function (input, array) {
    // Function determines minimum required index from input parameter and corresponding array of values
    let result;
    for (var i = 0; i < 9; i++) {
      if (input == array[i] || input < array[i]) {
        result = i;
        break;
      }
    }
    return result;
  };

  if (
    (parameters.payload == 5000) &
    (parameters.height == 500) &
    (parameters.diameter == 500) & // Bypass calculations for default display state
    (parameters.tilt == 1500) &
    (parameters.toggle == 0)
  ) {
    index = 4;
    PL = 5000;
    tilt_axis_height = parameters.tilt;
    rotational_height = min_rotation_axis_height[index];
  } else {
    // Determine parameters for custom model

    let payload_index = limitCheck(parameters.payload, payload); // Use limitCheck function to find minimum required model based on payload
    let diameter_index = limitCheck(
      parameters.diameter,
      max_workpiece_diameter
    ); // Use limitCheck function to find minimum required model based on workpiece diameter
    index = Math.max(payload_index, diameter_index); // Determine maximum index value based on payload and diameter

    // Rotational axis height options: set max height -> comment out second line below, set custom height - > comment out first line below
    //rotational_height = max_rotational_axis_height[index];														// Set rotational height to max value
    rotational_height = parameters.diameter / 2; // Set rotational height based on workpiece diameter input

    if (parameters.toggle == 0) {
      // Set height with tilt input if toggle is 0

      let height_index = limitCheck(parameters.tilt, max_tilt_axis_height); // Update existing index to take tilt axis height into account
      index = Math.max(height_index, index);
      tilt_axis_height = parameters.tilt;
    } else {
      // Set height with workpiece if toggle is 1

      let fixture_distance = parameters.height;

      let height_index = limitCheck(fixture_distance, max_input_fixture); // Update existing index to take max fixture distance into account
      index = Math.max(height_index, index);

      tilt_axis_height = (fixture_distance + min_tilt_axis_height[index]) / 2; // Calculate the tilt axis height from fixture distance input
    }
  }

  let LB_model_index = index;

  if (tilt_axis_height < min_tilt_axis_height[LB_model_index]) {
    // Check in case of min tilt axis case
    tilt_axis_height = min_tilt_axis_height[LB_model_index];
  }

  if (rotational_height < min_rotation_axis_height[LB_model_index]) {
    // Check in case of min rotational axis case
    rotational_height = min_rotation_axis_height[LB_model_index];
  }

  // ******************************************************************************************************
  // Assign mesh file name array based on the required model index
  // ******************************************************************************************************

  let mesh = [];

  if (LB_model_index < 4) {
    if (LB_model_index <= 1) {
      // 500 - 1500 model
      mesh = [
        "TB0",
        "TM0",
        "TT0",
        "C0",
        "AT1",
        "AB1",
        "RB1",
        "RM1",
        "RT1",
        "RF1",
        "RR1",
      ];
    } else {
      // 2000 - 3000 model

      if (LB_model_index == 2) {
        mesh = [
          "TB1",
          "TM1",
          "TT1",
          "C0",
          "AT1",
          "AB1",
          "RB1",
          "RM1",
          "RT1",
          "RF1",
          "RR1",
        ];
      } else {
        mesh = [
          "TB1",
          "TM1",
          "TT1",
          "C0",
          "AT1",
          "AB1",
          "RB2",
          "RM2",
          "RT2",
          "RF2",
          "RR2",
        ];
      }
    }
  } else {
    if (LB_model_index < 6) {
      // 5000 - 7500 model
      mesh = [
        "TB2",
        "TM2",
        "TT2",
        "C0",
        "AT1",
        "AB1",
        "RB2",
        "RM2",
        "RT2",
        "RF2",
        "RR2",
      ];
    } else {
      // 10000, 15000 and 20000 model
      mesh = [
        "TB3",
        "TM3",
        "TT3",
        "TF3",
        "C0",
        "AT1",
        "AB1",
        "RB3",
        "RM3",
        "RT3",
        "RF3",
        "RR3",
      ];
    }
  }

  let scale = [
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
    "1 1 1",
  ]; // Set scale and origin arrays to default values
  let origins = [
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
    "0 0 0",
  ];

  // ******************************************************************************************************
  // Calculate mesh scaling parameters based on the required model index
  // ******************************************************************************************************

  let default_tilt_axis_height = [
    950, 950, 1250, 1250, 1500, 1500, 1500, 1500, 1500,
  ]; // Tilt positioner vertical scaling
  let default_tilt_middle_height = [
    200, 200, 250, 250, 250, 250, 250, 250, 250,
  ];
  let tilt_axis_height_difference =
    tilt_axis_height - default_tilt_axis_height[LB_model_index];
  let tilt_middle_scale =
    (tilt_axis_height_difference + default_tilt_middle_height[LB_model_index]) /
    default_tilt_middle_height[LB_model_index];

  let default_rotational_axis_height = [
    450, 450, 450, 1500, 1500, 1500, 2000, 2000, 2000,
  ]; // Rotational positioner horizontal scaling
  let default_rotational_middle_height = [
    50, 50, 50, 750, 750, 750, 750, 750, 750,
  ];
  let rotational_axis_height_difference =
    rotational_height - default_rotational_axis_height[LB_model_index];
  let rotational_middle_scale =
    (rotational_axis_height_difference +
      default_rotational_middle_height[LB_model_index]) /
    default_rotational_middle_height[LB_model_index];

  const default_CW_width = 361; // Counterweight scaling
  const default_CW_depth = 714;
  const default_CW_height = 343;
  const new_CW_width = [205, 280, 275, 361, 500, 500, 550, 550, 550];
  const new_CW_depth = [400, 600, 600, 714, 830, 830, 1000, 1250, 1250];
  const new_CW_height = [255, 240, 240, 343, 170, 170, 430, 430, 430];
  let CW_X_scale = new_CW_width[LB_model_index] / default_CW_width;
  let CW_Y_scale = new_CW_depth[LB_model_index] / default_CW_depth;
  let CW_Z_scale = new_CW_height[LB_model_index] / default_CW_height;

  const new_arm_width = [197, 272, 272, 321, 450, 450, 500, 500, 500]; // Arm scaling
  const new_arm_depth = [400, 600, 600, 714, 830, 830, 1000, 1250, 1250];
  const new_arm_base_height = [15, 15, 15, 15, 26.43, 26.43, 25.4, 25.4, 25.4];
  let CW_Z_offset = [
    396.1, 396.1, 396.1, 578.75, 845, 845, 879.59, 729.59, 729.59,
  ];
  let rotational_faceplate_to_arm_base = [
    414, 592.31, 599.2, 561.24, 694.25, 694.25, 724.12, 738.05, 752.88,
  ];
  let default_arm_width = 321;
  let default_arm_depth = 714;
  let default_arm_height = 1625;
  let default_arm_base_height = 15;
  let default_arm_base_width = 341.156;
  let new_top_segment_height =
    tilt_axis_height +
    CW_Z_offset[LB_model_index] -
    (min_tilt_axis_height[LB_model_index] -
      (rotational_faceplate_to_arm_base[LB_model_index] -
        new_arm_base_height[LB_model_index]));
  let arm_base_Z_scale =
    new_arm_base_height[LB_model_index] / default_arm_base_height;
  let arm_base_X_scale = new_arm_width[LB_model_index] / default_arm_base_width;

  let arm_X_scale = new_arm_width[LB_model_index] / default_arm_width;
  let arm_Y_scale = new_arm_depth[LB_model_index] / default_arm_depth;
  let arm_Z_scale = new_top_segment_height / default_arm_height;

  // ******************************************************************************************************
  // Set the scale (x,y,z) values for each model type
  // ******************************************************************************************************

  let current_width = 0; // Tilt positioner scaled for 1500, 15000 and 20000 models
  let current_depth = 0;
  let new_width2 = 0;
  let new_depth2 = 0;
  let width_scale = 0;
  let depth_scale = 0;

  if (LB_model_index == 1) {
    // 1500 model
    current_width = 570;
    current_depth = 600;
    new_width2 = 690;
    new_depth2 = 690;
    width_scale = new_width2 / current_width;
    depth_scale = new_depth2 / current_depth;
  } else if (LB_model_index > 6) {
    // 15000 & 20000 models
    current_width = 1442;
    current_depth = 1350;
    new_width2 = 1590;
    new_depth2 = 1440;
    width_scale = new_width2 / current_width;
    depth_scale = new_depth2 / current_depth;
  } else {
    width_scale = 1;
    depth_scale = 1;
  }

  let default_rotational_Y = [600, 600, 600, 830, 830, 830, 1250, 1250, 1250]; // Rotational positioner needs to be scaled for 500, 2000, 3000 and 10000 models
  let default_rotational_Z = [570, 570, 570, 657, 657, 657, 705, 705, 705];
  let new_rotational_Y = [400, 600, 601.84, 714, 830, 830, 1000, 1250, 1250];
  let new_rotational_Z = [363.3, 563.15, 585, 513.28, 657, 657, 705, 705, 705];
  let rotational_Y_scale =
    new_rotational_Y[LB_model_index] / default_rotational_Y[LB_model_index];
  let rotational_Z_scale =
    new_rotational_Z[LB_model_index] / default_rotational_Z[LB_model_index];

  let rotational_faceplate_height = [32.9, 26.5, 16, 30, 31, 31, 30, 30, 45]; // Scale the rotational faceplate
  let default_faceplate_height = [25, 25, 25, 31, 31, 31, 45, 45, 45];
  let faceplate_Z_scale =
    rotational_faceplate_height[LB_model_index] /
    default_faceplate_height[LB_model_index];

  let new_diameter = [380, 500, 485, 650, 750, 750, 960, 1100, 1100];
  let default_diameter = [500, 500, 500, 750, 750, 750, 1100, 1100, 1100];
  let diameter_scale =
    new_diameter[LB_model_index] / default_diameter[LB_model_index];

  let fixed_faceplate_height = [57, 62.5, 60, 57, 56, 56, 73, 87, 87]; // Scale the fixed faceplate
  let default_fixed_faceplate_height = [
    61.5, 61.5, 61.5, 56, 56, 56, 87, 87, 87,
  ];
  let fixed_faceplate_Z_scale =
    fixed_faceplate_height[LB_model_index] /
    default_fixed_faceplate_height[LB_model_index];
  let new_X = [380, 617, 812, 980, 1084, 1084, 1250, 1604, 1604];
  let new_Y = [330, 330, 325, 659, 759, 759, 880, 1130, 1130];
  let default_X = [617, 617, 617, 1084, 1084, 1084, 1604, 1604, 1604];
  let default_Y = [330, 330, 330, 759, 759, 759, 1130, 1130, 1130];
  let fixed_faceplate_X_scale =
    new_X[LB_model_index] / default_X[LB_model_index];
  let fixed_faceplate_Y_scale =
    new_Y[LB_model_index] / default_Y[LB_model_index];

  let rotational_X_scale = 1; // Scale rotational top segments for scaled models
  if (LB_model_index == 0) {
    rotational_X_scale = 467.5 / 605.5;
  }

  let TF3_X_scale = "1 1 1"; // Separate faceplate for 10000, 15000 and 20000 models
  if (LB_model_index > 6) {
    TF3_X_scale = 1.039;
  }

  scale[0] = width_scale + " " + depth_scale + " 1"; // Set the mesh scale values for models without a separate tilt positioner faceplate
  scale[1] = width_scale + " " + depth_scale + " " + tilt_middle_scale;
  scale[2] = width_scale + " " + depth_scale + " 1";
  scale[3] = CW_X_scale + " " + CW_Y_scale + " " + CW_Z_scale;
  scale[4] = arm_X_scale + " " + arm_Y_scale + " " + arm_Z_scale;
  scale[5] = arm_base_X_scale + " " + arm_Y_scale + " " + arm_base_Z_scale;
  scale[6] = "1 " + rotational_Y_scale + " " + rotational_Z_scale;
  scale[7] =
    rotational_middle_scale +
    " " +
    rotational_Y_scale +
    " " +
    rotational_Z_scale;
  scale[8] =
    rotational_X_scale + " " + rotational_Y_scale + " " + rotational_Z_scale;
  scale[9] =
    fixed_faceplate_X_scale +
    " " +
    fixed_faceplate_Y_scale +
    " " +
    fixed_faceplate_Z_scale;
  scale[10] = diameter_scale + " " + diameter_scale + " " + faceplate_Z_scale;

  if (LB_model_index >= 6) {
    // Set the mesh scale values for models WITH a separate tilt positioner faceplate
    scale[0] = width_scale + " " + depth_scale + " 1";
    scale[1] = width_scale + " " + depth_scale + " " + tilt_middle_scale;
    scale[2] = width_scale + " " + depth_scale + " 1";
    scale[3] = TF3_X_scale + " 1 1";
    scale[4] = CW_X_scale + " " + CW_Y_scale + " " + CW_Z_scale;
    scale[5] = arm_X_scale + " " + arm_Y_scale + " " + arm_Z_scale;
    scale[6] = arm_base_X_scale + " " + arm_Y_scale + " " + arm_base_Z_scale;
    scale[7] = "1 " + rotational_Y_scale + " " + rotational_Z_scale;
    scale[8] =
      rotational_middle_scale +
      " " +
      rotational_Y_scale +
      " " +
      rotational_Z_scale;
    scale[9] =
      rotational_X_scale + " " + rotational_Y_scale + " " + rotational_Z_scale;
    scale[10] =
      fixed_faceplate_X_scale +
      " " +
      fixed_faceplate_Y_scale +
      " " +
      fixed_faceplate_Z_scale;
    scale[11] = diameter_scale + " " + diameter_scale + " " + faceplate_Z_scale;
  }

  // ******************************************************************************************************
  // Set the origin (x,y,z) values for each model type
  // ******************************************************************************************************

  let tilt_base_height = [400, 400, 520, 520, 650, 650, 600, 600, 600]; // Set parameters required for origins
  let rotational_base_height = [50, 50, 50, 26, 26, 26, 25, 25, 25];
  let tilt_top_z =
    tilt_base_height[LB_model_index] +
    default_tilt_middle_height[LB_model_index] +
    tilt_axis_height_difference;
  let rotational_top_x =
    rotational_base_height[LB_model_index] +
    default_rotational_middle_height[LB_model_index] +
    rotational_axis_height_difference;
  let CW_X_offset = [-5, -5, 0, -20, -25, -25, -25, -25, -25];
  let arm_base_Z = CW_Z_offset[LB_model_index] - new_top_segment_height;
  let fixed_faceplate_Z_offset = [
    144.29, 222.21, 230.895, 217.46, 278.5, 278.5, 268.62, 268.5, 268.5,
  ]; // Half depth of the rotational positioner

  let TF3_X_origin = 651.11; // Separate tilt faceplate for 10000, 15000 and 20000 models
  if (LB_model_index > 6) {
    TF3_X_origin = new_width2 / 2 - 88.26;
  }

  origins[1] = "0 0 " + tilt_base_height[LB_model_index] * scaling_factor; // Set the mesh origin values for models without a separate tilt positioner faceplate
  origins[2] = "0 0 " + tilt_top_z * scaling_factor;
  origins[3] =
    CW_X_offset[LB_model_index] * scaling_factor +
    " 0 " +
    CW_Z_offset[LB_model_index] * scaling_factor;
  origins[4] = "0 0 " + CW_Z_offset[LB_model_index] * scaling_factor;
  origins[5] = "0 0 " + arm_base_Z * scaling_factor;
  origins[7] = rotational_base_height[LB_model_index] * scaling_factor + " 0 0";
  origins[8] = rotational_top_x * scaling_factor + " 0 0";
  origins[9] =
    rotational_height * scaling_factor +
    " 0 " +
    (fixed_faceplate_Z_offset[LB_model_index] +
      fixed_faceplate_height[LB_model_index]) *
      scaling_factor;

  if (LB_model_index >= 6) {
    // Set the mesh origin values for models WITH a separate tilt positioner faceplate
    origins[1] = "0 0 " + tilt_base_height[LB_model_index] * scaling_factor;
    origins[2] = "0 0 " + tilt_top_z * scaling_factor;
    origins[3] =
      TF3_X_origin * scaling_factor + " 0 " + tilt_top_z * scaling_factor;
    origins[4] =
      CW_X_offset[LB_model_index] * scaling_factor +
      " 0 " +
      CW_Z_offset[LB_model_index] * scaling_factor;
    origins[5] = "0 0 " + CW_Z_offset[LB_model_index] * scaling_factor;
    origins[6] = "0 0 " + arm_base_Z * scaling_factor;
    origins[7] = "0 0 0";
    origins[8] =
      rotational_base_height[LB_model_index] * scaling_factor + " 0 0";
    origins[9] = rotational_top_x * scaling_factor + " 0 0";
    origins[10] =
      rotational_height * scaling_factor +
      " 0 " +
      (fixed_faceplate_Z_offset[LB_model_index] +
        fixed_faceplate_height[LB_model_index]) *
        scaling_factor;
  }

  // ******************************************************************************************************
  // Create links
  // ******************************************************************************************************

  let createLinks = function (link, mesh_count, index_start, index_end) {
    // Function that creates the necessary link code based on required model

    if (LB_model_index >= 6) {
      if (link == 0) {
        mesh_count = 4;
        index_start = 0;
        index_end = 4;
      } else {
        index_start += 1;
        index_end += 1;
      }
    }

    let result = "";
    let mesh_names = [];
    let scale_values = [];
    let origin_values = [];

    mesh_names = mesh.slice(index_start, index_end);
    scale_values = scale.slice(index_start, index_end);
    origin_values = origins.slice(index_start, index_end);

    for (var i = 0; i < mesh_count; i++) {
      let mesh_string = "/Dual/meshes/" + mesh_names[i] + ".dae";
      console.log(origin_values[i]);
      result += `
				<visual>
					<geometry>
						<mesh filename="${mesh_string}" scale="${scale_values[i]}" />
					</geometry>
					<origin xyz="${origin_values[i]}" rpy="0 0 0" />
				</visual>
				`;
    }
    return result;
  };

  // ******************************************************************************************************
  // Set joint origins
  // ******************************************************************************************************

  let tilt_centre_dist = [
    296, 333, 427, 451, 548.18, 548.18, 753, 826.42, 826.42,
  ];
  let rotational_z2 =
    (tilt_axis_height -
      (min_tilt_axis_height[LB_model_index] -
        rotational_faceplate_height[LB_model_index] -
        fixed_faceplate_height[LB_model_index] -
        fixed_faceplate_Z_offset[LB_model_index])) *
    -1;
  let tilt_axis_joint_origin =
    tilt_centre_dist[LB_model_index] * scaling_factor +
    " 0 " +
    tilt_axis_height * scaling_factor;
  let rotational_positioner_joint_origin = [];
  let rotational_axis_joint_origin =
    rotational_height * scaling_factor +
    " 0 " +
    (fixed_faceplate_Z_offset[LB_model_index] +
      fixed_faceplate_height[LB_model_index]) *
      scaling_factor;
  let workpiece_joint =
    "0 0 " + rotational_faceplate_height[LB_model_index] * scaling_factor;

  if (LB_model_index <= 2) {
    rotational_positioner_joint_origin =
      (new_arm_width[LB_model_index] - 3) * scaling_factor +
      " 0 " +
      rotational_z2 * scaling_factor;
  } else {
    rotational_positioner_joint_origin =
      new_arm_width[LB_model_index] * scaling_factor +
      " 0 " +
      rotational_z2 * scaling_factor;
  }

  // ******************************************************************************************************
  // URDF Code
  // ******************************************************************************************************

  return `
	<?xml version = "1.0"?>
	<!-- Generated with Verbotics Cell Editor -->
	<robot name="DAP_Test">
	
		<joint name="Base_Joint" type="fixed">
			<parent link="base_link" />
			<child link="Tilt_Axis_Positioner_Link" />
		</joint>

		<joint name="Tilt_Axis_Joint" type="revolute">
			<parent link="Tilt_Axis_Positioner_Link" />
			<child link="Tilt_Axis_Link" />
			<origin xyz="${tilt_axis_joint_origin}" rpy="0 0 0" />
			<axis xyz="1.00000 0 0" />
			<limit lower="0" upper="6.28" effort="30" velocity="628" />
		</joint>

		<joint name="Rotational_Positioner_Joint" type="fixed">
			<parent link="Tilt_Axis_Link" />
			<child link="Rotational_Positioner_Link" />
			<origin xyz="${rotational_positioner_joint_origin}" rpy="0 0 0" />
		</joint>

		<joint name="Rotational_Axis_Joint" type="revolute">
			<parent link="Rotational_Positioner_Link" />
			<child link="Rotational_Axis_Link" />
			<origin xyz="${rotational_axis_joint_origin}" rpy="0 0 0" />
			<axis xyz="0 0 1.00000" />
			<limit lower="0" upper="6.28" effort="30" velocity="628" />
		</joint>

		<joint name="Workpiece_Joint" type="fixed">
			<parent link="Rotational_Axis_Link" />
			<child link="Workpiece_Link" />
			<origin xyz="${workpiece_joint}" rpy="0 0 0" />
		</joint>

		<link name="base_link" />
		<link name="Tilt_Axis_Positioner_Link">
			${createLinks(0, 3, 0, 3)}
		</link>
		<link name="Tilt_Axis_Link">
			${createLinks(1, 3, 3, 6)}
		</link>
		<link name="Rotational_Positioner_Link">
			${createLinks(2, 4, 6, 10)}
		</link>
		<link name="Rotational_Axis_Link">
			${createLinks(3, 1, 10, 11)}
		</link>
		<link name="Workpiece_Link" />
	</robot>
	`;
};
