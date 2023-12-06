(function () {

	const body = ["Driver", "Idler"];									// Strings for link/joint functions
	const segment = ["Base", "Middle", "Top", "Faceplate"];
	const default_height = (parameters.payload <= 7500) ? 1250 : 1500;	// Set default rotational axis height based on user input
	const height_difference = parameters.height - default_height;		// Determine change (+/-) in elevation to scale middle segments in z-axis

	// 4 x dimension arrays used for origin placement based on payload type
	// [Driver_faceplate_offset,Base_origins_distance,Idler_faceplate_offset, Driver_base_height,Driver_middle_height_new,Driver_top_height,Provided input height,Idler_base_height,Idler_middle_height_new,Idler_top_height]
	const P1_dims = [314, parameters.fixture_distance + 297, 297, 320, height_difference + 500, 430, parameters.height, 320, height_difference + 500, 430];
	const P2_dims = [447, parameters.fixture_distance + 368, 368, 520, height_difference + 250, 480, parameters.height, 520, height_difference + 410, 320];
	const P3_dims = [556, parameters.fixture_distance + 548, 548, 650, height_difference + 250, 600, parameters.height, 650, height_difference + 250, 600];
	const P4_dims = [783, parameters.fixture_distance + 560, 560, 600, height_difference + 250, 650, parameters.height, 600, height_difference + 250, 650];

	let setPayloadVariables = function (input) {						// This function can be used to output variables if they are dependant on the payload (add other set states to return_array)

		let type = parameters.payload;
		let return_array = [];

		if (input == 0) {
			return_array = ["P1", "P2", "P3", "P4"];					// Payload class
		}

		output = (type <= 3000) ? return_array[0] : return_array[1];
		output = ((type > 3000) && (type <= 7500)) ? return_array[1] : output;
		output = ((type > 7500) && (type <= 15000)) ? return_array[2] : output;
		output = (type > 15000) ? return_array[3] : output;

		return output;
	}

	let PL = setPayloadVariables(0);									// Payload class - Used as control variable and string for link/joint functions

	var dim_array = [];
	switch (PL) {														// New dimensional array used for origin/joint placement and scale factor calculations 
		case "P1":
			dim_array = P1_dims;
			break;
		case "P2":
			dim_array = P2_dims;
			break;
		case "P3":
			dim_array = P3_dims;
			break;
		case "P4":
			dim_array = P4_dims;
			break;
	}

	const default_driver_middle = dim_array[4] - height_difference;     // Get default middle section heights from the dim array
	const default_idler_middle = dim_array[8] - height_difference;

	var zero = Array(8).fill(0);										// Create arrays containing x,y,z values for joint placement in sequence (to simplify for loops)
	var x_array = zero;
	x_array[3] = dim_array[0];
	x_array[4] = dim_array[1];
	x_array[7] = dim_array[2] * (-1);
	var z_array = dim_array.slice(3);
	z_array.unshift(0);
	z_array[4] = z_array[4] * (-1); 

	let getScale = function (body) {									// Function called by createLinks() to get appropriate scale factor for mesh files  

		let scale_factor = 1;

		if (body == 0) {												
			scale_factor = dim_array[4] / default_driver_middle;		// Return driver scale for middle section
		}
		else {															
			scale_factor = dim_array[8] / default_idler_middle;			// Return idler scale for middle section
		}

		return scale_factor;
	}
	
	let createJoints = function () {									// Function with for loop to create string of text required to create joints in urdf 

		let result = "";												// String parameters used to compile joint/link names 
		let joint_name = "";
		let parent_link = "";
		let origin = ""; 
		let m = 0;														// Body array counter for parent link name: m = [0,0,0,0,0,1,1,1]
		let n = 0;														// Segment array counter for parent link name: n = [0,0,1,2,3,0,1,2]
		let p = 0;														// Body array counter for joint name: p = [0,0,0,0,1,1,1,1]
		let q = 0;														// Segment array counter for q = [0,1,2,3,0,1,2,3]

		for (var i = 0; i < 8; i++) {									// Each for loop creates one joint string 

			if (i == 4) {												// Reset q and n array counters based on i value 
				q = 0; 

				if (parameters.tailstock == 0) {						// If tailstock is set to false then exit the for loop when the driver is complete
					break;
				}
			}
			if (i == 5) {
				n = 0; 
			}

			m = (i < 5) ? 0 : 1;										// Switch m and p array counters from 0 to 1 based on i value
			p = (i < 4) ? 0 : 1; 

			joint_name = body[p] + "_" + segment[q];					// Compile strings (joint name and child link name are equal)
			parent_link = (i < 1) ? "base" : body[m] + "_" + segment[n];
			origin = x_array[i] + " 0 " + z_array[i];

			q++;
			if (i >= 1) { 
				n++;
			}

			result += `
			<joint name="${joint_name}_joint" type="fixed">
				<parent link="${parent_link}_link" />
				<child link="${joint_name}_link" />
				<origin xyz="${origin}" rpy="0 0 0" />
			</joint>
			`
		}
		return result;
	}

	let createLinks = function () {										// Function with for loop to create string of text required to create links in urdf 

		let result = "";
		let j = 0;														// Segment array counter: j = [0,1,2,3,0,1,2,3]
		let k = 0;														// Body array counter: k = [0,0,0,0,1,1,1,1]

		for (var i = 0; i < 8; i++) {

			if (i == 4) {												// Reset segment array counter and set body array counter to 1 (for idler)
				j = 0;	
				k = 1; 

				if (parameters.tailstock == 0) {						// If tailstock is set to false then exit the for loop when the driver is complete
					break; 
				}
			}

			let link_name = body[k] + "_" + segment[j] + "_link"; 
			let mesh_name = "meshes/" + body[k] + "_" + segment[j] + "_" + PL + ".dae";

			let scale = 1; 
			if (j == 1) {												// Middle segment being placed > Call getScale() function
				scale = getScale(k);
			}

			result += `
			<link name="${link_name}">
				<visual>
					<geometry>
						<mesh filename="${mesh_name}" scale="1.00000 1.00000 ${scale}" /> 
					</geometry>
				</visual>
			</link>
			`

			j++; 
		}
		return result; 
	}

																		// Return URDF code for main function by calling the createLinks() and createJoints() functions
	return `															

	<?xml version = "1.0"?>
	<!-- Generated with Verbotics Cell Editor -->

	<robot name="1500_Mesh_Selection">
	${createJoints()}

	<link name="base_link" />
	${createLinks()}

</robot>`
})