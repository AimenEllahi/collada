(function () {

	const track_length = parameters.x_stroke + 1000;								// X-axis parameters (Track)
	const track_segments = Math.ceil(track_length / 2000); 

	const default_Y_middle = 375;													// Y-axis parameters (pillar cantilever)												
	const default_Y_stroke = 2000;
	const Y_diff = parameters.y_stroke - default_Y_stroke;
	const Y_scale = (Y_diff + default_Y_middle) / default_Y_middle;

	const default_flange = 2000;													// Z-axis parameters (vertical adjustment)
	const default_Z_middle = 500;
	const bumper_distance = parameters.z_stroke + 799.6;
	const new_middle = bumper_distance - 2299.6;
	const Z_scale = new_middle / default_Z_middle; 

	const default_pillar = 3048.12;													// Pillar / height parameters 
	const Z_diff = new_middle - default_Z_middle; 
	const flange_diff = parameters.height - default_flange; 
	const height_difference = flange_diff + Z_diff; 
	const new_pillar_height = (parameters.height + bumper_distance + 137.6) - 1894.1; 
	const pillar_scale = new_pillar_height / default_pillar; 

	const default_platform_length = 1470;											// Platform / ladder parameters 
	const new_platform_length = parameters.y_stroke - 530;
	const platform_Y_scale = new_platform_length / default_platform_length;
	const default_rail = 1418; 
	const rail_scale = ((new_platform_length - default_platform_length) + default_rail) / default_rail;  
	const default_ladder_height = 3744; 
	const ladder_right_scale = (default_ladder_height + height_difference) / default_ladder_height;
	const ladder_left_scale = ((default_ladder_height + 1081 + height_difference)) / default_ladder_height;
	const total_pillar = new_pillar_height + 840; 
	const cage_rail_scale = (2413 + height_difference) / 2413; 

	let placeTrack = function () {

		let result = "";
		let x = 0; 

		for (var i = 0; i < (track_segments - 1); i++) {

			x += 2000; 

			result += `
			<visual>
				<geometry>
					<mesh filename="meshes/Track2.dae" scale="1.00000 1.00000 1.00000" />
				</geometry>
				<origin xyz="${x} 0 0" rpy="0 0 0" />
			</visual>
			`
		}

		return result; 
	}

	let placeRung = function (object, repeat) {

		let result = "";
		let x = repeat*(-1);
		let count = Math.ceil(total_pillar / repeat);
		let mesh = ""; 

		if (object == 0) {
			mesh = "Service_Platform_Ladder_Rung.dae";
		}
		else if (object == 1) {
			mesh = "Service_Platform_Ladder_Bracket.dae";
		}
		else {
			mesh = "Service_Platform_Cage_Rung.dae";
			count = Math.ceil((total_pillar - 2477) / repeat);
		}

		for (var i = 0; i < (count - 1); i++) {

			x += repeat;

			if ((object == 2) & (x > (3083 + height_difference))) {
				mesh = "Service_Platform_Cage_Open_Rung.dae";
			}

			if ((object == 2) & (count < 1) & (i == 0)) {
				count = 3; 
				repeat = (total_pillar - 2477) / 2; 
			}

			result += `
			<visual>
				<geometry>
					<mesh filename="meshes/${mesh}" scale="1.00000 1.00000 1.00000" />
				</geometry>
				<origin xyz="255.00000 560.00000 ${x}" rpy="0 0 0" />
			</visual>
			`
		}

		return result; 
	}

	let weldPackage = function () {

		let result = "";

		if (parameters.weld == 1) {
			result += `
			<visual>
				<geometry>
					<mesh filename="meshes/Welding_Package.dae" scale="1.00000 1.00000 1.00000" />
				</geometry>
				<origin xyz="-1448.20000 0 579.00000" rpy="0 0 0" />
			</visual>
			`
		}

		return result; 
	}

	return `

<?xml version = "1.0"?>
<!-- Generated with Verbotics Cell Editor -->
<robot name="HD_Gantry">

	<joint name="base_joint" type="fixed">
		<parent link="base_link" />
		<child link="Gantry_link" />
	</joint>
	
	<joint name="Gantry_joint" type="fixed">
		<parent link="Gantry_link" />
		<child link="Track_link" />
	</joint>
		
	<joint name="Pillar_joint" type="prismatic">
		<parent link="Track_link" />
		<child link="Pillar_link" />
		<origin xyz="603.00000 123.17000 765.00000" rpy="0 0 0" />
		<axis xyz="1.00000 0 0" />
		<limit lower="0" upper="${parameters.x_stroke}" effort="30" velocity="6283185.307179586" />
	</joint>
			
	<joint name="Z-axis_plate_joint" type="prismatic">
		<parent link="Pillar_link" />
		<child link="Z-axis_plate_link" />
		<origin xyz="372.00000 ${-3308.37000 - Y_diff} ${3175 + height_difference}" rpy="0 0 0" />
		<axis xyz="0 1.00000 0" />
		<limit lower="0" upper="${parameters.y_stroke}" effort="30" velocity="6283185.307179586" />
	</joint>
	
	<joint name="Z-axis_joint" type="prismatic">
		<parent link="Z-axis_plate_link" />
		<child link="Z-axis_link" />
		<origin xyz="389.50000 255.00000 997.20000" rpy="0 0 0" />
		<axis xyz="0 0 1.00000" />
		<limit lower="0" upper="${parameters.z_stroke}" effort="30" velocity="6283185.307179586" />
	</joint>

	<link name="base_link" />
	<link name="Gantry_link" />
	<link name="Track_link">
		<visual>
			<geometry>
				<mesh filename="meshes/Track1.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
		</visual>
		${placeTrack()}
	</link>


	<link name="Pillar_link">
		<visual>
			<geometry>
				<mesh filename="meshes/Pillar_Base.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Pillar_Middle.dae" scale="1.00000 1.00000 ${pillar_scale}" />
			</geometry>
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Pillar_Top.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="0 0 ${3048.10000 + height_difference}" rpy="0 0 0" />
		</visual>

		${weldPackage()}

		<visual>
			<geometry>
				<mesh filename="meshes/Y-axis_Base.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="0 -434.97000 ${3048.10000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Y-axis_Middle.dae" scale="1.00000 ${Y_scale} 1.00000" />
			</geometry>
			<origin xyz="0 -509.97000 ${3048.10000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Y-axis_End.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="0 ${-884.97000 - Y_diff} ${3048.10000 + height_difference}" rpy="0 0 0" />
		</visual>

		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Landing.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="-548.27000 0 ${3083.12000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Floor.dae" scale="1.00000 ${platform_Y_scale} 1.00000" />
			</geometry>
			<origin xyz="-1097.80000 -1888.47000 ${3083.12000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_End.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="-1137.78000 ${-3386.50000 - Y_diff} ${3083.12000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Support.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="-1097.78000 ${-1888.47000 - (new_platform_length / 2)} ${3083.12000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Wire.dae" scale="1.00000 ${rail_scale} 1.00000" />
			</geometry>
			<origin xyz="-1117.78000 ${-3326.47000 - Y_diff} ${3506.12000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Wire.dae" scale="1.00000 ${rail_scale} 1.00000" />
			</geometry>
			<origin xyz="-1117.78000 ${-3326.47000 - Y_diff} ${3926.12000 + height_difference}" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Rail.dae" scale="1.00000 ${rail_scale} 1.00000" />
			</geometry>
			<origin xyz="-1117.78000 ${-3326.47000 - Y_diff} ${4366.12000 + height_difference}" rpy="0 0 0" />
		</visual>

		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Ladder_Rail.dae" scale="1.00000 1.00000 ${ladder_left_scale}" />
			</geometry>
			<origin xyz="255.00000 560.00000 0" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Ladder_Rail.dae" scale="1.00000 1.00000 ${ladder_right_scale}" />
			</geometry>
			<origin xyz="-255.00000 560.00000 0" rpy="0 0 0" />
		</visual>

		${placeRung(0, 300)}
		${placeRung(1, 900)}
		${placeRung(2, 755)}

		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Ladder_Bracket.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="255.00000 560.00000 ${(ladder_right_scale * default_ladder_height) - 297}" rpy="0 0 0" />
		</visual>

		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Cage_Open_Rung.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="255.00000 560.00000 ${(ladder_left_scale * default_ladder_height) - 2477}" rpy="0 0 0" />
		</visual>

		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Cage_Open_Rung.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
			<origin xyz="255.00000 560.00000 ${(ladder_left_scale * default_ladder_height) - 2477 - 945}" rpy="0 0 0" />
		</visual>

		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Cage_Rail_Left.dae" scale="1.00000 1.00000 ${cage_rail_scale}" />
			</geometry>
			<origin xyz="255.00000 560.00000 2412" rpy="0 0 0" />
		</visual>
		<visual>
			<geometry>
				<mesh filename="meshes/Service_Platform_Cage_Rail_Right.dae" scale="1.00000 1.00000 ${cage_rail_scale}" />
			</geometry>
			<origin xyz="255.00000 560.00000 2412" rpy="0 0 0" />
		</visual>

	</link>



	<link name="Z-axis_plate_link">
		<visual>
			<geometry>
				<mesh filename="meshes/Z-axis_Plate.dae" scale="1.00000 1.00000 1.00000" />
			</geometry>
		</visual>
	</link>

	<link name="Z-axis_link">
	<visual>
		<geometry>
			<mesh filename="meshes/Z-axis_Top.dae" scale="1.00000 1.00000 1.00000" />
		</geometry>
	</visual>
	<visual>
		<geometry>
			<mesh filename="meshes/Z-axis_Middle.dae" scale="1.00000 1.00000 ${Z_scale}" />
		</geometry>
		<origin xyz="0 0 -1837.20000" rpy="0 0 0" />
	</visual>
	<visual>
		<geometry>
			<mesh filename="meshes/Z-axis_End.dae" scale="1.00000 1.00000 1.00000" />
		</geometry>
		<origin xyz="0 0 ${-2337.20000 - Z_diff}" rpy="0 0 0" />
	</visual>
</link>

</robot>
`
})