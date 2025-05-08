def extract_gt_pose_simple(input_path, output_path):
    with open(input_path, 'r') as f:
        lines = f.readlines()

    poses = []
    position_line = False
    quaternion_line = False

    robot_pose = False

    for line in lines:
        line = line.strip()

        if "nsec:" in line:
            nsec = int(line.split(":")[1].strip())
            print("nsec")

        elif "sec:" in line:
            sec = int(line.split(":")[1].strip())

        elif 'name: "ground_vehicle/robot"' in line:
            robot_pose = True
        
        elif "position" in line and robot_pose:
            position_line   = True
            current_loc = []

        elif "orientation" in line and robot_pose:
            position_line   = False
            quaternion_line = True
            current_ori = []

        elif (("x:" in line) or ("y:" in line) or ("z:" in line) or ("w:" in line) ) and (quaternion_line or position_line) and robot_pose:
            print(f"position_line : {position_line}, quaternion_line : {quaternion_line}")
            if position_line:
                item = float(line.split(":")[1].strip())
                current_loc.append(item)
            elif quaternion_line:
                item = float(line.split(":")[1].strip())
                current_ori.append(item)

        if "w:" in line and quaternion_line and robot_pose:
            if len(current_loc) == 3 and len(current_ori)==4:
                pose = {
                    "t"  : sec + nsec*(1e-9),
                    "tx" : current_loc[0],
                    "ty" : current_loc[1],
                    "tz" : current_loc[2],
                    "qx" : current_ori[0],
                    "qy" : current_ori[1],
                    "qz" : current_ori[2],
                    "qw" : current_ori[3],
                }
                poses.append(pose)
            quaternion_line = False
            position_line   = False
            robot_pose      = False
            
    # Write the results to the output file in TUM format
    with open(output_path, 'w') as f:
        for pose in poses:
            f.write(f"{pose['t']:.9f} {pose['tx']} {pose['ty']} {pose['tz']} "
                    f"{pose['qx']} {pose['qy']} {pose['qz']} {pose['qw']}\n")

    print(f"Extracted {len(poses)} GT poses to {output_path}")

# Run the function
extract_gt_pose_simple("pipeline2/groundtruth.txt", "poses_tum.txt")
