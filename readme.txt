*****************************************************************
General way of running the system:

1. make install the vehicles
2. run the simulator, choose the track (normally "E_track_5" or "Aalborg")
3. set the car [10 cars total: 5 front 1 host 4 back] such as (yyf_1~yyf_5 Player yyf_7~yyf10) or (yyf_1~yyf_5 yyf_6 yyf_7~yyf10)
	the position of the host vehicle is defined as "leadingCarLabel"+1 in "yyf.cpp"

4. run roscore, trosc.py, display_Tcoordinate, rviz
	trosc.py
		for training: set the Training = True
		for validation [not done yet]: set the Training = False
	display_Tccordinate (receive BufferData, send markers to rviz)

5. for storage, run rosbag /buffer_data

*****************************************************************
Parameters and Special tasks

In "yyf", the most important function is
	Driver(int index, float max_speed = FLT_MAX, bool inverse_driving = false, int initial_pose=-1, int leading_car=-1); //(See "driver.cpp")

	max_speed: the maximum speed for the car [m/s]
	inverse_driving: driving inverse or not
	initial_pose: if initial_pose>=0 the car will stop at the pose, waiting for the leading_car
	leading_car: the label of the leading_car, when it drives, all the waiting cars drive

 
1. For "Snake Driving Task"
	set "INDEX_FOR_CHANGE" in "Driver.cpp" into '0~9', choose the "INDEX_FOR_CHANGE"+1 car in the simulator
	The style is decided by "offset" in function "getTargetPoint" (line 485)

2. For "Speed Controller Data"
	set "INDEX_FOR_VELOCITY" in "Driver.cpp" into '0~9', choose the "INDEX_FOR_VELOCITY"+1 car in the simulator
	More details are in the function "VelocityTest()"
	Hint: If "FileDir" in "VelocityTest()" is wrong, the simulator might be crashed

3. For "Robot Overtaking Task"
	set "INDEX_FOR_OVERTAKING" in "Driver.cpp" into '0~9', choose the "INDEX_FOR_OVERTAKING"+1 car in the simulator, this car will save files and send messages like human_ros
	Hint: "INDEX_FOR_OVERTAKING" '0~9' is for yyf1~yyf10, the "leadingCarLabel" in "yyf.cpp" is for the simulator
		example:
			order in simulator:	yyf1,	yyf2,	yyf5
						INDEX:	0,		1,		4
					simulator:	0,		1,		2
		the simplest way is put the order yyf1~10, and let "INDEX_FOR_OVERTAKING" = "leadingCarLabel"

	WAIT_FRAME_FOR_OVERTAKING = 4000: the host vehicle will start after 4000 frames (80 seconds)
	MIN_DIST_FOR_OVERTAKING = 20: the distance of the front vehicle for overtaking
	MIN_SPEED_FOR_OVERTAKING = 30.0: if the allowedSpeed < 30.0, the vehicle will not overtake

*****************************************************************
For the Files

Important:
	DataCollection.cpp, DataCollection.h, facade.cpp, facade.h, semaphore.cpp, semaphore.h, structs.h
	in "yyf", "human_ros", "trosc" are always the same
		for now, "human_ros" has the newest version
