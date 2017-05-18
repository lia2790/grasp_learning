 1 - MOdify file param/box_poses
	1.1 in box_input set the box dimensions
		- example line 4= box_input: [0.08, 0.08, 0.08]


	1.2 - Ceate folder to save results
		- example line 5: mkdir -p box_008/

2 - Launcht the file roslaunch grasp_learning box_poses.launch
	 -  ctl+c
	 - cd python 
	 - python ./simple_batch_splitter.py ../box_008/box_db_10.csv
	 - 

3 - in a new terminal : launch the file roslaunch grasp_learning display.launch
4 - Modify file param/


