#!/bib/bash

roslaunch grasp_learning box_poses.launch
cd src/grasp_learning/python/
python ./simple_batch_splitter_collision.py ../db/
cd ../../..
roslaunch grasp_learning svm.launch
roslaunch grasp_learning comunication_robot.launch

exit