# grasp_learning
Learning to grasp MVBBs with Soft-Hands

## Creating poses
From the `python` folder, you can execute `python grasp_boxes_batch.py db_file.csv` to create an output database file `db_file_simulated.csv` with all successful and unsuccessful grasps, given a database `db_file.csv` containing box dimensions for a given box, and desired `o_T_p` pose which describes the palm pose in the object frame of reference. The box frame of reference is placed at the center of its axis, and is axis-aligned.
The data format for the file is:
```
box_x_dim, box_y_dim, box_z_dim, p_x, p_y, p_z, q_x, q_y, q_z, q_w
```
With `[p_x, p_y, p_z]` the translation part of the `p_T_h` transform, and `[q_x, q_y, q_z, q_w]` the unitary quaternion representing the rotational part of `p_T_h`

The script `simple_batch_splitter.py` can be also called in the same form, `python grasp_boxes_batch.py db_file.csv` and will parallelize the computation over all available cores in the machine.
The obtained simulated database can then be analyzed with the `view_boxes_grasps.py` script.

For instance, the pipeline would look something like:
```
cd python;
python simple_batch_splitter.py ../db/box_db_test_only_successful.csv
python view_bozes_grasps.py ../db/box_db_test_only_successful.csv
```

## Database usage
Once the database with successful and unsuccessful grasps has been made, an SVM is used to generate a function to use in real time to estimate a valid grasp pose of a novel obejct. A valid pose is intended to be a grasp pose with the higher probability of success in executing the grasp. 

## Final execution
Finally, in the execution of a main program, the SVM's learnt function is used to give in output a valid grasp pose over a novel object.
