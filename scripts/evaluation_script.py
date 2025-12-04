from evaluate_trajectory import TrajectoryEval

# Example usage

HalfTank_Hard = "/home/ubuntu/ws_blue/data/pipeline_runs/tank/HalfTank_Hard/stereo_only/live_trajec/live_trajec.txt"
gt_HalfTank_Hard = "/home/ubuntu/ws_blue/data/ros2_bags/tank/gt/HalfTank_Hard/gt_data.txt"

HalfTank_Medium = "/home/ubuntu/ws_blue/data/pipeline_runs/tank/HalfTank_Medium/stereo_only/live_trajec/live_trajec.txt"
gt_HalfTank_Medium = "/home/ubuntu/ws_blue/data/ros2_bags/tank/gt/HalfTank_Medium/gt_data.txt"

Structure_Easy = "/home/ubuntu/ws_blue/data/pipeline_runs/tank/Structure_Easy/stereo_only/live_trajec/live_trajec.txt"
gt_Structure_Easy = "/home/ubuntu/ws_blue/data/ros2_bags/tank/gt/Structure_Easy/gt_data.txt"

Structure_Medium = "/home/ubuntu/ws_blue/data/pipeline_runs/tank/Structure_Medium/stereo_only/live_trajec/live_trajec.txt"
gt_Structure_Medium = "/home/ubuntu/ws_blue/data/ros2_bags/tank/gt/Structure_Medium/gt_data.txt"

Structure_Hard = "/home/ubuntu/ws_blue/data/pipeline_runs/tank/Structure_Hard/stereo_only/live_trajec/live_trajec.txt"
gt_Structure_Hard = "/home/ubuntu/ws_blue/data/ros2_bags/tank/gt/Structure_Hard/gt_data.txt"

te = TrajectoryEval(odometry_path=Structure_Easy,
                    gt_path=gt_Structure_Easy,
                    sensor_config="stereo", gravity_vector=[-0, -1, 0])
# rotation around vector [-0.00385631,  0.99990967, -0.01287541]
# unnormalised [-0.01175016,  3.04671612, -0.03923125]
# te.draw_trajectory(gt=True, add_orientation_gt=0, add_orientation_est=0)
# te.align(align_all_frames=True)
# te.draw_trajectory(gt=True, add_orientation_est=0, add_orientation_gt=0)
# te.align(align_all_frames=False)
# te.draw_trajectory(gt=True, add_orientation_est=0, add_orientation_gt=0)
err = te.relative_error(trajec_lenghts=(0.5, 0.7, 1), show=True)
# ate = te.absolue_trajectory_error()
# print(f"{ate[0]:.3f}m -- ATE position error")
# print(f"{ate[1]:.2f}° -- ATE rotation error")
# print(f"{(te.frac_gt_used * 100):.1f}% -- Percentage of GT poses used" )
# print(f"GT length: {te.gt_length}")
# te.relative_error(trajec_lenghts=(2, 5, 10))
