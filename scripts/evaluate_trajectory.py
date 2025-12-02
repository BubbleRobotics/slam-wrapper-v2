"""
Evaluate a trajectory against ground truth data

Aligns the estimated trajectory to the ground truth, accounting for unobservables
in monocular, stereo, and inertial configurations. The alignment occurs as specified
by Zhang and Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual
(-Intertial) Odometry.

Implementation by clandsmeer
"""

from matplotlib import pyplot as plt
from pathlib import Path
import numpy as np
from cv2 import Rodrigues
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize_scalar


class TrajectoryEval:

    def __init__(self, odometry_path: str, gt_path: str, sensor_config: str="mono",
                 gravity_vector: list=None):
        """
        Evaluate a trajectory against ground truth data by aligning time stamps

        ### Parameters
        1. odometry_path : str
            Path to the .txt file containing the estimated poses. Absolute path.
            Expected format: (timestamp, tx, ty, tz, qx, qy, qz, qw) where columns
            are space separated and timestamp is a float or double in seconds unix time
        2. gt_path : str
            Path to the .txt file containing the ground truth poses. Absolute path.
            Expected format: (timestamp, tx, ty, tz, qx, qy, qz, qw) where columns
            are space separated and timestamp is a float or double in seconds unix time
        3. sensor_config : str
            Choose one of ["stereo", "mono", "inertial"]
            Select "interial" for either stereo-inertial or mono-inertial configurations.
            This determines the kind of alignment transformation that is applied to
            compensate for unobservables in the respective configurations.
        4. gravity_vector : list (default: None)
            - The gravity vector in world coordinates. Only needed for inertial. Pass
            list or np.array of shape (3,)
        """

        # Naming conventions:
        #   T_cw_list       a list of (3 x 4) np.arrays. T from w to current c frame
        #   T_cw_array      an np.array of (3n x 4) where n is the number of poses. Also 
        #                   contains T_cw
        #   T_wc_list       As above but containing the inverse of each matrix: T_wc
        #   T_wc_array      See above.
        #
        #   gt_T_wc_list    The ground truth version of T_wc_list
        #   gt_T_wc_array   The ground truth version of T_wc_array

        if sensor_config not in ["stereo", "mono", "inertial"]:
            raise ValueError("sensor_config must be one of ['stereo', 'mono', 'inertial']")
        else:
            self.sensor_config = sensor_config

        if sensor_config == "inertial":
            if gravity_vector is None:
                raise ValueError("gravity_vector must be provided for inertial configurations")
            else:
                self.gravity_vector = np.array(gravity_vector)
                l2_norm = np.sum(self.gravity_vector**2)**0.5 # Normalize
                self.gravity_vector = self.gravity_vector / l2_norm

        # ---------- LOAD EST. TRAJECTORY FROM FILE ---------- #

        trajec_file_path = Path(odometry_path)
        gt_file_path = Path(gt_path)

        trajec = np.loadtxt(trajec_file_path.as_posix())
        gt = np.loadtxt(gt_file_path.as_posix())

        # How many ground truth poses are in the file
        n_gt_poses = gt.shape[0]

        # Total length of the gt trajectory (computed in _re_get_split_pos())
        self.gt_length = None

        # Align trajectory and ground truth poses by time stamp
        trajec, gt = self.time_align(trajec, gt, threshold=0.001)

        self.n_poses = gt.shape[0]

        # The fraction of ground truth poses that was actually used:
        self.frac_gt_used = self.n_poses / n_gt_poses

        # Quaternions to rotation matrices
        trajc_rotations = Rotation.from_quat(trajec[:, 4:8]).as_matrix()
        gt_rotations = Rotation.from_quat(gt[:, 4:8]).as_matrix()

        # Adapt format of poses to what the rest of the code expects
        self.T_wc_list = []
        for i in range(self.n_poses):
            T = np.zeros((3, 4))
            T[:3, :3] = trajc_rotations[i]
            T[:3, 3] = trajec[i, 1:4]
            self.T_wc_list.append(T)
        
        # DEBUGGING: Applying rotation to the ground truth
        # u = np.array([1, 1, 1], dtype=float) / np.sqrt(3)
        # theta = np.pi / 6
        # # rotates points counter-clockwise around the vector u by theta radians
        # R = Rotation.from_rotvec(theta * u).as_matrix()

        self.gt_T_wc_list = []
        for i in range(self.n_poses):
            T = np.zeros((3, 4))
            T[:3, :3] = gt_rotations[i]
            T[:3, 3] = gt[i, 1:4]
            self.gt_T_wc_list.append(T)

        self.T_wc_array = np.vstack(self.T_wc_list)
        self.gt_T_wc_array = np.vstack(self.gt_T_wc_list)

    def draw_trajectory(self, gt: bool=False, add_orientation_gt: int=None, 
                        add_orientation_est: int=None,):
        """
        Draw the VO trajectory; plots the current self.T_wc

        ### Parameters
        1. gt : bool (default: False)
            - Inidicate whether to add the ground truth trajectory to the plot.
            Call self.similarity_transform_3d() before setting this to true
        2. add_orientation_gt : int (default: None)
            - Add the camera frame (3 arrows) at the corresponding frame index
            of the VO trajectory. If None: no camera frame is added.
            The colour coding of the arrows:
                - c_X : Red
                - c_Y : Green
                - c_Z : Blue
        3. add_orientation_est : int (default: None)
            - Add the camera frame (3 arrows) at the corresponding frame index
            of the VO trajectory. If None: no camera frame is added.
            The colour coding of the arrows:
                - c_X : Red
                - c_Y : Green
                - c_Z : Blue
        """

        w_t_wc__x = self.T_wc_array[0::3, 3]
        w_t_wc__y = self.T_wc_array[1::3, 3]
        w_t_wc__z = self.T_wc_array[2::3, 3]

        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.plot(w_t_wc__x, w_t_wc__y, w_t_wc__z, color="orange")
        ax.set_xlabel("$X_w$")
        ax.set_ylabel("$Y_w$")
        ax.set_zlabel("$Z_w$")
        ax.set_title("Absolute Trajectory Error")

        if gt:  # If the ground truth should be plotted as well
            gt_w_t_wc__x = self.gt_T_wc_array[0::3, 3]
            gt_w_t_wc__y = self.gt_T_wc_array[1::3, 3]
            gt_w_t_wc__z = self.gt_T_wc_array[2::3, 3]
            ax.plot(gt_w_t_wc__x, gt_w_t_wc__y, gt_w_t_wc__z, color="purple")
            ax.legend(["VO estimate", "Ground truth"])

            # Compute the limits for the plot; same scale for both axes
            xmin = np.min(np.r_[w_t_wc__x, gt_w_t_wc__x]) - 1
            xmax = np.max(np.r_[w_t_wc__x, gt_w_t_wc__x]) + 1
            ymin = np.min(np.r_[w_t_wc__y, gt_w_t_wc__y]) - 1
            ymax = np.max(np.r_[w_t_wc__y, gt_w_t_wc__y]) + 1
            zmin = np.min(np.r_[w_t_wc__z, gt_w_t_wc__z]) - 1
            zmax = np.max(np.r_[w_t_wc__z, gt_w_t_wc__z]) + 1
        
        else:
            # Compute the limits for the plot; same scale for both axes
            xmin, xmax = np.min(w_t_wc__x) - 1, np.max(w_t_wc__x) + 1
            ymin, ymax = np.min(w_t_wc__y) - 1, np.max(w_t_wc__y) + 1
            zmin, zmax = np.min(w_t_wc__z) - 1, np.max(w_t_wc__z) + 1

        max_range = max(xmax - xmin, zmax - zmin, ymax - ymin)
        mid_x = 0.5 * (xmin + xmax)
        mid_z = 0.5 * (zmin + zmax)
        mid_y = 0.5 * (ymin + ymax)

        # Set the computed limits
        ax.set_xlim(mid_x - 0.5 * max_range, mid_x + 0.5 * max_range)
        ax.set_ylim(mid_y - 0.5 * max_range, mid_y + 0.5 * max_range)
        ax.set_zlim(mid_z - 0.5 * max_range, mid_z + 0.5 * max_range)

        # Have the camera view be the orientation of the world coord sys.
        ax.view_init(elev=-80, azim=-90, roll=0)

        # Add a camera frame at some position if desired:
        if add_orientation_est is not None:
            self._draw_orientation(ax, gt=False, frame_idx=add_orientation_est)
        if add_orientation_gt is not None:
            self._draw_orientation(ax, gt=True, frame_idx=add_orientation_gt)
        
        plt.show()


    def similarity_transform_3d(self, align_all_frames: bool=True):
        """
        Apply a 3d similarity transform to the trajectory (Umeyama method).

        - For mono:
            Alignes the trajectory to the ground truth in terms of:
            - Absolute scale
            - Absolute rotation
            - Absolute translation

        - For stereo:
            Alignes the trajectory to the ground truth in terms of:
            - Absolute rotation
            - Absolute translation

        - For inertial (mono-inertial or stereo-inertial):
            Alignes the trajectory to the ground truth in terms of:
            - Rotation around gravity vector
            - Absolute translation

        ### Parameters
        1. align_all_frames : bool (default: True)
            - If True: use all frames to compute the similarity transform
            - If False: only use the first frame to compute the transform
        """

        # --------- FIND R, t, s BY CASE ---------- #

        if align_all_frames: 
            if self.sensor_config == "mono":
                R, t, s = self._find_transform_mono_stereo_all()
            elif self.sensor_config == "stereo":
                R, t, s = self._find_transform_mono_stereo_all()
                s = 1  # no scale correction for stereo
            elif self.sensor_config == "inertial":
                R, t, s = self._find_transform_inertial_all()
            # as checked in __init__(): sensor config is in ["mono", "stereo", "inertial"]
        else:
            if self.sensor_config == "stereo":
                R, t, s = self._find_transform_stereo_first()
            elif self.sensor_config == "inertial":
                R, t, s = self._find_transform_inertial_first()
            else:
                raise ValueError("Cannot align only using first frame in mono config. " \
                                 "To find the scale several poses are necessary.")

        # ---------- APPLY SIMILARITY TRANSFORM TO TRAJECTORY ---------- #

        w_t_wc = self.T_wc_array[:, 3].reshape(-1, 3)

        # Rotate the camera frame orientations by R (not rescaling by s):
        R_wc_dash = R @ self.T_wc_array[:, :3].reshape(-1, 3, 3)
        R_wc_dash = R_wc_dash.reshape(-1, 3)

        # Apply the similarity transform to the points w_t_wc
        # use post- multiply with R.T since w_t_wc has position vectors along 
        # the columns, not rows
        t_dash = s * w_t_wc @ R.T + t

        # Update self.T_wc_array and self.T_wc_list
        self.T_wc_array = np.hstack((R_wc_dash, t_dash.reshape(-1, 1)))
        self.T_wc_list = [self.T_wc_array[(3 * i):(3 * i + 3)] 
                          for i in range(self.n_poses)]


    def _find_transform_stereo_first(self):
        """
        Aligns the estimated trajctory to the ground truth by making the 
        orientation of the first estimated frame be the orientation of 
        the first ground truth frame

        Returns:
        1. R : np.array
            - The (3 x 3) rotation matrix
        2. t : np.array
            - The (3,) translation vector
        3. s : float
            - The scale factor: always 1.0 for stereo
        """

        # first pose ground truth
        gt_R_wc_0 = self.gt_T_wc_list[0][:, :3]
        gt_t_wc_0 = self.gt_T_wc_list[0][:, 3]

        # first pose estimated
        R_wc_0 = self.T_wc_list[0][:, :3]
        t_wc_0 = self.T_wc_list[0][:, 3]

        # Rotation from estimated to ground truth
        R = gt_R_wc_0 @ R_wc_0.T
        t = gt_t_wc_0 - R @ t_wc_0

        # Scale is not corrected, so return None for s
        return R, t, 1.0
        
    def _find_transform_inertial_first(self):
        """
        Aligns the estimated trajctory to the ground truth by making the 
        orientation of the first estimated frame be as close as possible to 
        the orientation of the first ground truth frame, however we only 
        allow rotation around the gravity vector.

        Returns:
        1. R : np.array
            - The (3 x 3) rotation matrix
        2. t : np.array
            - The (3,) translation vector
        3. s : float
            - The scale factor: always 1.0 for inertial
        """

        # first pose ground truth
        gt_R_wc_0 = self.gt_T_wc_list[0][:, :3]
        gt_t_wc_0 = self.gt_T_wc_list[0][:, 3]

        # first pose estimated
        R_wc_0 = self.T_wc_list[0][:, :3]
        t_wc_0 = self.T_wc_list[0][:, 3]

        def cost_fn(theta):
            R = Rotation.from_rotvec(theta * self.gravity_vector).as_matrix()
            return -1 * np.trace(R @ R_wc_0 @ gt_R_wc_0.T)

        # Find optimal roatation around gravity vector
        res = minimize_scalar(cost_fn, 
                              bounds=(-np.pi, np.pi), 
                              method='bounded')

        R = Rotation.from_rotvec(res.x * self.gravity_vector).as_matrix()
        t = gt_t_wc_0 - R @ t_wc_0

        # Scale is not corrected, so return 1 for s
        return R, t, 1.0

    def _find_transform_mono_stereo_all(self):
        """
        Use all positions of gt and estimated trajectory to find the similarity
        transform parameters R, t, s according to the Umeyama method.

        Returns:
        1. R : np.array
            - The (3 x 3) rotation matrix
        2. t : np.array
            - The (3,) translation vector
        3. s : float
            - The scale factor
        """

        # ---------- FIND R, t, s BY UMEYAMA ---------- #

        # The point coordinates in a (n x 3) array (...in world coordinates)
        gt_w_t_wc = self.gt_T_wc_array[:, 3].reshape(-1, 3)
        w_t_wc = self.T_wc_array[:, 3].reshape(-1, 3)

        # The mean point for the ground truth and estimated trajectors
        mu_p_hat = np.mean(w_t_wc, axis=0)          # mean trajectory point
        mu_p = np.mean(gt_w_t_wc, axis=0)           # mean g.t. point

        # The mean squared deviation of points
        sigma_p_hat_squared = np.sum((w_t_wc - mu_p_hat)**2) / self.n_poses
        sigma_p_squared = np.sum((gt_w_t_wc - mu_p)**2) / self.n_poses

        # Sigma matrix: sum of outer products (covariance)
        Sigma = (gt_w_t_wc - mu_p).T @ (w_t_wc - mu_p_hat) / self.n_poses

        # Singular value decomposition
        svd = np.linalg.svd(Sigma)

        # Find W based on the SVD
        if np.linalg.det(svd.U) * np.linalg.det(svd.Vh) < 0:
            W = np.diag((1, 1, -1))
        else:
            W = np.eye(3)
        
        # Find the solution rotation matrix R: essentially R_gt_w
        # from the VO world system to the ground truth system
        R = svd.U @ W @ svd.Vh

        # Find the solution scale factor s
        s = np.trace(np.diag(svd.S) @ W) / sigma_p_hat_squared

        # Find the solution translation vector t
        t = mu_p - s * R @ mu_p_hat

        return R, t, s

    def _find_transform_inertial_all(self):

        # The point coordinates in a (n x 3) array (...in world coordinates)
        gt_w_t_wc = self.gt_T_wc_array[:, 3].reshape(-1, 3)
        w_t_wc = self.T_wc_array[:, 3].reshape(-1, 3)

        # The mean point for the ground truth and estimated trajectors
        mu_p_hat = np.mean(w_t_wc, axis=0)          # mean trajectory point
        mu_p = np.mean(gt_w_t_wc, axis=0)           # mean g.t. point

        # Differences of positions from the mean of the trajectory
        P = (gt_w_t_wc - mu_p).T  # (3 x n)
        P_hat = (w_t_wc - mu_p_hat).T  # (3 x n)
        
        def cost_fn(theta, P, P_hat):
            R = Rotation.from_rotvec(theta * self.gravity_vector).as_matrix()
            return -1 * np.trace(R @ P_hat @ P.T)

        # Find optimal roatation around gravity vector
        res = minimize_scalar(lambda theta: cost_fn(theta, P, P_hat), 
                              bounds=(-np.pi, np.pi), 
                              method='bounded')

        R = Rotation.from_rotvec(res.x * self.gravity_vector).as_matrix()

        t = mu_p - R @ mu_p_hat

        return R, t, 1.0  # No scale correction for inertial
    
    def absolue_trajectory_error(self):
        """
        Compute the ATE. The trajectories should already be aligned by use
        of self.similarity_transform_3d() at this point

        ### Returns
        1. position error in m
        2. rotation error in deg
        """

        # Position error
        RMSE_pos = np.sqrt(np.sum((self.gt_T_wc_array[:, 3] 
                               - self.T_wc_array[:, 3])**2) / self.n_poses)
        
        # Rotation error
        Ri_hat = self.T_wc_array[:, :3].reshape(-1, 3, 3)
        Ri = self.gt_T_wc_array[:, :3].reshape(-1, 3, 3)
        delta_R = Ri @ Ri_hat.transpose(0, 2, 1)  # R_i * R_i_hat.T
        delta_angle = Rotation.from_matrix(delta_R).magnitude()

        RMSE_rot = np.sqrt(np.sum(delta_angle**2) / self.n_poses) * 180 / np.pi
        
        return RMSE_pos, RMSE_rot
    
    def _get_relative_error(self, trajectory_length: float):
        """
        The relative error is composed of two separable errors: roation and position.
        Sub-trajectories are aligned to the position and orientation of the first
        frame in the corresponding ground truth. The error is computed on the 
        last frame of the sub-trajectory.
        Scale drift can also be visualised by the change of alignment scales across
        the trajectory

        ### Parameters
        1. trajectory_length : float
            - Trajectory length in meters

        ### Returns
        1. position error : np.array
            - Shape (n_s,) where n_s is len(split_pos) - 1 ie. the maximum number
            of sub-trajectories of (at least) the given length that fit into the 
            estimated trajectory. 
            - For each of the n_s sub-trajectories: position error delta_p_k as 
            defined in equation 26 of the paper.
        2. rotation error : np.array
            - Shape (n_s,)
            - Rotation error in degrees, computed as delta_phi_k in equation 26
            of the paper
        3. alignment scales : np.array
            - Shape (n_s,)
            - Only for mono mode
            - Estimated trajectory is scaled such that the length of the estimated
            sub-trajectory is equal to the length of the sub-trajectory in the GT
            - Contains the scale factor used for each sub-trajectory
            - Used to quantify scale drift
        4. split and aligned trajectories : list(np.array)
            - List of n_s np.arrays
            - Each np.array: shape (3, n_p) where n_p is the number of poses
            in the corresponding sub-trajectory
            - Used to plot the sub-trajectories aligned at their first frame
        5. split_pos : list
            - List containing start and stop indices of each subtrajectory
            - The stop index of sub-trajectory i is the start index of 
            sub-trajectory i+1. Hence split_pos contains n_s + 1 integers
        """

        split_pos = self.re_get_split_pos(trajectory_length)
        split_trajec = []
        rot_err = np.zeros(len(split_pos) - 1)
        scales = np.zeros(len(split_pos) - 1)
        pos_err = np.zeros(len(split_pos) - 1)

        T_wc = self.T_wc_array.reshape(-1, 3, 4)
        gt_T_wc = self.gt_T_wc_array.reshape(-1, 3, 4)

        for i in range(len(split_pos) - 1):

            pos = split_pos[i]
            next_pos = split_pos[i + 1]

            # ---------- COMPUTE ROTATION ERROR ---------- #

            # Difference of rotation between the poses
            R_dash_s = gt_T_wc[pos, :, :3] @ T_wc[pos, :, :3].T 
            
            # First apply alignment to the rotation estimation of the last frame
            R_dash_e = R_dash_s @ T_wc[next_pos, :, :3]
            
            # Compute the rotation error at the last frame of the subtrajectory
            R_k = gt_T_wc[next_pos, :, :3] @ R_dash_e.T

            # Convert the rotation error to vector representation and save
            rot_err[i] = Rotation.from_matrix(R_k).magnitude() * 180 / np.pi

            # ---------- ALIGN THE SUBTRAJECTORIES ---------- #

            # Reshaping positions as (3, n) array
            p_hat = T_wc[pos:(next_pos+1), :, 3].T

            if self.sensor_config == "mono":
                
                # MONOCULAR: FIND SCALE

                # Extension to what the paper provides
                # Compute the scale such that the lengths of the estimated
                # sub-trajectories matcth the length of the corresponding
                # sub-trajectory in the GT
                
                # Length of estmiated sub-trajectory
                p_hat_shift = np.c_[p_hat[:, 1:], np.zeros((3, 1))]
                diff_vec_est = p_hat_shift - p_hat
                diff_vec_est = diff_vec_est[:, :-1]
                length_est = np.sum(np.sqrt(np.sum(diff_vec_est ** 2, axis=0)))

                # Length of corrsp. gt sub-trajectory
                p = gt_T_wc[pos:(next_pos+1), :, 3].T
                p_shift = np.c_[p[:, 1:], np.zeros((3, 1))]
                diff_vec = p_shift - p
                diff_vec = diff_vec[:, :-1]
                length = np.sum(np.sqrt(np.sum(diff_vec ** 2, axis=0)))

                # Find necessary scaling factor:
                s = length / length_est
                scales[i] = s

                # Find translation vector for alignment at pose s (current pose)
                t = gt_T_wc[pos, :, 3] - s * R_dash_s @ T_wc[pos, :, 3]

                p_hat_dash = s * R_dash_s @ p_hat + t.reshape(3, 1)

            else:
                # STEREO / INERTIAL: SCALE = 1

                # Find translation vector for alignment at pose s (current pose)
                t = gt_T_wc[pos, :, 3] - R_dash_s @ T_wc[pos, :, 3]

                p_hat_dash = R_dash_s @ p_hat + t.reshape(3, 1)

            pos_err[i] = np.linalg.norm(gt_T_wc[next_pos, :, 3] - R_k @ p_hat_dash[:, -1])

            # Keep the transformed positions for visualisation later
            split_trajec.append(p_hat_dash)
        
        return pos_err, rot_err, scales, split_trajec, split_pos


    def relative_error(self, trajec_lenghts=(1, 2, 3, 4, 5)):
        """
        Compute and visualise the relative error measures for several subtrajectory 
        lengths.

        ### Parameters
        1. trajec_lengths: tuple
            - The subtrajectory lengths in meters for which the relative error and its
            statistics should be computed. 
            - For the last length: aligned sub-trajectories visualised in a 3D plot. 
            - The first of these provides the information for the scale-drift plot
        """

        n_lengths = len(trajec_lenghts)

        pos_errs = []
        rot_errs = []
        scale_drifts = []
        split_trajecs = []
        split_pos_s = []

        for i in range(n_lengths):

            for_length_i = self._get_relative_error(trajec_lenghts[i])
            pos_err, rot_err, scale_drift, split_trajec, split_pos = for_length_i
            pos_errs.append(pos_err)
            rot_errs.append(rot_err)
            scale_drifts.append(scale_drift)
            split_trajecs.append(split_trajec)
            split_pos_s.append(split_pos)

        # ---------- STATISTICS PLOT ---------- #

        mosaic_struc = np.array([["box_pos"],
                                 ["box_rot"],
                                 ["scale"]])

        fig1, axs = plt.subplot_mosaic(mosaic_struc, layout="constrained",
                                       figsize=(6, 7))
        
        axs["box_pos"].boxplot(pos_errs)
        axs["box_pos"].set_title("Translation error")
        axs["box_pos"].set_xlabel("Subtrajectory length [m]")
        axs["box_pos"].set_ylabel("Translation error [m]")
        axs["box_pos"].set_xticklabels([str(i) for i in trajec_lenghts])

        axs["box_rot"].boxplot(rot_errs)
        axs["box_rot"].set_title("Rotation error")
        axs["box_rot"].set_xlabel("Subtrajectory length [m]")
        axs["box_rot"].set_ylabel("Rotation error [deg]")
        axs["box_rot"].set_xticklabels([str(i) for i in trajec_lenghts])

        # For analysing scale drift: use the smallest sub-trajectory length
        min_length_idx = np.argmin(trajec_lenghts)

        dist_travelled = [trajec_lenghts[min_length_idx] * i 
                          for i in range(len(scale_drifts[min_length_idx]))]

        # Plot scale correction factor for each consecutive sub-trajectory
        axs["scale"].plot(dist_travelled, scale_drifts[min_length_idx])
        axs["scale"].set_title("Scale Drift")
        axs["scale"].set_xlabel("Lenght along the total trajectory [m]")
        axs["scale"].set_ylabel("Scaling factor wrt. ground truth")
        axs["scale"].grid(True)

        # ---------- PLOT 3D ---------- #
        
        # The split trajectories that are plotted are the ones of the length
        # that is passed in the last position of the trajec_lengths vector

        gt_w_t_wc__x = self.gt_T_wc_array[0::3, 3]
        gt_w_t_wc__y = self.gt_T_wc_array[1::3, 3]
        gt_w_t_wc__z = self.gt_T_wc_array[2::3, 3]

        fig1 = plt.figure()
        ax = fig1.add_subplot(projection="3d")

        # plot ground truth
        ax.plot(gt_w_t_wc__x, gt_w_t_wc__y, gt_w_t_wc__z, color="purple")

        # Plot each split part of the estimated trajectory
        for i in range(len(split_pos) - 1): # split_pos is the last element of split_pos_s
            ax.plot(split_trajecs[-1][i][0, :], 
                    split_trajecs[-1][i][1, :], 
                    split_trajecs[-1][i][2, :], color="orange")
            ax.plot(split_trajecs[-1][i][0, 0], 
                    split_trajecs[-1][i][1, 0], 
                    split_trajecs[-1][i][2, 0], "go", markersize=3)
        ax.set_xlabel("$X_w$")
        ax.set_ylabel("$Y_w$")
        ax.set_zlabel("$Z_w$")
        ax.set_title("Relative Trajectory Error")
        ax.legend(["Ground truth", "Estimate", "Subtrajectory"])

        # Compute the limits for the plot; same scale for both axes
        xmin = np.min(gt_w_t_wc__x) - 1
        xmax = np.max(gt_w_t_wc__x) + 1
        ymin = np.min(gt_w_t_wc__y) - 1
        ymax = np.max(gt_w_t_wc__y) + 1
        zmin = np.min(gt_w_t_wc__z) - 1
        zmax = np.max(gt_w_t_wc__z) + 1

        max_range = max(xmax - xmin, zmax - zmin, ymax - ymin)
        mid_x = 0.5 * (xmin + xmax)
        mid_z = 0.5 * (zmin + zmax)
        mid_y = 0.5 * (ymin + ymax)

        # Set the computed limits
        ax.set_xlim(mid_x - 0.5 * max_range, mid_x + 0.5 * max_range)
        ax.set_ylim(mid_y - 0.5 * max_range, mid_y + 0.5 * max_range)
        ax.set_zlim(mid_z - 0.5 * max_range, mid_z + 0.5 * max_range)

        # Have the camera view be the orientation of the world coord sys.
        ax.view_init(elev=-80, azim=-90, roll=0)

        plt.show()

    def re_get_split_pos(self, trajectory_length: float):
        """
        Get the locations in the ground truth and the estimate trajectories at which 
        they should be split into sub-trajectories. A split is made where the trans-
        lation vectors of the ground truth add to trajectory_length. 
        Also computes the total length of the ground truth trajectory and sets 
        self.gt_length

        ### Parameters
        1. trajectory_lenght : float
            - Minimum length of each sub-trajectory in meters

        ### Returns
        1. split_pos : list
            - Contains indices such that the distance travelled between the
            corresponding frames is (at least) the trajectory_length
        """

        split_pos = [0]
        travelled = 0
        tot_travelled = 0
        previous_t = self.gt_T_wc_list[0][:, 3]

        for pos, gt_T_wc in enumerate(self.gt_T_wc_list):
            
            # The difference in translation compared to the previous pose
            travel_vec = gt_T_wc[:, 3] - previous_t

            # Increment the distance counter by the L2 norm of the translation
            travelled += np.sqrt(np.sum(travel_vec**2))

            if travelled >= trajectory_length:
                split_pos.append(pos)
                tot_travelled += travelled
                travelled = 0
            
            previous_t = gt_T_wc[:, 3]

        tot_travelled += travelled

        self.gt_length = tot_travelled

        return split_pos

    def _draw_orientation(self, ax, gt: bool, frame_idx: int):
        """
        Draw the orientation axes at a given frame index

        ### Parameters
        1. ax : matplotlib axis
            - The axis on which to draw
        2. gt : bool
            - Whether to draw the ground truth orientation (True) or the estimated one (False)
        3. frame_idx : int
            - The frame index at which to draw the orientation axes
        """

        if gt:
            T_wc = self.gt_T_wc_list[frame_idx]
        else:
            T_wc = self.T_wc_list[frame_idx]

        # Where the arrows will be put
        origin = T_wc[:, 3]
        c_X = T_wc[:, 0]
        c_Y = T_wc[:, 1]
        c_Z = T_wc[:, 2]

        ax.quiver(origin[0], origin[1], origin[2],
                  c_X[0], c_X[1], c_X[2],
                  color="r", length=0.5, arrow_length_ratio=0.1)
        ax.quiver(origin[0], origin[1], origin[2],
                  c_Y[0], c_Y[1], c_Y[2],
                  color="g", length=0.5, arrow_length_ratio=0.1)
        ax.quiver(origin[0], origin[1], origin[2],
                  c_Z[0], c_Z[1], c_Z[2],
                  color="b", length=0.5, arrow_length_ratio=0.1)
    
    @staticmethod
    def time_align(trajec: np.array, gt: np.array, threshold: float) -> tuple:
        """
        Align the estimated pose trajectory and the the ground truth poses using
        the timestamps in the first column. Implements a threshold for minimum
        distance in time, otherwise no match is found

        ###Parameters
        1. trajec : np.array
            - (n_traj, 8) array of estimated poses. First column time stamp
        2. gt : np.array
            - (n_gt, 8) array of ground truth poses. First column time stamp
        3. threshold : float
            - Minimum distance in time for a match
        
        ###Returns:
        1. cut_trajec : np.array
            - (n_poses, 8) array of cut estimated poses that do have a corresponding
            ground thruth within threshold
        2. cut_gt : np.array
            - (n_poses, 8) array of cut gt poses that do have a corresponding
            estimated pose within threshold
        """
        
        trajec_ts = trajec[:, 0]
        gt_ts = gt[:, 0]
        
        # Find insertion positions
        pos = np.searchsorted(gt_ts, trajec_ts)
        
        # Candidates: left and right neighbors
        left = np.clip(pos - 1, 0, len(gt_ts) - 1)
        right = np.clip(pos, 0, len(gt_ts) - 1)
        
        # Compute differences
        left_diff = np.abs(gt_ts[left] - trajec_ts)
        right_diff = np.abs(gt_ts[right] - trajec_ts)
        
        # Pick closer one
        use_right = right_diff < left_diff
        closest_sorted_idx = np.where(use_right, right, left)
        closest_diff = np.where(use_right, right_diff, left_diff)
        
        # Cut trajectory entries that do not have a gt within threshold
        mask = closest_diff <= threshold

        # Estimated poses without corresponding ground truth are not useful
        cut_trajec = trajec[mask]

        # Cut also the ground truth to those poses that have a corresponding
        # estimated poses
        cut_closest_sorted_idx = closest_sorted_idx[mask]
        cut_gt = gt[cut_closest_sorted_idx]

        return cut_trajec, cut_gt


if __name__ == "__main__":

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
    # te.similarity_transform_3d(align_all_frames=True)
    # te.draw_trajectory(gt=True, add_orientation_est=0, add_orientation_gt=0)
    te.relative_error(trajec_lenghts=(1, 0.7, 0.5))
    ate = te.absolue_trajectory_error()
    print(f"{ate[0]:.3f}m -- ATE position error")
    print(f"{ate[1]:.2f}° -- ATE rotation error")
    print(f"{(te.frac_gt_used * 100):.1f}% -- Percentage of GT poses used" )
    print(f"GT length: {te.gt_length}")
    # te.relative_error(trajec_lenghts=(2, 5, 10))

