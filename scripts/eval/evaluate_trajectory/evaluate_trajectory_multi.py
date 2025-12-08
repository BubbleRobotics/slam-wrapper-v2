"""
Evaluate multiple trajectories against one or several ground truth
trajectories.

Implemented by clandsmeer
"""

from .evaluate_trajectory_single import TrajectoryEval
from omegaconf import DictConfig
from pathlib import Path
from collections import namedtuple
import pandas as pd
import numpy as np

class TrajectoryEvalMulti:
    
    def __init__(self, cfg: DictConfig):

        self.cfg = cfg
        self.evaluators = []

        Evaluator = namedtuple(
            "Evaluator", 
            ["TrajectoryEval", "seq_name", "pipeline_type", "plot_dir"]
            )
        
        # Where plots will be saved
        self.plot_dir = Path(cfg.output_subdir.plots)
        self.plot_dir.mkdir(exist_ok=True)

        # Where RTE trajectory errors will be saved
        self.data_dir = Path(cfg.output_subdir.data)
        self.data_dir.mkdir(exist_ok=True)
        
        self.rte_df = None # pd.DataFrame initialised later
        self.ate_df = None

        # ------ INITIALISE TrajectoryEval OBJECTS ------ #

        for seq_name, seq in self.cfg.evals.items():
            plot_seq_dir = self.plot_dir.joinpath(seq_name)
            
            for pipeline_type_name, pipeline_type in seq.run.items():
                plot_dir = plot_seq_dir.joinpath(pipeline_type_name)

                for pipeline_run in Path(pipeline_type.dir).glob("*.txt"):

                    if self.cfg.eval_setting == "adaptive":
                        eval_setting = pipeline_type_name
                    else:
                        eval_setting = self.cfg.eval_setting

                    eval_obj = TrajectoryEval(
                        odometry_path=pipeline_run,
                        gt_path=Path(seq.gt),
                        sensor_config=eval_setting
                        )
                    
                    evaluator = Evaluator(
                        TrajectoryEval=eval_obj,
                        seq_name=seq_name,
                        pipeline_type=pipeline_type_name,
                        plot_dir=plot_dir
                        )
                    
                    self.evaluators.append(evaluator)
    

    def do_analysis(self):
        """
        Master method calling the relative_error() method and 
        absolute_trajectory_error() method depending on config settings
        """

        if self.cfg.rte.do_analysis:
            self.relative_error()
        
        if self.cfg.ate.do_analysis:
            self.align()
            if self.cfg.ate.draw_trajec.show:
                self.draw_trajectory()
            self.absolute_error()


    def relative_error(self):
        """
        Call .relative_error() method on TrajectoryEval objects
        """

        for evaluator in self.evaluators:
            
            # Check if the current estimated trajectory is considered a
            # failure. If so, the relative error is not evaluated for it.
            current_trajec = evaluator.TrajectoryEval.odometry_path.stem
            failures = self.cfg.evals[evaluator.seq_name].run.stereo.failures
            if current_trajec in failures:
                continue

            ### ---------- PLOTTING ---------- ###
            
            # Create plot subtrajectories if plots should be saved
            if self.cfg.rte.save.stats or self.cfg.rte.save.subtrajec:
                evaluator.plot_dir.mkdir(exist_ok=True, parents=True)

            # If the boxplot should be saved
            if self.cfg.rte.save.stats:
                path_stat_plot = ("rte_statistics_plot_" 
                                  + evaluator.TrajectoryEval.odometry_path.stem 
                                  + ".png")
                path_stat_plot = evaluator.plot_dir.joinpath(path_stat_plot)
            else:
                path_stat_plot = None

            # If the subtrajectory plot should be saved
            if self.cfg.rte.save.subtrajec:
                path_subtraj_plot = ("rte_subtrajectory_plot_" 
                                        + evaluator.TrajectoryEval.odometry_path.stem 
                                        + ".png")
                path_subtraj_plot = evaluator.plot_dir.joinpath(path_subtraj_plot)
            else:
                path_subtraj_plot = None
            
            # Print information on the specific trajectory being displayed
            if self.cfg.rte.show:
                print(f"Sequence: \033[1m\033[96m{evaluator.seq_name}\033[0m, "
                      + f"pipeline type: \033[1m\033[96m{evaluator.pipeline_type}\033[0m, "
                      + f"eval_setting: \033[1m\033[96m{evaluator.TrajectoryEval.sensor_config}"
                      + "\033[0m, trajectory_file: \033[1m\033[96m" 
                      + f"{evaluator.TrajectoryEval.odometry_path.stem}.txt\033[0m")
            
            # Call the .relative_error() method of the TrajectoryEval object

            rel_err = evaluator.TrajectoryEval.relative_error(
                    trajec_lenghts=self.cfg.rte.trajec_lengths,
                    show=self.cfg.rte.show,
                    save=[path_stat_plot, path_subtraj_plot]
                    )
            
            ### ---------- SAVING RTE DATA ---------- ###

            def fill_in_pos_err(row, rel_err):
                """Function applied to df to fill in error data from
                different pipeline runs"""
                idx = self.cfg.rte.trajec_lengths.index(row["subtraj_len"])
                pos_err = row["pos_err"]
                pos_err = np.concatenate([pos_err, rel_err[idx].pos])
                return pos_err
            
            def fill_in_rot_err(row, rel_err):
                """Function applied to df to fill in error data from
                different pipeline runs"""
                
                idx = self.cfg.rte.trajec_lengths.index(row["subtraj_len"])
                rot_err = row["rot_err"]
                rot_err = np.concatenate([rot_err, rel_err[idx].rot])
                return rot_err

            if self.rte_df is None or not (self.rte_df["gt_name"]==evaluator.seq_name).any():
                
                new_rows = []

                for i, tup in enumerate(rel_err):

                    # Form new rows: position and rotation error,
                    # one row for each trajecotry length
                    new_rows.append(
                        {"gt_name": evaluator.seq_name,
                         "pipeline_type": evaluator.pipeline_type,
                         "eval_setting": evaluator.TrajectoryEval.sensor_config,
                         "subtraj_len": self.cfg.rte.trajec_lengths[i],
                         "pos_err": tup.pos,
                         "rot_err": tup.rot})
                    
                # Append the new rows to the dataframe
                if self.rte_df is not None:
                    self.rte_df = pd.concat([self.rte_df, pd.DataFrame(new_rows)], 
                                            ignore_index=True)
                else:
                    self.rte_df = pd.DataFrame(new_rows)
                
            else:

                # Rows representing this sequence exist already

                self.rte_df.loc[self.rte_df["gt_name"]==evaluator.seq_name, "pos_err"] \
                    = self.rte_df.loc[self.rte_df["gt_name"]==evaluator.seq_name].apply(
                        fill_in_pos_err, args=(rel_err,),axis=1)
                
                self.rte_df.loc[self.rte_df["gt_name"]==evaluator.seq_name, "rot_err"] \
                    = self.rte_df.loc[self.rte_df["gt_name"]==evaluator.seq_name].apply(
                        fill_in_rot_err, args=(rel_err,), axis=1)

        self.rte_df.to_pickle(self.data_dir.joinpath("rte.pkl"))

    def align(self):

        for evaluator in self.evaluators:
            evaluator.TrajectoryEval.align(
                align_all_frames=self.cfg.ate.align_all_frames
            )
    
    def draw_trajectory(self):

        for evaluator in self.evaluators:

            # Check if the current estimated trajectory is considered a
            # failure. If so, the relative error is not evaluated for it.
            current_trajec = evaluator.TrajectoryEval.odometry_path.stem
            failures = self.cfg.evals[evaluator.seq_name].run.stereo.failures
            if current_trajec in failures:
                continue

            print(f"Sequence: \033[1m\033[96m{evaluator.seq_name}\033[0m, "
                  + f"pipeline type: \033[1m\033[96m{evaluator.pipeline_type}\033[0m, "
                  + f"eval_setting: \033[1m\033[96m{evaluator.TrajectoryEval.sensor_config}"
                  + "\033[0m, trajectory_file: \033[1m\033[96m" 
                  + f"{evaluator.TrajectoryEval.odometry_path.stem}.txt\033[0m")
            evaluator.TrajectoryEval.draw_trajectory(
                gt=self.cfg.ate.draw_trajec.gt,
                add_orientation_gt=self.cfg.ate.draw_trajec.add_orientation_gt,
                add_orientation_est=self.cfg.ate.draw_trajec.add_orientation_est
            )

    def absolute_error(self):

        if self.cfg.ate.align_all_frames:
            alignment_type = "all frames"
        else:
            alignment_type = "first frame"
        
        for evaluator in self.evaluators:

            abs_err = evaluator.TrajectoryEval.absolute_trajectory_error()
            
            def fill_in_pos_err(row, abs_err):
                """Function applied to df to fill in error data from
                different pipeline runs"""
                pos_err = row["pos_err"]
                pos_err = np.concatenate([pos_err, abs_err.pos], axis=1)
                return pos_err
            
            def fill_in_rot_err(row, abs_err):
                """Function applied to df to fill in error data from
                different pipeline runs"""
                rot_err = row["rot_err"]
                rot_err = np.concatenate([rot_err, abs_err.rot])
                return rot_err

            if self.ate_df is None or not (self.ate_df["gt_name"]==evaluator.seq_name).any():
                
                new_row={"gt_name": evaluator.seq_name,
                         "pipeline_type": evaluator.pipeline_type,
                         "eval_setting": evaluator.TrajectoryEval.sensor_config,
                         "alignment_type": alignment_type,
                         "pos_err": abs_err.pos,
                         "rot_err": abs_err.rot}
                
                # Append the new rows to the dataframe
                if self.ate_df is not None:
                    self.ate_df.loc[len(self.ate_df)] = new_row
                else:
                    self.ate_df = pd.DataFrame([new_row])

            else:
                # Rows representing this sequence exist already 

                self.ate_df.loc[self.ate_df["gt_name"]==evaluator.seq_name, "pos_err"] \
                    = self.ate_df.loc[self.ate_df["gt_name"]==evaluator.seq_name].apply(
                        fill_in_pos_err, args=(abs_err,),axis=1)
                
                self.ate_df.loc[self.ate_df["gt_name"]==evaluator.seq_name, "rot_err"] \
                    = self.ate_df.loc[self.ate_df["gt_name"]==evaluator.seq_name].apply(
                        fill_in_rot_err, args=(abs_err,), axis=1)
        
        self.ate_df.to_pickle(self.data_dir.joinpath("ate.pkl"))
