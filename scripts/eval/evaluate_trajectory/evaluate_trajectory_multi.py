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

        # Evaluate mono, stereo, inertial, etc.
        self.eval_setting = cfg.eval_setting
        
        # Where plots will be saved
        self.plot_dir = Path(cfg.output_subdir.plots)
        self.plot_dir.mkdir(exist_ok=True)
        self.data_dir = Path(cfg.output_subdir.data)
        self.data_dir.mkdir(exist_ok=True)

        # ------ INITIALISE TrajectoryEval OBJECTS ------ #

        for seq_name, seq in self.cfg.evals.items():
            plot_seq_dir = self.plot_dir.joinpath(seq_name)
            
            for pipeline_type_name, pipeline_type in seq.run.items():
                plot_dir = plot_seq_dir.joinpath(pipeline_type_name)

                for pipeline_run in Path(pipeline_type.dir).glob("*.txt"):

                    eval_obj = TrajectoryEval(
                        odometry_path=pipeline_run,
                        gt_path=Path(seq.gt),
                        sensor_config=cfg.eval_setting
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


    def relative_error(self):
        """
        Call .relative_error() method on TrajectoryEval objects
        """
        rte_df = None  # pd.DataFrame initialised later

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
                path_stat_plot = ("statistics_plot_" 
                                    + evaluator.TrajectoryEval.odometry_path.stem 
                                    + ".png")
                path_stat_plot = evaluator.plot_dir.joinpath(path_stat_plot)
            else:
                path_stat_plot = None

            # If the subtrajectory plot should be saved
            if self.cfg.rte.save.subtrajec:
                path_subtraj_plot = ("subtrajectory_plot_" 
                                        + evaluator.TrajectoryEval.odometry_path.stem 
                                        + ".png")
                path_subtraj_plot = evaluator.plot_dir.joinpath(path_subtraj_plot)
            else:
                path_subtraj_plot = None
            
            # Print information on the specific trajectory being displayed
            if self.cfg.rte.show:
                print(f"Showing {evaluator.seq_name}, {evaluator.pipeline_type}, " 
                      + f"{evaluator.TrajectoryEval.odometry_path.stem}.txt")
            
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

            if rte_df is None or not (rte_df["gt_name"]==evaluator.seq_name).any():
                
                new_rows = []

                for i, tup in enumerate(rel_err):

                    # Form new rows: for position and rotation error,
                    # one row for each trajecotry length
                    new_rows.append(
                        {"gt_name": evaluator.seq_name,
                         "pipeline_type": evaluator.pipeline_type,
                         "eval_setting": self.eval_setting,
                         "subtraj_len": self.cfg.rte.trajec_lengths[i],
                         "pos_err": tup.pos,
                         "rot_err": tup.rot})
                    
                # Append the new rows to the dataframe
                if rte_df is not None:
                    rte_df = pd.concat([rte_df, pd.DataFrame(new_rows)], ignore_index=True)
                else:
                    rte_df = pd.DataFrame(new_rows)
                
            else:

                # Rows representing this sequence exist already

                rte_df.loc[rte_df["gt_name"]==evaluator.seq_name, "pos_err"] \
                    = rte_df.loc[rte_df["gt_name"]==evaluator.seq_name].apply(
                        fill_in_pos_err, args=(rel_err,),axis=1)
                
                rte_df.loc[rte_df["gt_name"]==evaluator.seq_name, "rot_err"] \
                    = rte_df.loc[rte_df["gt_name"]==evaluator.seq_name].apply(
                        fill_in_rot_err, args=(rel_err,), axis=1)

            rte_df.to_pickle(self.data_dir.joinpath("rte.pkl"))
