"""
Evaluate multiple trajectories against one or several ground truth
trajectories.

Implemented by clandsmeer
"""

from .evaluate_trajectory_single import TrajectoryEval
from omegaconf import DictConfig
from pathlib import Path
from collections import namedtuple

class TrajectoryEvalMulti:
    
    def __init__(self, cfg: DictConfig):

        self.cfg = cfg
        self.evaluators = []

        Evaluator = namedtuple("Evaluator", ["TrajectoryEval", "plot_dir"])

        # Evaluate mono, stereo, inertial, etc.
        self.eval_setting = cfg.eval_setting
        
        # Where plots will be saved
        self.plot_dir = Path(cfg.output_subdir.plots)
        self.plot_dir.mkdir(exist_ok=True)

        # ------ INITIALISE TrajectoryEval OBJECTS ------ #

        for seq_name, seq in self.cfg.evals.items():
            
            plot_seq_dir = self.plot_dir.joinpath(seq_name)
            # plot_seq_dir.mkdir(exist_ok=True)
            
            for pipeline_type, pipeline_dir in seq.run.items():
                
                plot_dir = plot_seq_dir.joinpath(pipeline_type)
                # plot_seq_type_dir.mkdir(exist_ok=True)

                for pipeline_run in Path(pipeline_dir).glob("*.txt"):

                    eval_obj = TrajectoryEval(
                        odometry_path=pipeline_run,
                        gt_path=Path(seq.gt),
                        sensor_config=cfg.eval_setting
                        )
                    
                    evaluator = Evaluator(
                        TrajectoryEval=eval_obj,
                        plot_dir=plot_dir
                        )
                    
                    self.evaluators.append(evaluator)
                    
    def relative_error(self):
        """
        Call .relative_error() method on TrajectoryEval objects
        """

        for evaluator in self.evaluators:

            evaluator.plot_dir.mkdir(exist_ok=True, parents=True)

            path_stat_plot = ("statistics_plot_" 
                                + evaluator.TrajectoryEval.odometry_path.stem 
                                + ".png")
            path_stat_plot = evaluator.plot_dir.joinpath(path_stat_plot)

            path_subtraj_plot = ("subtrajectory_plot_" 
                                    + evaluator.TrajectoryEval.odometry_path.stem 
                                    + ".png")
            path_subtraj_plot = evaluator.plot_dir.joinpath(path_subtraj_plot)
            
            evaluator.TrajectoryEval.relative_error(
                trajec_lenghts=self.cfg.rte.trajec_lengths,
                show=self.cfg.rte.show,
                save=[path_stat_plot, path_subtraj_plot]
                )




        

        