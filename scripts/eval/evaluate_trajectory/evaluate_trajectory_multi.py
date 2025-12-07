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

        Evaluator = namedtuple(
            "Evaluator", 
            ["TrajectoryEval", "seq_name", "pipeline_type", "plot_dir"]
            )

        # Evaluate mono, stereo, inertial, etc.
        self.eval_setting = cfg.eval_setting
        
        # Where plots will be saved
        self.plot_dir = Path(cfg.output_subdir.plots)
        self.plot_dir.mkdir(exist_ok=True)

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

        for evaluator in self.evaluators:
            
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
            evaluator.TrajectoryEval.relative_error(
                trajec_lenghts=self.cfg.rte.trajec_lengths,
                show=self.cfg.rte.show,
                save=[path_stat_plot, path_subtraj_plot]
                )



        