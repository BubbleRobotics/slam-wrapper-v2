"""
Evaluate multiple trajectories against one or several ground truth
trajectories.

Implemented by clandsmeer
"""

from .evaluate_trajectory_single import TrajectoryEval
from omegaconf import DictConfig
from pathlib import Path
import numpy as np

class TrajectoryEvalMulti:
    
    def __init__(self, cfg: DictConfig):

        # Evaluate mono, stereo, inertial, etc.
        self.eval_setting = cfg.eval_setting
        
        # Where plots will be saved
        self.plot_dir = Path(cfg.output_subdir.plots)
        self.plot_dir.mkdir(exist_ok=True)

        # ------ INITIALISE TrajectoryEval OBJECTS ------ #

        for seq_name, seq in cfg.evals.items():
            
            plot_seq_dir = self.plot_dir.joinpath(seq_name)
            plot_seq_dir.mkdir(exist_ok=True)
            
            for pipeline_type, pipeline_dir in seq.run.items():
                
                plot_seq_type_dir = plot_seq_dir.joinpath(pipeline_type)
                plot_seq_type_dir.mkdir(exist_ok=True)

                for pipeline_run in Path(pipeline_dir).glob("*.txt"):
                    
                    evaluator = None

                    evaluator = TrajectoryEval(
                        odometry_path=pipeline_run,
                        gt_path=Path(seq.gt),
                        sensor_config=cfg.eval_setting
                        )
                    
                    evaluator.relative_error(
                        trajec_lenghts=cfg.rte.trajec_lengths,
                        show=cfg.rte.show,
                        save=[
                            plot_seq_type_dir.joinpath(
                                "statistics_plot_" + pipeline_run.stem + ".png"
                                ), 
                            plot_seq_type_dir.joinpath(
                                "subtrajectory_plot_" + pipeline_run.stem + ".png"
                                ),
                            ]
                        )
                    
                    # evaluator.align(align_all_frames=cfg.align_all_frames)



        

        