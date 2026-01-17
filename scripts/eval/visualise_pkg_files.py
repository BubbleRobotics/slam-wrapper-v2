from pathlib import Path
import numpy as np
import pandas as pd

DIR = Path("/home/ubuntu/ws_blue/data/eval_output/2026-01-17/18-17-47/error_data")

rte_df = pd.read_pickle(DIR.joinpath("rte.pkl"))
ate_df = pd.read_pickle(DIR.joinpath("ate.pkl"))

breakpoint()