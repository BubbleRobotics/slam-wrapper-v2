from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def clean_outliers_iqr(df):
    """
    Modifieds dataframe where pos_err and rot_err columns of numpy arrays
    have been cleaned using the 1.5*IQR outlier rule.
    All other columns are copied unchanged.
    """

    def _remove_outliers(arr):
        arr = np.asarray(arr)

        if arr.size == 0:
            return arr

        # Quartiles
        q1 = np.percentile(arr, 25)
        q3 = np.percentile(arr, 75)
        iqr = q3 - q1

        # Outlier thresholds
        lower = q1 - 1.5 * iqr
        upper = q3 + 1.5 * iqr

        # Keep only non-outliers
        return arr[(arr >= lower) & (arr <= upper)]

    # # Copy the dataframe to avoid modifying the original
    # df_new = df.copy()

    # Apply outlier removal row-by-row
    df["pos_no_err"] = df["pos_err"].apply(_remove_outliers)
    df["rot_no_err"] = df["rot_err"].apply(_remove_outliers)


def add_median(df):
    """
    Modifies the df, adding one rows for the medians
    """
    df["pos_median"] = df["pos_err"].apply(lambda arr: np.median(arr))
    df["rot_median"] = df["rot_err"].apply(lambda arr: np.median(arr))
    
    if "rot_no_err" in df.columns:
        df["rot_no_median"] = df["rot_no_err"].apply(lambda arr: np.median(arr))
    
    if "pos_no_err" in df.columns:
        df["pos_no_median"] = df["pos_no_err"].apply(lambda arr: np.median(arr))


def compare_histograms(df_si, df_sid, mode, bins=30):
    """
    For each row index shared by df1 and df2, create a figure with an Axes object
    and plot two histograms (one from each dataframe) with median markers.
    ### Params:
    1. df_si: pandas dataframe
    2. df_sid: pandaas dataframe
    3. mode: str
        - one of ["rot", "pos", "rot_no", "pos_no"]
    """
    assert mode in ["rot", "pos", "rot_no", "pos_no"]

    n = min(len(df_si), len(df_sid))

    for i in range(n):
        arr_si = df_si.loc[i, mode + "_err"]
        arr_sid = df_sid.loc[i, mode + "_err"]

        # Skip invalid or empty arrays
        if arr_si is None or arr_sid is None:
            continue
        if len(arr_si) == 0 or len(arr_sid) == 0:
            continue

        med1 = df_si.loc[i, mode + "_median"]
        med2 = df_sid.loc[i, mode + "_median"]

        std1 = np.std(arr_si)
        std2 = np.std(arr_sid)

        # Create figure + axis (OO style)
        fig, ax = plt.subplots(figsize=(8, 5))

        row_name = df_si.loc[i, "gt_name"] + " lenght " + str(df_si.loc[i, "subtraj_len"])

        # Histogram for df1
        ax.hist(arr_si, bins=bins, alpha=0.5, color="blue", label="SI " + row_name)
        ax.axvline(med1, color="blue", linestyle="--", linewidth=2,
                   label=f"SI median = {med1:.3f}")

        # Histogram for df2
        row_name = df_sid.loc[i, "gt_name"] + " lenght " + str(df_si.loc[i, "subtraj_len"])
        ax.hist(arr_sid, bins=bins, alpha=0.5, color="orange", label="SID " + row_name)
        ax.axvline(med2, color="orange", linestyle="--", linewidth=2,
                   label=f"SID median = {med2:.3f}")
        # ax.set_xlim((0, max(std1, std2)))

        # Axis labels and title
        ax.set_title(mode + f" Histogram comparison for " + row_name)
        ax.set_xlabel("Value")
        ax.set_ylabel("Frequency")
        ax.legend()

        fig.tight_layout()
        plt.show()


def compare_gt_use_histograms(df_si, df_sid, dvlcov):

    sequences = set(df_si["gt_name"]).intersection(set(df_sid["gt_name"]))

    for sequence in sequences:
        
        si_frac_gt_used = df_si.loc[df_si["gt_name"]==sequence, "frac_gt_used"].iloc[0]
        sid_frac_gt_used = df_sid.loc[df_sid["gt_name"]==sequence, "frac_gt_used"].iloc[0]

        # Convert to percent
        si_frac_gt_used *= 100
        sid_frac_gt_used *= 100

        m_si = si_frac_gt_used.mean()
        m_sid = sid_frac_gt_used.mean()

        # ----- PLOTTING ----- #

        # Create figure + axis (OO style)
        fig, ax = plt.subplots(figsize=(8, 5))

        title = f"Percent Ground Truth Used in {sequence} with DVL Cov {dvlcov}"

        # Specify the bins such that they are evenly spaced
        bins = np.linspace(0, 100, 11)

        # Histogram for SI
        ax.hist(si_frac_gt_used, bins=bins, alpha=0.5, color="blue", label="SI")
        ax.axvline(m_si, color="blue", linestyle="--", linewidth=2,
                   label=f"SI mean {m_si:.3f} over 10 runs")

        # Histogram for SID
        ax.hist(sid_frac_gt_used, bins=bins, alpha=0.5, color="orange", label="SID")
        ax.axvline(m_sid, color="orange", linestyle="--", linewidth=2,
                   label=f"SID mean {m_sid:.3f} over 10 runs")
        
        ax.set_title(title)
        ax.set_xlabel("Value [%]")
        ax.set_ylabel("Frequency")
        ax.legend()

        fig.tight_layout()
        plt.show()


if __name__ == "__main__":

    DVL_COV = 10.0

    string_conversion = str(DVL_COV).replace(".", "p")

    dir_sid = Path(f"/home/ubuntu/ws_blue/data/eval_output/2026-02-01/sidm_dvlcov{string_conversion}_noWholeTank/error_data")
    dir_si = Path("/home/ubuntu/ws_blue/data/eval_output/2026-02-01/si_noWholeTank/error_data")

    rte_sid = pd.read_pickle(dir_sid.joinpath("rte.pkl"))
    rte_si = pd.read_pickle(dir_si.joinpath("rte.pkl"))

    # Outlier removed versions
    clean_outliers_iqr(rte_sid)
    clean_outliers_iqr(rte_si)

    # Compute the medians
    add_median(rte_si)
    add_median(rte_sid)

    compare_histograms(rte_si, rte_sid, "rot_no", 50)

    # compare_gt_use_histograms(rte_si, rte_sid, 10)
