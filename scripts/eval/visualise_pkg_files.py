import collections.abc
from collections import namedtuple
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Patch


# No longer used. Here, using the IQR rule will skew the results
def clean_outliers_iqr(df):
    """
    Modifieds dataframe, two new columns are added using the 1.5*IQR outlier rule,
    based on the rot_err and pos_err columns. Only outliers above are cut since these
    likely correspond to pipeline resets
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
        return arr[arr <= upper]

    # Apply outlier removal row-by-row
    df["pos_no_err"] = df["pos_err"].apply(_remove_outliers)
    df["rot_no_err"] = df["rot_err"].apply(_remove_outliers)


def clean_outliers_threshold(df):
    """
    Modifies dataframe, adding columns pos_err_no and rot_err_no based on
    the columns pos_err and rot_err respectively
    Rule for removing outliers: if the error is greater than the subtrajectory
    length, this is almost certainly a reset.
    """

    # Function applied to df
    def _remove_outliers(row, to_correct):
        subtraj_len = row["subtraj_len"]
        no_outlier = (row["pos_err"] <= subtraj_len)
        if to_correct == "pos":
            return row["pos_err"][no_outlier]
        else:
            return row["rot_err"][no_outlier]
        

    df["pos_no_err"] = df.apply(_remove_outliers, args=("pos",), axis=1)
    df["rot_no_err"] = df.apply(_remove_outliers, args=("rot",), axis=1)



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


def compare_histograms(df_si, df_sid, mode, bins=30, save=None):
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

    if mode == "rot_no" or "pos_no":
        clean_outliers_threshold(df_si)
        clean_outliers_threshold(df_sid)
    
    if "pos_median" not in df_si.columns:
        add_median(df_si)

    if "pos_median" not in df_sid.columns:
        add_median(df_sid)

    # ----- PLOTTING ----- #

    sequence_dic = {
        "Structure_Easy": "se",
        "Structure_Medium": "sm",
        "Structure_Hard": "sh",
        "HalfTank_Easy": "he",
        "HalfTank_Medium": "hm",
        "HalfTank_Hard": "hh",
    }

    plt.rcParams['font.family'] = 'serif'

    for i in range(n):

        # Iterate through df_si and match the rows in df_sid based on subtraj_len and gt_name
        # Within the eval scripts, the indexing on the subtrajectory lengths was certainly consistent,
        # but since now different eval outputs are compared, we have to make sure that we match the 
        # same sequences and subtrajectory lengths
        curr_subtraj_len = df_si.iloc[i]["subtraj_len"]
        curr_gt_name = df_si.iloc[i]["gt_name"]

        if not (df_sid["gt_name"]==curr_gt_name).any():
            # If this triggers, df_sid frame does not contain this gt_name
            continue
        else:
            # What remains in df_sid when filtering for the current gt_name
            df_sid_filtered = df_sid[df_sid["gt_name"]==curr_gt_name]
            if not (df_sid_filtered["subtraj_len"]==curr_subtraj_len).any():
                # If this triggers, the df_sid_filtered does not containt this subtraj_len
                continue
            else:
                matched_row = df_sid_filtered[df_sid_filtered["subtraj_len"]==curr_subtraj_len]
                # This is technically a whole df, but only ever one row should match since the combination
                # of gt_name and subtraj_len uniquely identifies a row in the rel_err table

        arr_si = df_si.loc[i, mode + "_err"]  # for the si frame we can directly use the index
        arr_sid = matched_row[mode + "_err"].iloc[0]

        # print("SI: ")
        # print(df_si.loc[i])
        # print()
        # print("SIDM: ")
        # print(matched_row.iloc[0])

        # Skip invalid or empty arrays
        if arr_si is None or arr_sid is None:
            continue
        if len(arr_si) == 0 or len(arr_sid) == 0:
            continue

        # Retrieve the median from the df, again making sure to use corresponding rows
        med_si = df_si.loc[i, mode + "_median"]
        med_sid = matched_row[mode + "_median"].iloc[0]

        # Create figure + axis (OO style)
        fig, ax = plt.subplots(figsize=(6, 4))

        # Construct the title
        title = "Relative "
        if mode[0] == "p":
            title += "Position "
        else:
            title += "Rotation "
        title += "Error"
        if len(mode) > 3:  # Mode has the "_no" suffix
            title += ", Outliers Removed\n"
        else:
            title += "\n"
        title += f"Sequence {df_si.loc[i, "gt_name"]}, " 
        title += f"Subtrajectory length {df_si.loc[i, "subtraj_len"]}m"

        unit = "m" if mode in ["pos", "pos_no"] else "°"

        # Histogram for SI
        ax.hist(arr_si, bins=bins, alpha=0.5, color="blue", label="Stereo-Inertial")
        ax.axvline(med_si, color="blue", linestyle="--", linewidth=2,
                   label=f"Median {med_si:.5f}{unit} over {arr_si.shape[0]} values")

        # Histogram for SID
        # row_name = df_sid.loc[i, "gt_name"] + " lenght " + str(df_si.loc[i, "subtraj_len"])
        ax.hist(arr_sid, bins=bins, alpha=0.5, color="orange", label="Stereo-Inertial-DVL")
        ax.axvline(med_sid, color="orange", linestyle="--", linewidth=2,
                   label=f"Median {med_sid:.5f}{unit} over {arr_sid.shape[0]} values")

        # Axis labels and title
        ax.set_title(title)
        ax.set_xlabel(f"Value [{unit}]")
        ax.set_ylabel("Frequency")
        ax.legend()

        if save is not None:
            name = f"{save}/hist_{sequence_dic[df_si.loc[i, "gt_name"]]}_"
            length = str(df_si.loc[i, "subtraj_len"]).replace(".", "p")
            name += length
            name += "_" + mode
            name += ".png"
            plt.savefig(name, dpi=300)
        else:
            fig.tight_layout()
            plt.show()


def histogram_boxplot_joined(df_si, df_sid, mode, save):

    # Aesthetic settings
    OFFSET = 0.03

    assert mode in ["rot", "pos", "rot_no", "pos_no"]

    n = min(len(df_si), len(df_sid))

    if mode == "rot_no" or "pos_no":
        clean_outliers_threshold(df_si)
        clean_outliers_threshold(df_sid)
    
    if "pos_median" not in df_si.columns:
        add_median(df_si)

    if "pos_median" not in df_sid.columns:
        add_median(df_sid)

    # Define what goes into each subplot-histogram combination. A template for transport.
    PlotEntry = namedtuple("PlotEntry", ["arr_si", "arr_sid", "med_si", "med_sid", "subtraj_len"])

    gt_names = set([name for name in df_sid["gt_name"]])

    for curr_gt_name in gt_names:

        plot_entries = []

        subtraj_lens = [j for j in df_si[df_si["gt_name"] == curr_gt_name]["subtraj_len"]]

        # ----- LOOP THROUGH SUBTRAJ LENGHTS ALGINING AND PACKAGING ----- #

        for curr_subtraj_len in subtraj_lens:

            # Iterate through df_si and match the rows in df_sid based on subtraj_len and gt_name
            # Within the eval scripts, the indexing on the subtrajectory lengths was certainly consistent,
            # but since now different eval outputs are compared, we have to make sure that we match the 
            # same sequences and subtrajectory lengths

            if not (df_sid["gt_name"]==curr_gt_name).any():
                # If this triggers, df_sid frame does not contain this gt_name
                continue
            else:
                # What remains in df_sid when filtering for the current gt_name
                df_sid_filtered = df_sid[df_sid["gt_name"]==curr_gt_name]
                if not (df_sid_filtered["subtraj_len"]==curr_subtraj_len).any():
                    # If this triggers, the df_sid_filtered does not containt this subtraj_len
                    continue
                else:
                    matched_row_sid = df_sid_filtered[df_sid_filtered["subtraj_len"]==curr_subtraj_len]
                    # This is technically a whole df, but only ever one row should match since the combination
                    # of gt_name and subtraj_len uniquely identifies a row in the rel_err table

            # Find the sub-df (really just one row) of df_si that corresponds to the curr_gt_name and the curr_sub_traj
            df_si_filtered = df_si[df_si["gt_name"]==curr_gt_name]
            matched_row_si = df_si_filtered[df_si_filtered["subtraj_len"]==curr_subtraj_len]

            arr_si = matched_row_si[mode + "_err"].iloc[0] 
            arr_sid = matched_row_sid[mode + "_err"].iloc[0]

            print("SI: ")
            print(matched_row_si.iloc[0])
            print()
            print("SIDM: ")
            print(matched_row_sid.iloc[0])

            # Skip invalid or empty arrays
            if arr_si is None or arr_sid is None:
                continue
            if len(arr_si) == 0 or len(arr_sid) == 0:
                continue

            # Retrieve the median from the df, again making sure to use corresponding rows
            med_si = matched_row_si[mode + "_median"].iloc[0]
            med_sid = matched_row_sid[mode + "_median"].iloc[0]

            plot_entry = PlotEntry(
                arr_si=arr_si,
                arr_sid=arr_sid,
                med_si=med_si,
                med_sid=med_sid,
                subtraj_len=curr_subtraj_len
            )

            plot_entries.append(plot_entry)


        # ------- PLOTS ------- #

        plt.rcParams['font.family'] = 'serif'

        fig, ax = plt.subplots(figsize=(6, 4))

        ax.grid(True)

        # Construct the title
        title = "Relative "
        if mode[0] == "p":
            title += "Position "
        else:
            title += "Rotation "
        title += "Error"
        if len(mode) > 3:  # Mode has the "_no" suffix
            title += ", Outliers Removed\n"
        else:
            title += "\n"
        title += f"Sequence {curr_gt_name}, " 

        for plot_entry in plot_entries:

            # --- Violinplots --- #

            # ---------------------- PLOT SI ---------------------- #

            v1 = ax.violinplot(plot_entry.arr_si, positions=[plot_entry.subtraj_len], widths=0.2, showmeans=False, showmedians=False) 
            
            # Colour violins according to distribution (orange for SID)
            for body in v1['bodies']: 
                body.set_facecolor('blue') 
                # body.set_edgecolor('black') 
                body.set_alpha(0.5)

            style_violin_parts(v1, color='blue')

            q1, q2, q3 = np.percentile(plot_entry.arr_si, [25, 50, 75])
            ax.vlines([plot_entry.subtraj_len + OFFSET], q1, q3, color="blue", linestyle="-", lw=8)
            ax.scatter([plot_entry.subtraj_len + OFFSET], [q2], marker="o", color="white", s=30, zorder=2, label="Median")
            ax.hlines([q2], [plot_entry.subtraj_len + 2 * OFFSET], [plot_entry.subtraj_len], color="blue")

            # ) 
            # ax.hlines([q1, q3], plot_entry.subtraj_len - 0.1, plot_entry.subtraj_len + 0.1, colors='blue', linewidth=1.2)



            # ---------------------- PLOT SID ---------------------- #

            v2 = ax.violinplot(plot_entry.arr_sid, positions=[plot_entry.subtraj_len], widths=0.2, showmeans=False, showmedians=False)
            
            # Colour violins according to distribution (blue for SI)
            for body in v2['bodies']: 
                body.set_facecolor('orange') 
                # body.set_edgecolor('black') 
                body.set_alpha(0.5)

            style_violin_parts(v2, color='orange')

            q1, q2, q3 = np.percentile(plot_entry.arr_sid, [25, 50, 75])
            ax.vlines([plot_entry.subtraj_len - OFFSET], q1, q3, color="orange", linestyle="-", lw=8)
            ax.scatter([plot_entry.subtraj_len - OFFSET], [q2], marker="o", color="white", s=30, zorder=2)
            ax.hlines([q2], [plot_entry.subtraj_len - 2 * OFFSET], [plot_entry.subtraj_len], color="orange")

            # --- Histograms --- #
            # SI
            # ax.hist(
            #     arr_si, 
            #     bins=50, 
            #     alpha=0.5, 
            #     color="blue", 
            #     # label="Stereo-Inertial",
            #     label=None,
            #     orientation="horizontal"
            # )
            # # SID
            # ax.hist(
            #     arr_sid, 
            #     bins=50, 
            #     alpha=0.5, 
            #     color="orange", 
            #     # label="Stereo-Inertial-DVL",
            #     label=None,
            #     orientation="horizontal"
            # )
            
        unit = "m" if mode in ["pos", "pos_no"] else "°"

        ax.set_xticks(subtraj_lens)
        ax.set_xticklabels([f"{j}" for j in subtraj_lens])
        ax.set_title(title)
        ax.set_xlabel("Subtrajectory Length [m]")
        ax.set_ylabel(f"Relative Error [{unit}]")
        ax.legend()

        legend_elements = [ 
            Patch(facecolor='orange', label='Stereo-Inertial-DVL'), 
            Patch(facecolor='blue', label='Stereo-Inertial') 
        ] 
        ax.legend(handles=legend_elements, title="Distributions", loc="upper left")

        sequence_dic = {
            "Structure_Easy": "se",
            "Structure_Medium": "sm",
            "Structure_Hard": "sh",
            "HalfTank_Easy": "he",
            "HalfTank_Medium": "hm",
            "HalfTank_Hard": "hh",
        }

        if save is not None:
            name = f"{save}_{sequence_dic[curr_gt_name]}" + mode
            name += ".png"
            plt.savefig(name, dpi=300)
        else:
            fig.tight_layout()
            plt.show()

def style_violin_parts(v, color):
    """
    v: dictionary returned by ax.violinplot
    color: violin color ('orange' or 'blue')
    """

    # 1. Vertical center line → always black
    if 'cbars' in v:
        obj = v['cbars']
        if isinstance(obj, collections.abc.Iterable):
            for artist in obj:
                artist.set_color('black')
                artist.set_linewidth(1.2)
                # artist.set_visible(False)
        else:
            obj.set_color('black')
            obj.set_linewidth(1.2)
            # obj.set_visible(False)

    # 2. Median, min, max → violin color
    for part in ('cmedians', 'cmins', 'cmaxes'):
        if part in v:
            obj = v[part]
            if isinstance(obj, collections.abc.Iterable):
                for artist in obj:
                    artist.set_color(color)
                    artist.set_linewidth(1.2)
            else:
                obj.set_color(color)
                obj.set_linewidth(1.2)



def compare_gt_use_histograms(df_si, df_sid, dvlcov, save=None):

    plt.rcParams['font.family'] = 'serif'

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
        fig, ax = plt.subplots(figsize=(5, 4))

        title = f"Percent Ground Truth Used in 10 Runs\non {sequence} with DVL Cov Factor {dvlcov}"

        # Specify the bins such that they are evenly spaced
        bins = np.linspace(0, 100, 11)

        # Histogram for SI
        ax.hist(si_frac_gt_used, bins=bins, alpha=0.5, color="blue", label="Stereo-Inertial")
        ax.axvline(m_si, color="blue", linestyle="--", linewidth=2,
                   label=f"Mean {m_si:.3f}%")

        # Histogram for SID
        ax.hist(sid_frac_gt_used, bins=bins, alpha=0.5, color="orange", label="Stereo-Inertial-DVL")
        ax.axvline(m_sid, color="orange", linestyle="--", linewidth=2,
                   label=f"Mean {m_sid:.3f}%")
        
        ax.set_title(title)
        ax.set_xlabel("Value [%]")
        ax.set_ylabel("Frequency")
        ax.legend()
        
        fig.tight_layout()

        sequence_dic = {
            "Structure_Easy": "se",
            "Structure_Medium": "sm",
            "Structure_Hard": "sh",
            "HalfTank_Easy": "he",
            "HalfTank_Medium": "hm",
            "HalfTank_Hard": "hh",
         }

        print(f"{round(m_sid - m_si, 3)} -- m_sid - msi {sequence}")

        if save is not None:
            name = f"gt_used_{sequence_dic[sequence]}_dvlcov"
            dvlcovstr = str(dvlcov).replace(".", "p")
            name += dvlcovstr + ".png"
            plt.savefig(save + f"/{name}", dpi=300)
        else:
            plt.show()


if __name__ == "__main__":

    DVL_COV = 10.0

    string_conversion = str(DVL_COV).replace(".", "p")

    dir_sid = Path(f"/home/ubuntu/ws_blue/data/eval_output/2026-02-02/sidm_dvlcov{string_conversion}_noWholeTank/error_data")
    dir_si = Path("/home/ubuntu/ws_blue/data/eval_output/2026-02-02/si_noWholeTank/error_data")

    rte_sid = pd.read_pickle(dir_sid.joinpath("rte.pkl"))
    rte_si = pd.read_pickle(dir_si.joinpath("rte.pkl"))

    # compare_histograms(rte_si, rte_sid, "pos_no", 50, save=f"/home/ubuntu/ws_blue/data/results/histograms_dvlcov{string_conversion}")

    # compare_gt_use_histograms(rte_si, rte_sid, DVL_COV, save="/home/ubuntu/ws_blue/data/results/frac_gt_used")

    histogram_boxplot_joined(rte_si, rte_sid, "rot_no") #, save=f"/home/ubuntu/ws_blue/data/results/vioilin_{string_conversion}")
