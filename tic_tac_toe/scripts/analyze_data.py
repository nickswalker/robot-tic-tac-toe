#!/usr//bin/env python
import pickle
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
#rcParams['font.serif'] = ['DejuVu Serif']
import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.stats

from matplotlib.ticker import MultipleLocator, AutoMinorLocator

from plotting_utils import Plot
import pandas
import sys

params = {'backend': 'ps',
          'axes.labelsize': 2,
          'font.size': 14,
          'font.weight': "bold",
          'legend.fontsize': 6,
          'xtick.labelsize': 6,
          'ytick.labelsize': 6,

          'figure.titleweight': "bold",

          }

SIGNIFICANCE = 0.05

likert_item_to_numeric = {1: "Strongly disagree",
                          2: "Disagree",
                          3: "Somewhat disagree",
                          4: "Neither agree nor disagree",
                          5: "Somewhat agree",
                          6: "Agree",
                          7: "Strongly agree"}

INTELLIGENCE = "intelligence"
LIKEABILITY = "likability"
ANIMACY = "animacy"
ABILITY = "ability"
SPEED = "speed"
RESPONSIVENESS = "responsiveness"
MEASURES = {INTELLIGENCE: ["QID2_2", "QID2_7", "QID2_11", "QID2_15", "QID2_17"],
            LIKEABILITY: ["QID2_1", "QID2_4", "QID2_9", "QID2_13", "QID2_16"],
            ANIMACY: ["QID2_3", "QID2_6", "QID2_8", "QID2_10", "QID2_12", "QID2_14"],
            ABILITY: ["QID1_1", "QID1_3", "QID1_4", "QID1_5", "QID1_6"],
            RESPONSIVENESS: ["QID1_2"],
            SPEED: ["QID2_5"]}

REVERSE_SCALE = ["QID1_6", "QID1_4"]

treatment_group = '1'
control_group = '2'

HEADER_OFFSET = 2


def get_composite_responses(data):
    scores = {}
    for measure, component_list in MEASURES.items():
        scores[measure] = data[component_list].astype("float32").mean(1)
    return scores


def mean_std(data):
    desc = {}
    for measure, scores in data.items():
        desc[measure] = (scores.mean(), scores.std(), scores.size)
    return desc


def cronbach(itemscores):
    itemvars = itemscores.var(axis=1, ddof=1)
    tscores = itemscores.sum(axis=0)
    nitems = len(itemscores)
    return nitems / (nitems-1) * (1 - itemvars.sum() / tscores.var(ddof=1))


def main():
    with open(sys.argv[1], mode="rb") as data_csv:
        data = pandas.read_csv(data_csv)
    # Lop off the header rows
    data = data[2:]

    # Some of the questions have inverted responses. Flip them.
    data[REVERSE_SCALE] = data[REVERSE_SCALE].astype("float32").multiply(-1).add(8)

    # Check the integrity of the measures
    for measure, components in MEASURES.items():
        if len(components) < 2:
            continue
        alpha = cronbach(data[components].T.astype("float32"))
        print("{} alpha={}".format(measure, alpha))

    treatment_data = data[data["QID5"] == treatment_group]
    control_data = data[data["QID5"] == control_group]

    treatment_composites = get_composite_responses(treatment_data)
    control_composites = get_composite_responses(control_data)

    treatment_desc = mean_std(treatment_composites)
    control_desc = mean_std(control_composites)

    for measure in MEASURES.keys():

        treatment_data = treatment_composites[measure]
        control_data = control_composites[measure]
        stat, p = scipy.stats.ttest_ind(treatment_data, control_data)
        print("{}: {}".format(measure, p))

    names = MEASURES.keys()
    N = len(names)
    control_means = [control_desc[name][0] for name in names]
    control_std = [control_desc[name][1] for name in names]
    treatment_means = [treatment_desc[name][0] for name in names]
    treatment_std = [treatment_desc[name][1] for name in names]

    minorLocator = AutoMinorLocator(2)
    for i in range(N):
        plt.subplot(1, N, i + 1)
        width = 0.50       # the width of the bars
        padding = .15
        fig = plt.gcf()
        ax = plt.gca()
        plt.margins(padding, 0)
        plt.tight_layout(pad=1, w_pad=0.1, h_pad=.1)
        rects1 = ax.bar(padding, control_means[i], width, color=(.25, .30, .35), yerr=control_std[i], linewidth=0, ecolor='black')
        rects2 = ax.bar(padding + width + padding, treatment_means[i], width, color=(.26, .65, .96), yerr=treatment_std[i], linewidth=0, ecolor='black')

        # add some text for labels, title and axes ticks
        ax.set_ylabel(names[i])
        #ax.set_title(names[i])
        ax.set_xticks([padding + width / 2, 2 * padding + width + width / 2])
        ax.set_ylim([1, 7])
        ax.set_xticklabels(["Absent", "Present"])
        plt.minorticks_on()
        ax.yaxis.set_minor_locator(minorLocator)
        ax.tick_params(direction="out", which="both", width=1.5)
        ax.tick_params(which="minor", length=5)
        ax.tick_params(which="major", length=10)
        #ax.tick_params(which="minor", axis="y", length=5)

        ax.tick_params(axis='x',which='minor',bottom='off')
        ax.tick_params(top=False, right=False, which="both")


    plt.show()


if __name__ == "__main__":
    main()