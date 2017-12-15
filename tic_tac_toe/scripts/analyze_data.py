#!/usr//bin/env python
import pickle
import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.stats
from plotting_utils import Plot
import pandas
import sys

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
MEASURES = {INTELLIGENCE: ["Q2_2", "Q2_7", "Q2_11", "Q2_15", "Q2_17"],
            LIKEABILITY: ["Q2_1", "Q2_4", "Q2_9", "Q2_13", "Q2_16"],
            ANIMACY: ["Q2_3", "Q2_6", "Q2_8", "Q2_10", "Q2_12", "Q2_14"],
            ABILITY: [],
            RESPONSIVENESS: [],
            SPEED: ["Q2_5"]}

idle_group = "Group A"
control_group = "Group B"

HEADER_OFFSET = 2


def compute_alpha():
    return 0


def calculate_scores(data):
    scores = {}
    for measure, component_list in MEASURES.items():
        scores[measure] = data[component_list].astype("float32").mean(1)
    return scores


def mean_std(data):
    desc = {}
    for measure, scores in data.items():
        desc[measure] = (scores.mean(), scores.std(), scores.size)
    return desc


with open(sys.argv[1], mode="rb") as file:
    results = pandas.read_csv(file)
    results = results[2:]

idle_data = results[results["Q4"] == idle_group]
control_data = results[results["Q4"] == control_group]

idle_scores = calculate_scores(idle_data)
control_scores = calculate_scores(control_data)

idle_descriptive = mean_std(idle_scores)
control_descriptive = mean_std(control_scores)

confidences_by_line = []
for measure in MEASURES.keys():
    idle_desc = idle_descriptive[measure]
    control_desc = control_descriptive[measure]
    idle_data = idle_scores[measure]
    control_data = control_scores[measure]
    line_confidences = []
    stat, p = scipy.stats.ttest_ind(idle_data, control_data)
    print("{}: {}".format(measure, p))
    confidences_by_line.append(line_confidences)

train_err, val_err, test_err = tuple(map(list, zip(*means_by_k)))
x_series = range(1, len(means_by_k) + 1)

training_err_plot = Plot("Test error")
training_err_plot.plot_evaluations(x_series, test_err, confidences_by_line[2], "")
# plt.semilogy()
plt.show()
