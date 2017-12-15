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

with open(sys.argv[1], mode="rb") as file:
    results = pandas.read_csv(file)


means_by_k = []
variances_by_k = []
num_samples_by_k = []
for results_for_k in results:
    all_values = results_for_k
    as_lists = list(map(list, zip(*all_values)))
    means_by_k.append(np.mean(as_lists,axis=1))
    variances_by_k.append(np.std(as_lists,axis=1) ** 2)
    num_samples_by_k.append(list(map(len, as_lists)))


confidences_by_line = []
for i in range(3):
    line_confidences = []
    for k in range(0, len(means_by_k)):
        mean, variance, n = means_by_k[k][i], variances_by_k[k][i], num_samples_by_k[k][i]
        crit = scipy.stats.t.ppf(1.0 - SIGNIFICANCE, n - 1)
        width = crit * np.math.sqrt(variance) / np.math.sqrt(n)
        line_confidences.append(width)
    confidences_by_line.append(line_confidences)

train_err, val_err, test_err = tuple(map(list, zip(*means_by_k)))
x_series = range(1, len(means_by_k) + 1)

training_err_plot = Plot("Training error")
training_err_plot.plot_evaluations(x_series, train_err, confidences_by_line[0], "")
plt.show()

training_err_plot = Plot("Validation error")
training_err_plot.plot_evaluations(x_series, val_err, confidences_by_line[1], "")
plt.show()

training_err_plot = Plot("Test error")
training_err_plot.plot_evaluations(x_series, test_err, confidences_by_line[2], "")
#plt.semilogy()
plt.show()