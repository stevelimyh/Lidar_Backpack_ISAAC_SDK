'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from __future__ import absolute_import, division, print_function
import argparse
import time
import math
import logging
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sn
import os
import json
import datetime
from sklearn.metrics import auc

from engine.pyalice import *

np.random.seed(10)


class VerifyConfusionMatrices(Codelet):
    """
    This codelet consumes the aggregated confusion matrices of an evaluation run of the object detection pipeline.
    On every other tick, this codelet produces visualizations of the evaluation results and a JSON containing
    a summary of the processed metrics. On stop, the precisions and recall values are verified by checking that
    they exceed certain baseline values.
    """
    def start(self):
        self.rx = self.isaac_proto_rx("ConfusionMatrixProto", "object_detection_metrics")
        self.tick_on_message(self.rx)
        self.config.results_directory = "/tmp/object_detection_evaluation/"
        self.config.class_names = ["Dolly"]
        self.config.min_precision = 0.0
        self.config.min_recall = 0.0
        self.config.visualization_interval = 2

        try:
            self.num_images = 0
            self.iou_thresholds = []
            self.confusion_matrics = None
            self.class_names = self.config.class_names

            # Create a folder to store all the generated files (visualizations and summary JSON report)
            metric_plots_dir = self.config.results_directory
            if not os.path.exists(metric_plots_dir):
                os.makedirs(metric_plots_dir)
        except:
            logging.exception('')

    def tick(self):
        try:
            odmp_msg = self.rx.message.proto
            buffer_data = self.rx.message.buffers[odmp_msg.confusionMatrices.dataBufferIndex]
            self.sizes = odmp_msg.confusionMatrices.sizes
            self.element_type = odmp_msg.confusionMatrices.elementType
            if self.element_type != "int32":
                self.log_error("Buffer element type must be Int32 but was {}".format(
                    self.element_type))
            self.confusion_matrices = np.reshape(np.frombuffer(buffer_data, dtype=np.int32),
                                                 self.sizes)
            self.num_images = odmp_msg.numSamples
            self.iou_thresholds = list(odmp_msg.thresholds)

            self.num_classes = self.sizes[0]
            self.num_iou_thresholds = self.sizes[2]

            # Show the results every other tick
            if self.tick_count % self.config.visualization_interval == 0:
                self.process_confusion_matrices()
                self.create_json_report()
                self.create_visualizations()

        except:
            logging.exception('')

    def stop(self):
        print("Evaluated over " + str(self.num_images) + " frames")
        try:
            # Assert that the final metrics meet the baseline values for object detection metrics
            for iou_idx in range(len(self.iou_thresholds)):
                for class_idx in range(len(self.class_names)):
                    assert \
                        self.per_class_precisions[class_idx, iou_idx] >= \
                        self.config.min_precision, \
                        "Precision is too low for class {} with IoU {}".format( \
                            self.class_names[class_idx], self.iou_thresholds[iou_idx])
                    assert self.per_class_recalls[class_idx, iou_idx] >= \
                        self.config.min_recall, \
                        "Recall is too low for class {} with IoU {}".format(
                            self.class_names[class_idx], self.iou_thresholds[iou_idx])

        except:
            logging.exception('')

    def process_confusion_matrices(self):
        """ Compute precision and recall metrics for each class and each IoU from the confusion matrix.
        Using these precison and recall values, compute the Area Under Curve value, which describes
        how much the model is capable of distinguishing between classes.
        """
        # Calculate precision and recall for each class and IoU.
        self.per_class_precisions = np.zeros((self.num_classes, self.num_iou_thresholds))
        self.per_class_recalls = np.zeros((self.num_classes, self.num_iou_thresholds))
        for iou_idx in range(self.num_iou_thresholds):
            conf_mat = self.confusion_matrices[:, :, iou_idx]
            precisions = np.diagonal(conf_mat) / np.sum(conf_mat, axis=0)
            recalls = np.diagonal(conf_mat) / np.sum(conf_mat, axis=1)
            self.per_class_precisions[:, iou_idx] = precisions
            self.per_class_recalls[:, iou_idx] = recalls

        # For each class, calculate the Area Under Curve metric (AUC)
        self.aucs = np.zeros(len(self.class_names))
        for class_idx in range(len(self.class_names)):
            self.aucs[class_idx] = auc(self.per_class_recalls[class_idx, :],
                                       self.per_class_precisions[class_idx, :])

    def create_json_report(self):
        """ Create a JSON report with statistics derived from the confusion matrices."""
        json_report = {}
        json_report["trial_name"] = "object_detection_metrics_" + str(
            datetime.datetime.now().strftime("%Y-%m-%d"))
        json_report["iou_thresholds"] = self.iou_thresholds

        statistics = []
        for class_idx in range(len(self.class_names)):
            per_class_stats = {}
            per_class_stats["class_name"] = self.class_names[class_idx]
            per_class_stats["precisions"] = self.per_class_precisions[class_idx, :].tolist()
            per_class_stats["recalls"] = self.per_class_recalls[class_idx, :].tolist()
            per_class_stats["area_under_curve"] = self.aucs[class_idx]
            statistics.append(per_class_stats)
        json_report["statistics"] = statistics

        # Write the Json to file
        with open(self.config.results_directory + "object_detection_metrics.json", 'w') as f:
            json.dump(json_report, f, indent=2)

    def create_visualizations(self):
        """Create visualizations of confusion matrices for all IoU thresholds and precision-recall curves for each class."""
        # Visualize a confusion matrix per IoU threshold
        for iou_idx in range(len(self.iou_thresholds)):
            conf_mat = self.confusion_matrices[:, :, iou_idx]
            self.plot_confusion_matrix(conf_mat, self.iou_thresholds[iou_idx])

        # For each class, visualize a precision-recall curve
        for class_idx in range(len(self.class_names)):
            self.plot_precision_recall(self.class_names[class_idx],
                                       self.per_class_recalls[class_idx, :],
                                       self.per_class_precisions[class_idx, :],
                                       self.aucs[class_idx])

    def plot_confusion_matrix(self, conf_mat, iou_threshold):
        """Plot a confusion matrix and save the figure to the output directory."""
        ax = plt.gca()
        conf_mat_dataframe = pd.DataFrame(conf_mat, self.class_names + ["bg"],
                                          self.class_names + ["bg"])
        sn.heatmap(conf_mat_dataframe,
                   ax=ax,
                   linewidths=1,
                   cbar=False,
                   annot=True,
                   fmt="g",
                   annot_kws={"size": 10})
        plt.title("Confusion Matrix for IoU Threshold = " + "{:.2f}".format(iou_threshold))
        plt.ylabel("Ground truth class")
        plt.xlabel("Predicted class")
        plt.tight_layout()
        plt.savefig(
            str(self.config.results_directory) + "{:.2f}".format(iou_threshold) + "-confmat.png")
        plt.cla()

    def plot_precision_recall(self, class_name, per_class_precisions, per_class_recalls, auc):
        """Plot a precision-recall curve and save the figure to the output directory."""
        ax = plt.gca()
        ax.plot(per_class_recalls, per_class_precisions, marker='x')
        for iou_idx in range(len(self.iou_thresholds)):
            ax.annotate("{:.2f}".format(self.iou_thresholds[iou_idx]),
                        (per_class_recalls[iou_idx], per_class_precisions[iou_idx]))
        ax.set_xlim([0, 1])
        ax.set_ylim([0, 1])
        plt.title("Precision-Recall Curve for " + class_name + " class with AUC=" +
                  "{:.2f}".format(auc))
        plt.xlabel("Recall")
        plt.ylabel("Precision")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(str(self.config.results_directory) + class_name + "-PR-curve.png")
        plt.cla()


def main():
    parser = argparse.ArgumentParser(description='Verify confusion matrices')
    parser.add_argument(
        '--mode',
        dest='mode',
        choices=['sim', 'log'],
        type=str,
        default='sim',
        help='Running mode. Valid values: sim, log',
    )
    args, _ = parser.parse_known_args()

    app = Application(name="evaluate_object_detection")
    running_time = 100
    if (args.mode == "sim"):
        app.load(
            "packages/ml/apps/evaluate_object_detection/" \
                "evaluate_object_detection_simulation.app.json")
    elif (args.mode == "log"):
        app.load(
            "packages/ml/apps/evaluate_object_detection/evaluate_object_detection_log.app.json")
        running_time = 40 #length of the default log
    else:
        print("Mode not supported")
        return

    app.nodes['object_detection_metrics_verifier'].add(VerifyConfusionMatrices,
                                                       'isaac.alice.PyCodelet')
    app.load("packages/detect_net/apps/detect_net_industrial_dolly_fof.config.json")
    decoder = app.nodes["detect_net_inference.detection_decoder"]\
        .components["isaac.detect_net.DetectNetDecoder"]
    decoder.config.labels = ["Dolly"]
    app.run(float(running_time))


if __name__ == '__main__':
    main()
