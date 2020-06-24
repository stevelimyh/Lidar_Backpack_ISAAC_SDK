/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "ConfusionMatrixGenerator.hpp"

#include <utility>

#include "messages/tensor.hpp"

namespace isaac {
namespace message_generators {

void ConfusionMatrixGenerator::start() {
  tickPeriodically();
}

void ConfusionMatrixGenerator::tick() {
  auto object_detection_metrics = tx_confusion_matrix().initProto();

  object_detection_metrics.setNumSamples(1);

  auto ious = object_detection_metrics.initThresholds(2);
  ious.set(0, 0.5);
  ious.set(1, 0.95);

  Tensor3i confusionMatrices(5, 5, 2);

  int count = 0;
  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 5; col++) {
      for (int i = 0; i < 2; i++) {
        confusionMatrices(row, col, i) = ++count;
      }
    }
  }

  ToProto(std::move(confusionMatrices), object_detection_metrics.initConfusionMatrices(),
          tx_confusion_matrix().buffers());

  tx_confusion_matrix().publish();
}

}  // namespace message_generators
}  // namespace isaac
