// ============================================================
// embedder_onnx.h — ResNet-18 appearance embedder via ONNX Runtime.
//
// Loads models/resnet18_embedder.onnx (exported by scripts/export_onnx.py
// from the same torchvision ResNet-18 used in the validated Python
// prototype, truncated at avgpool). CUDA execution provider is requested
// first; falls back to CPU automatically if unavailable so this still
// builds/runs on a laptop for smoke testing.
//
// Batched: embed() takes N crops and runs one forward pass, which is
// what makes the redetector's ~48-candidate grid search tractable at
// interactive frame rates.
// ============================================================
#pragma once

#include <onnxruntime_cxx_api.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class Embedder {
public:
  static constexpr int kImgSize = 128;
  static constexpr int kEmbedDim = 512;

  explicit Embedder(const std::string &onnxPath, bool preferCuda = true);

  // Returns an (N x kEmbedDim) row-major float matrix, L2-normalized rows.
  cv::Mat embed(const std::vector<cv::Mat> &bgrCrops);

  bool usingCuda() const { return usingCuda_; }

private:
  Ort::Env env_;
  Ort::Session session_{nullptr};
  Ort::MemoryInfo memInfo_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::string inputName_;
  std::string outputName_;
  bool usingCuda_ = false;
};
