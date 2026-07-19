#include "embedder_onnx.h"

#include <array>
#include <cstdio>
#include <cstring>

namespace {
constexpr float kMean[3] = {0.485f, 0.456f, 0.406f};
constexpr float kStd[3] = {0.229f, 0.224f, 0.225f};
}  // namespace

Embedder::Embedder(const std::string &onnxPath, bool preferCuda)
    : env_(ORT_LOGGING_LEVEL_WARNING, "ltmu_embedder") {
  Ort::SessionOptions opts;
  opts.SetIntraOpNumThreads(2);
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  if (preferCuda) {
    try {
      OrtCUDAProviderOptions cudaOpts{};
      opts.AppendExecutionProvider_CUDA(cudaOpts);
      usingCuda_ = true;
    } catch (const Ort::Exception &e) {
      fprintf(stderr, "[EMBEDDER] CUDA EP unavailable (%s) — falling back to CPU\n",
              e.what());
      usingCuda_ = false;
    }
  }

  session_ = Ort::Session(env_, onnxPath.c_str(), opts);

  Ort::AllocatorWithDefaultOptions alloc;
  inputName_ = session_.GetInputNameAllocated(0, alloc).get();
  outputName_ = session_.GetOutputNameAllocated(0, alloc).get();

  printf("[EMBEDDER] loaded %s (%s) — input=%s output=%s\n", onnxPath.c_str(),
         usingCuda_ ? "CUDA" : "CPU", inputName_.c_str(), outputName_.c_str());
}

cv::Mat Embedder::embed(const std::vector<cv::Mat> &bgrCrops) {
  const int N = static_cast<int>(bgrCrops.size());
  if (N == 0) return cv::Mat();

  std::vector<float> blob(static_cast<size_t>(N) * 3 * kImgSize * kImgSize);
  for (int n = 0; n < N; n++) {
    cv::Mat resized;
    cv::resize(bgrCrops[n], resized, cv::Size(kImgSize, kImgSize));
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

    std::vector<cv::Mat> ch(3);
    cv::split(rgb, ch);
    for (int c = 0; c < 3; c++) {
      ch[c] = (ch[c] - kMean[c]) / kStd[c];
      float *dst = blob.data() + (static_cast<size_t>(n) * 3 + c) * kImgSize * kImgSize;
      std::memcpy(dst, ch[c].ptr<float>(), sizeof(float) * kImgSize * kImgSize);
    }
  }

  std::array<int64_t, 4> shape{N, 3, kImgSize, kImgSize};
  Ort::Value input = Ort::Value::CreateTensor<float>(
      memInfo_, blob.data(), blob.size(), shape.data(), shape.size());

  const char *inputNames[] = {inputName_.c_str()};
  const char *outputNames[] = {outputName_.c_str()};
  auto outputs = session_.Run(Ort::RunOptions{nullptr}, inputNames, &input, 1,
                               outputNames, 1);

  float *outData = outputs[0].GetTensorMutableData<float>();
  auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  int D = static_cast<int>(outShape.back());  // expected kEmbedDim

  cv::Mat feats(N, D, CV_32F, outData);
  cv::Mat result = feats.clone();
  for (int n = 0; n < N; n++) {
    cv::Mat row = result.row(n);
    float norm = static_cast<float>(cv::norm(row, cv::NORM_L2));
    if (norm > 1e-6f) row /= norm;
  }
  return result;
}
