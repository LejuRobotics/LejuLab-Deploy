#pragma once

#include <memory>
#include <string>
#include <vector>

// 注意: 需要先安装 ONNX Runtime C++ API
// #include <onnxruntime_cxx_api.h>

#include "leju-rl-controller/inference/inference_model.h"

namespace leju {

/**
 * @brief ONNX Runtime 推理模型实现
 *
 * 使用 Microsoft ONNX Runtime 进行神经网络推理。
 * 支持 CPU、GPU 等多种执行提供者。
 *
 * 依赖安装:
 *   - sudo apt install libonnxruntime-dev
 *   或从源码编译: https://github.com/microsoft/onnxruntime
 */
class ONNXRuntimeModel : public InferenceModel {
 public:
  ONNXRuntimeModel();
  ~ONNXRuntimeModel() override;

  /**
   * @brief 加载模型文件
   * @param model_path 模型文件路径 (.onnx)
   * @return 加载成功返回 true，失败返回 false
   */
  bool load(const std::string& model_path) override;

  /**
   * @brief 检查模型是否已加载
   * @return 已加载返回 true，否则返回 false
   */
  bool isLoaded() const override { return loaded_; }

  /**
   * @brief 执行模型前向推理
   * @param input 输入张量数据（一维展平的 float 向量）
   * @return 输出张量数据（一维展平的 float 向量）
   */
  std::vector<float> forward(const std::vector<float>& input) override;

  /**
   * @brief 获取模型类型
   * @return 返回 ModelType::kONNXRuntime
   */
  ModelType getModelType() const override { return ModelType::kONNXRuntime; }

  /**
   * @brief 设置执行提供者 (CPU/CUDA/Tensorrt等)
   * @param provider 执行提供者名称
   *
   * 支持的提供者:
   *   - "CPUExecutionProvider" (默认)
   *   - "CUDAExecutionProvider" (需要 GPU 支持)
   *   - "TensorrtExecutionProvider" (需要 TensorRT)
   */
  void setProvider(const std::string& provider);

 private:
  bool loaded_ = false;                           ///< 模型加载状态

  // ONNX Runtime 对象 (Pimpl 模式，避免头文件依赖)
  struct Impl;
  std::unique_ptr<Impl> impl_;                    ///< 实现细节
};

}  // namespace leju
