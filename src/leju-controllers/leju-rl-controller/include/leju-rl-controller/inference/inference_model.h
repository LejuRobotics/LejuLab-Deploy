#pragma once

#include <string>
#include <vector>

namespace leju {

/**
 * @brief 推理模型类型
 */
enum class ModelType {
  kOpenVINO,    ///< OpenVINO
  kONNXRuntime, ///< ONNX Runtime
  kTorch,       ///< PyTorch TorchScript
};

/**
 * @brief 推理模型抽象接口
 */
class InferenceModel {
 public:
  virtual ~InferenceModel() = default;

  /**
   * @brief 加载模型
   * @param model_path 模型文件路径
   * @return 是否加载成功
   */
  virtual bool load(const std::string& model_path) = 0;

  /**
   * @brief 是否已加载
   */
  virtual bool isLoaded() const = 0;

  /**
   * @brief 前向推理
   * @param input 输入数据 (float)
   * @return 输出数据 (float)
   */
  virtual std::vector<float> forward(const std::vector<float>& input) = 0;

  /**
   * @brief 获取模型类型
   */
  virtual ModelType getModelType() const = 0;
};

}  // namespace leju
