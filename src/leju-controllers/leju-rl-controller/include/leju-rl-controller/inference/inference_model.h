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
   * @brief 前向推理（多输入模型）
   *
   * 默认实现只兼容单输入模型，具体后端可重写以支持命名输入。
   */
  virtual std::vector<float> forward(const std::vector<std::vector<float>>& inputs,
                                     const std::vector<std::string>& input_names) {
    if (inputs.size() == 1) {
      return forward(inputs[0]);
    }
    (void)input_names;
    return {};
  }

  /**
   * @brief 获取模型输入名称（不支持时返回空）
   */
  virtual std::vector<std::string> getInputNames() const { return {}; }

  /**
   * @brief 获取指定输入的展平特征维度（不含 batch）；失败返回 -1
   */
  virtual int getInputFeatureDim(const std::string& input_name) const {
    (void)input_name;
    return -1;
  }

  /**
   * @brief 读取 ONNX 自定义 metadata；不存在返回空字符串
   */
  virtual std::string getMetadata(const std::string& key) const {
    (void)key;
    return {};
  }

  /**
   * @brief 选择输出张量（多输出模型用）
   * @param output_name 输出张量名称（如 "action"）
   */
  virtual void setOutputName(const std::string& output_name) { (void)output_name; }

  /**
   * @brief 获取模型类型
   */
  virtual ModelType getModelType() const = 0;
};

}  // namespace leju
