#pragma once

#include <openvino/openvino.hpp>

#include "leju-rl-controller/inference/inference_model.h"

namespace leju {

/**
 * @brief OpenVINO 推理模型实现
 *
 * 使用 Intel OpenVINO 运行时进行神经网络推理。
 * 支持加载 ONNX、OpenVINO IR 等格式的模型文件。
 */
class OpenVINOModel : public InferenceModel {
 public:
  OpenVINOModel() = default;
  ~OpenVINOModel() override = default;

  /**
   * @brief 加载模型文件
   * @param model_path 模型文件路径
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
   * @return 返回 ModelType::kOpenVINO
   */
  ModelType getModelType() const override { return ModelType::kOpenVINO; }

 private:
  bool loaded_ = false;              ///< 模型加载状态
  ov::Core core_;                    ///< OpenVINO 运行时核心
  ov::CompiledModel compiled_model_; ///< 编译后的模型
  ov::InferRequest infer_request_;   ///< 推理请求对象
};

}  // namespace leju
