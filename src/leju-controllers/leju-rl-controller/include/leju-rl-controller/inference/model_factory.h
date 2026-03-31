#pragma once

#include <memory>
#include <string>

#include "leju-rl-controller/inference/inference_model.h"
#include "leju-rl-controller/inference/openvino_model.h"

#ifdef USE_ONNXRUNTIME
#include "leju-rl-controller/inference/onnxruntime_model.h"
#endif

namespace leju {

/**
 * @brief 推理模型工厂类
 *
 * 根据指定的类型创建推理模型实例。
 * 支持的模型类型: OpenVINO, ONNX Runtime
 */
class ModelFactory {
 public:
  /**
   * @brief 创建推理模型实例
   * @param type 模型类型 (kOpenVINO, kONNXRuntime)
   * @return 模型智能指针，失败返回 nullptr
   *
   * 示例:
   *   auto model = ModelFactory::create(ModelType::kOpenVINO);
   *   model->load("model.onnx");
   *   auto output = model->forward(input);
   */
  static std::unique_ptr<InferenceModel> create(ModelType type) {
    switch (type) {
      case ModelType::kOpenVINO:
        return std::make_unique<OpenVINOModel>();

#ifdef USE_ONNXRUNTIME
      case ModelType::kONNXRuntime:
        return std::make_unique<ONNXRuntimeModel>();
#endif

      case ModelType::kTorch:
        std::cerr << "[ERROR] [ModelFactory] "
                  << "PyTorch TorchScript not yet implemented" << std::endl;
        return nullptr;

      default:
        std::cerr << "[ERROR] [ModelFactory] "
                  << "Unknown model type" << std::endl;
        return nullptr;
    }
  }

  /**
   * @brief 从字符串创建模型
   * @param type_str 模型类型字符串 ("openvino", "onnxruntime")
   * @return 模型智能指针，失败返回 nullptr
   */
  static std::unique_ptr<InferenceModel> create(const std::string& type_str) {
    if (type_str == "openvino" || type_str == "ov") {
      return create(ModelType::kOpenVINO);
#ifdef USE_ONNXRUNTIME
    } else if (type_str == "onnxruntime" || type_str == "onnx" || type_str == "ort") {
      return create(ModelType::kONNXRuntime);
#endif
    } else if (type_str == "torch" || type_str == "pytorch") {
      return create(ModelType::kTorch);
    } else {
      std::cerr << "[ERROR] [ModelFactory] "
                << "Unknown model type: " << type_str << std::endl;
      std::cerr << "[ERROR] [ModelFactory] "
                << "Supported types: openvino, onnxruntime" << std::endl;
      return nullptr;
    }
  }

  /**
   * @brief 获取默认模型类型
   * @return 默认返回 OpenVINO
   */
  static ModelType getDefaultType() {
    return ModelType::kOpenVINO;
  }

  /**
   * @brief 将模型类型转换为字符串
   */
  static std::string typeToString(ModelType type) {
    switch (type) {
      case ModelType::kOpenVINO:
        return "OpenVINO";
#ifdef USE_ONNXRUNTIME
      case ModelType::kONNXRuntime:
        return "ONNXRuntime";
#endif
      case ModelType::kTorch:
        return "PyTorch";
      default:
        return "Unknown";
    }
  }
};

}  // namespace leju
