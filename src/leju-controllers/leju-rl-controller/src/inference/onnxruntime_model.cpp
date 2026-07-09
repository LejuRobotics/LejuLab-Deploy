#include "leju-rl-controller/inference/onnxruntime_model.h"

#include <iostream>
#include <memory>

// ONNX Runtime C++ API
#ifdef USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace leju {

// ==================== Pimpl 实现细节 ====================

struct ONNXRuntimeModel::Impl {
#ifdef USE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env;
  std::unique_ptr<Ort::Session> session;
  std::unique_ptr<Ort::SessionOptions> session_options;
  std::unique_ptr<Ort::MemoryInfo> memory_info;
  std::string provider_name = "CPUExecutionProvider";

  // 输入输出信息
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  std::vector<int64_t> input_shape;
  std::vector<int64_t> output_shape;
#else
  // 当未定义 USE_ONNXRUNTIME 时的占位符
  int dummy = 0;
#endif
};

// ==================== 构造/析构 ====================

ONNXRuntimeModel::ONNXRuntimeModel() : impl_(std::make_unique<Impl>()) {
#ifdef USE_ONNXRUNTIME
  try {
    // 创建 ONNX Runtime 环境
    impl_->env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "leju_rl_controller");

    // 配置会话选项
    impl_->session_options = std::make_unique<Ort::SessionOptions>();
    impl_->session_options->SetIntraOpNumThreads(1);
    impl_->session_options->SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // 内存信息 (CPU)
    impl_->memory_info = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] [ONNXRuntimeModel::Constructor] "
              << "Failed to initialize: " << e.what() << std::endl;
  }
#endif
}

ONNXRuntimeModel::~ONNXRuntimeModel() {
  // 智能指针自动清理
}

// ==================== 设置执行提供者 ====================

void ONNXRuntimeModel::setProvider(const std::string& provider) {
#ifdef USE_ONNXRUNTIME
  impl_->provider_name = provider;
#else
  std::cerr << "[WARNING] [ONNXRuntimeModel::setProvider] "
            << "USE_ONNXRUNTIME not defined, provider setting ignored" << std::endl;
#endif
}

// ==================== 加载模型 ====================

bool ONNXRuntimeModel::load(const std::string& model_path) {
#ifndef USE_ONNXRUNTIME
  std::cerr << "[ERROR] [ONNXRuntimeModel::load] "
            << "ONNX Runtime not enabled. Please define USE_ONNXRUNTIME "
            << "and link against onnxruntime library" << std::endl;
  return false;
#else
  try {
    // 创建会话
    impl_->session = std::make_unique<Ort::Session>(
        *impl_->env,
        model_path.c_str(),
        *impl_->session_options);

    // 获取输入输出数量
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes = impl_->session->GetInputCount();
    size_t num_output_nodes = impl_->session->GetOutputCount();

    // 获取输入信息
    impl_->input_names.reserve(num_input_nodes);
    for (size_t i = 0; i < num_input_nodes; ++i) {
      // 使用新 API: GetInputNameAllocated
      auto input_name_allocated = impl_->session->GetInputNameAllocated(i, allocator);
      impl_->input_names.push_back(input_name_allocated.get());

      // 获取输入形状
      auto inputTypeInfo = impl_->session->GetInputTypeInfo(i);
      auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
      impl_->input_shape = inputTensorInfo.GetShape();
    }

    // 获取输出信息
    impl_->output_names.reserve(num_output_nodes);
    for (size_t i = 0; i < num_output_nodes; ++i) {
      // 使用新 API: GetOutputNameAllocated
      auto output_name_allocated = impl_->session->GetOutputNameAllocated(i, allocator);
      impl_->output_names.push_back(output_name_allocated.get());

      // 获取输出形状
      auto outputTypeInfo = impl_->session->GetOutputTypeInfo(i);
      auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
      impl_->output_shape = outputTensorInfo.GetShape();
    }

    loaded_ = true;

    // 打印模型信息
    std::cout << "[INFO] [ONNXRuntimeModel::load] "
              << "Model loaded from: " << model_path << std::endl;
    std::cout << "[INFO] [ONNXRuntimeModel::load] "
              << "Input shape: [";
    for (size_t i = 0; i < impl_->input_shape.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << impl_->input_shape[i];
    }
    std::cout << "]" << std::endl;

    std::cout << "[INFO] [ONNXRuntimeModel::load] "
              << "Output shape: [";
    for (size_t i = 0; i < impl_->output_shape.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << impl_->output_shape[i];
    }
    std::cout << "]" << std::endl;

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] [ONNXRuntimeModel::load] "
              << "Failed to load model: " << e.what() << std::endl;
    loaded_ = false;
    return false;
  }
#endif
}

// ==================== 前向推理 ====================

std::vector<float> ONNXRuntimeModel::forward(const std::vector<float>& input) {
#ifndef USE_ONNXRUNTIME
  std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
            << "ONNX Runtime not enabled" << std::endl;
  return {};
#else
  if (!loaded_) {
    std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
              << "Model not loaded" << std::endl;
    return {};
  }

  try {
    // 计算输入大小
    size_t input_size = 1;
    for (auto dim : impl_->input_shape) {
      input_size *= static_cast<size_t>(dim);
    }

    // 验证输入大小
    if (input.size() != input_size) {
      std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
                << "Input size mismatch: got " << input.size()
                << ", expected " << input_size << std::endl;
      return {};
    }

    // 创建输入张量
    std::vector<int64_t> input_dims(impl_->input_shape.begin(),
                                     impl_->input_shape.end());

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        *impl_->memory_info,
        const_cast<float*>(input.data()),
        input.size(),
        input_dims.data(),
        input_dims.size());

    // 创建输出张量
    size_t output_size = 1;
    for (auto dim : impl_->output_shape) {
      output_size *= static_cast<size_t>(dim);
    }
    std::vector<float> output(output_size);

    std::vector<int64_t> output_dims(impl_->output_shape.begin(),
                                      impl_->output_shape.end());

    Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
        *impl_->memory_info,
        output.data(),
        output.size(),
        output_dims.data(),
        output_dims.size());

    // 准备输入输出名称指针
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;
    for (const auto& name : impl_->input_names) {
      input_name_ptrs.push_back(name.c_str());
    }
    for (const auto& name : impl_->output_names) {
      output_name_ptrs.push_back(name.c_str());
    }

    // 执行推理
    Ort::RunOptions run_options;
    impl_->session->Run(
        run_options,
        input_name_ptrs.data(), &input_tensor, 1,
        output_name_ptrs.data(), &output_tensor, 1);

    return output;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
              << "Inference failed: " << e.what() << std::endl;
    return {};
  }
#endif
}

}  // namespace leju
