#include "leju-rl-controller/inference/onnxruntime_model.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

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
  std::vector<std::vector<int64_t>> input_shapes;
  std::vector<int64_t> input_shape;
  std::vector<std::vector<int64_t>> output_shapes;
  std::map<std::string, std::string> metadata;
  std::string requested_output_name;  // empty => output 0
  size_t selected_output_index = 0;

  bool resolveOutputIndex() {
    if (requested_output_name.empty()) {
      selected_output_index = 0;
      return true;
    }
    for (size_t i = 0; i < output_names.size(); ++i) {
      if (output_names[i] == requested_output_name) {
        selected_output_index = i;
        return true;
      }
    }
    std::cerr << "[ERROR] [ONNXRuntimeModel] output '" << requested_output_name
              << "' not found" << std::endl;
    return false;
  }
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

// ==================== 设置输出名称 ====================

void ONNXRuntimeModel::setOutputName(const std::string& output_name) {
#ifdef USE_ONNXRUNTIME
  impl_->requested_output_name = output_name;
  if (loaded_ && !impl_->resolveOutputIndex()) {
    loaded_ = false;
  }
#else
  (void)output_name;
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
    impl_->input_names.clear();
    impl_->input_shapes.clear();
    impl_->input_names.reserve(num_input_nodes);
    impl_->input_shapes.reserve(num_input_nodes);
    for (size_t i = 0; i < num_input_nodes; ++i) {
      // 使用新 API: GetInputNameAllocated
      auto input_name_allocated = impl_->session->GetInputNameAllocated(i, allocator);
      impl_->input_names.push_back(input_name_allocated.get());

      // 获取输入形状
      auto inputTypeInfo = impl_->session->GetInputTypeInfo(i);
      auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
      auto shape = inputTensorInfo.GetShape();
      impl_->input_shapes.push_back(shape);
      impl_->input_shape = shape;
    }

    // 获取输出信息
    impl_->output_names.clear();
    impl_->output_shapes.clear();
    impl_->output_names.reserve(num_output_nodes);
    impl_->output_shapes.reserve(num_output_nodes);
    for (size_t i = 0; i < num_output_nodes; ++i) {
      // 使用新 API: GetOutputNameAllocated
      auto output_name_allocated = impl_->session->GetOutputNameAllocated(i, allocator);
      impl_->output_names.push_back(output_name_allocated.get());

      // 获取输出形状
      auto outputTypeInfo = impl_->session->GetOutputTypeInfo(i);
      auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
      impl_->output_shapes.push_back(outputTensorInfo.GetShape());
    }

    // Resolve selected output (default: index 0).
    if (!impl_->resolveOutputIndex()) {
      loaded_ = false;
      return false;
    }

    // 读取自定义 metadata（observation_terms / future_step_num 等）
    impl_->metadata.clear();
    try {
      Ort::ModelMetadata model_meta = impl_->session->GetModelMetadata();
      auto keys = model_meta.GetCustomMetadataMapKeysAllocated(allocator);
      for (size_t i = 0; i < keys.size(); ++i) {
        const char* key = keys[i].get();
        auto value = model_meta.LookupCustomMetadataMapAllocated(key, allocator);
        if (key && value) {
          impl_->metadata[key] = value.get();
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "[WARNING] [ONNXRuntimeModel::load] "
                << "Failed to read model metadata: " << e.what() << std::endl;
    }

    loaded_ = true;

    // 打印模型信息
    std::cout << "[INFO] [ONNXRuntimeModel::load] "
              << "Model loaded from: " << model_path << std::endl;
    for (size_t i = 0; i < impl_->input_names.size(); ++i) {
      std::cout << "[INFO] [ONNXRuntimeModel::load] "
                << "Input '" << impl_->input_names[i] << "' shape: [";
      for (size_t d = 0; d < impl_->input_shapes[i].size(); ++d) {
        if (d > 0) std::cout << ", ";
        std::cout << impl_->input_shapes[i][d];
      }
      std::cout << "]" << std::endl;
    }

    const auto& out_shape = impl_->output_shapes[impl_->selected_output_index];
    std::cout << "[INFO] [ONNXRuntimeModel::load] selected='"
              << impl_->output_names[impl_->selected_output_index]
              << "' shape: [";
    for (size_t i = 0; i < out_shape.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << out_shape[i];
    }
    std::cout << "]" << std::endl;
    if (!impl_->metadata.empty()) {
      std::cout << "[INFO] [ONNXRuntimeModel::load] "
                << "Metadata keys: " << impl_->metadata.size() << std::endl;
    }

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
#ifdef USE_ONNXRUNTIME
  return forward(std::vector<std::vector<float>>{input},
                 impl_->input_names.empty()
                     ? std::vector<std::string>{"obs"}
                     : std::vector<std::string>{impl_->input_names.front()});
#else
  (void)input;
  std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
            << "ONNX Runtime not enabled" << std::endl;
  return {};
#endif
}

std::vector<float> ONNXRuntimeModel::forward(const std::vector<std::vector<float>>& inputs,
                                             const std::vector<std::string>& input_names) {
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
    if (inputs.size() != input_names.size()) {
      std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
                << "inputs/input_names size mismatch: " << inputs.size()
                << " vs " << input_names.size() << std::endl;
      return {};
    }

    std::vector<Ort::Value> input_tensors;
    std::vector<const char*> input_name_ptrs;
    input_tensors.reserve(impl_->input_names.size());
    input_name_ptrs.reserve(impl_->input_names.size());

    for (size_t model_input_idx = 0; model_input_idx < impl_->input_names.size(); ++model_input_idx) {
      const auto& model_input_name = impl_->input_names[model_input_idx];
      auto provided = std::find(input_names.begin(), input_names.end(), model_input_name);
      if (provided == input_names.end()) {
        std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
                  << "Missing input: " << model_input_name << std::endl;
        return {};
      }

      const size_t provided_idx = static_cast<size_t>(std::distance(input_names.begin(), provided));
      const auto& input = inputs[provided_idx];

      auto input_dims = impl_->input_shape;
      if (impl_->input_names.size() > 1) {
        auto inputTypeInfo = impl_->session->GetInputTypeInfo(model_input_idx);
        auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        input_dims = inputTensorInfo.GetShape();
      }

      size_t expected_size = 1;
      for (size_t dim_idx = 0; dim_idx < input_dims.size(); ++dim_idx) {
        if (input_dims[dim_idx] <= 0) {
          input_dims[dim_idx] = (dim_idx == 0) ? 1 : static_cast<int64_t>(input.size());
        }
        expected_size *= static_cast<size_t>(input_dims[dim_idx]);
      }

      if (input.size() != expected_size) {
        std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
                  << "Input size mismatch for '" << model_input_name
                  << "': got " << input.size()
                  << ", expected " << expected_size << std::endl;
        return {};
      }

      input_name_ptrs.push_back(model_input_name.c_str());
      input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
          *impl_->memory_info,
          const_cast<float*>(input.data()),
          input.size(),
          input_dims.data(),
          input_dims.size()));
    }

    // 只拉取配置选中的输出；Sonic 默认使用输出 0，HEFT 可选择 "action"。
    const size_t out_idx = impl_->selected_output_index;
    const char* output_name_ptr = impl_->output_names[out_idx].c_str();

    // 执行推理
    Ort::RunOptions run_options;
    auto output_tensors = impl_->session->Run(
        run_options,
        input_name_ptrs.data(), input_tensors.data(), input_tensors.size(),
        &output_name_ptr, 1);

    if (output_tensors.empty() || !output_tensors[0].IsTensor()) {
      std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
                << "Model returned no tensor output" << std::endl;
      return {};
    }

    auto output_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    const size_t output_size = output_info.GetElementCount();
    const float* output_data = output_tensors[0].GetTensorData<float>();
    std::vector<float> output(output_data, output_data + output_size);
    return output;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] [ONNXRuntimeModel::forward] "
              << "Inference failed: " << e.what() << std::endl;
    return {};
  }
#endif
}

std::vector<std::string> ONNXRuntimeModel::getInputNames() const {
#ifdef USE_ONNXRUNTIME
  return impl_->input_names;
#else
  return {};
#endif
}

int ONNXRuntimeModel::getInputFeatureDim(const std::string& input_name) const {
#ifdef USE_ONNXRUNTIME
  if (!loaded_) {
    return -1;
  }
  for (size_t i = 0; i < impl_->input_names.size(); ++i) {
    if (impl_->input_names[i] != input_name) {
      continue;
    }
    const auto& shape = impl_->input_shapes[i];
    int dim = 1;
    bool has_positive = false;
    for (size_t d = 0; d < shape.size(); ++d) {
      if (shape[d] <= 0) {
        // batch 维常为动态，跳过；特征维动态则改走 metadata 兜底
        if (d == 0) {
          continue;
        }
        has_positive = false;
        break;
      }
      dim *= static_cast<int>(shape[d]);
      has_positive = true;
    }
    if (has_positive) {
      return dim;
    }
    break;
  }

  // 兜底：从 observation_shapes / input_shapes metadata 解析，例如 {"prop":[750],...}
  for (const char* meta_key : {"observation_shapes", "input_shapes"}) {
    auto it = impl_->metadata.find(meta_key);
    if (it == impl_->metadata.end()) {
      continue;
    }
    const std::string& json = it->second;
    const std::string key = "\"" + input_name + "\"";
    const size_t key_pos = json.find(key);
    if (key_pos == std::string::npos) {
      continue;
    }
    const size_t bracket = json.find('[', key_pos + key.size());
    if (bracket == std::string::npos) {
      continue;
    }
    const size_t end = json.find(']', bracket);
    if (end == std::string::npos) {
      continue;
    }
    // 取最后一个正整数作为特征维（跳过 batch）
    int last_positive = -1;
    std::string nums = json.substr(bracket + 1, end - bracket - 1);
    std::stringstream ss(nums);
    std::string item;
    while (std::getline(ss, item, ',')) {
      try {
        const int v = std::stoi(item);
        if (v > 0) {
          last_positive = v;
        }
      } catch (...) {
      }
    }
    if (last_positive > 0) {
      return last_positive;
    }
  }
  return -1;
#else
  (void)input_name;
  return -1;
#endif
}

std::string ONNXRuntimeModel::getMetadata(const std::string& key) const {
#ifdef USE_ONNXRUNTIME
  auto it = impl_->metadata.find(key);
  if (it == impl_->metadata.end()) {
    return {};
  }
  return it->second;
#else
  (void)key;
  return {};
#endif
}

}  // namespace leju
