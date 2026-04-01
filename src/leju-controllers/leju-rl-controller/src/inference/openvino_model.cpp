#include "leju-rl-controller/inference/openvino_model.h"

#include <iostream>

namespace leju {

bool OpenVINOModel::load(const std::string& model_path) {
  try {
    // 编译模型到 CPU
    compiled_model_ = core_.compile_model(model_path, "CPU");

    // 创建推理请求
    infer_request_ = compiled_model_.create_infer_request();

    loaded_ = true;

    // 打印模型信息
    auto input_shape = compiled_model_.input().get_shape();
    auto output_shape = compiled_model_.output().get_shape();

    std::cout << "[INFO] [OpenVINOModel::load] "
              << "Model loaded from: " << model_path << std::endl;
    std::cout << "[INFO] [OpenVINOModel::load] "
              << "Input shape: [";
    for (size_t i = 0; i < input_shape.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << input_shape[i];
    }
    std::cout << "]" << std::endl;

    std::cout << "[INFO] [OpenVINOModel::load] "
              << "Output shape: [";
    for (size_t i = 0; i < output_shape.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << output_shape[i];
    }
    std::cout << "]" << std::endl;

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] [OpenVINOModel::load] "
              << "Failed to load model: " << e.what() << std::endl;
    loaded_ = false;
    return false;
  }
}

std::vector<float> OpenVINOModel::forward(const std::vector<float>& input) {
  if (!loaded_) {
    std::cerr << "[ERROR] [OpenVINOModel::forward] "
              << "Model not loaded" << std::endl;
    return {};
  }

  try {
    // 获取输入张量
    auto input_tensor = infer_request_.get_input_tensor();
    auto input_shape = input_tensor.get_shape();
    size_t input_size = input_tensor.get_size();

    // 检查输入大小
    if (input.size() != input_size) {
      std::cerr << "[ERROR] [OpenVINOModel::forward] "
                << "Input size mismatch: got " << input.size()
                << ", expected " << input_size << std::endl;
      return {};
    }

    // 复制输入数据
    auto element_type = input_tensor.get_element_type();
    if (element_type == ov::element::f32) {
      float* input_data = input_tensor.data<float>();
      std::copy(input.begin(), input.end(), input_data);
    } else if (element_type == ov::element::f64) {
      double* input_data = input_tensor.data<double>();
      for (size_t i = 0; i < input.size(); ++i) {
        input_data[i] = static_cast<double>(input[i]);
      }
    } else {
      std::cerr << "[ERROR] [OpenVINOModel::forward] "
                << "Unsupported input element type: " << element_type << std::endl;
      return {};
    }

    // 执行推理
    infer_request_.infer();

    // 获取输出张量
    auto output_tensor = infer_request_.get_output_tensor();
    size_t output_size = output_tensor.get_size();
    std::vector<float> output(output_size);

    // 复制输出数据
    auto output_element_type = output_tensor.get_element_type();
    if (output_element_type == ov::element::f32) {
      const float* output_data = output_tensor.data<float>();
      std::copy(output_data, output_data + output_size, output.begin());
    } else if (output_element_type == ov::element::f64) {
      const double* output_data = output_tensor.data<double>();
      for (size_t i = 0; i < output_size; ++i) {
        output[i] = static_cast<float>(output_data[i]);
      }
    } else {
      std::cerr << "[ERROR] [OpenVINOModel::forward] "
                << "Unsupported output element type: " << output_element_type << std::endl;
      return {};
    }

    return output;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] [OpenVINOModel::forward] "
              << "Inference failed: " << e.what() << std::endl;
    return {};
  }
}

}  // namespace leju
