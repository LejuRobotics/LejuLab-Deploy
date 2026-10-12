#pragma once
/**
 * 电机测试纯函数工具库
 * 线性回归、RMS 误差、稳态提取、通过标准判定
 * 无 DDS/硬件依赖，可独立单测
 */

#include <vector>
#include <cmath>
#include <numeric>
#include <string>
#include <sstream>
#include <iomanip>

namespace motor_test {

// ============================================================
// 线性回归
// ============================================================

struct LinRegResult {
    double slope = 0;
    double intercept = 0;
    double r_squared = 0;
    int n = 0;
};

inline LinRegResult linearRegression(const std::vector<double>& x,
                                     const std::vector<double>& y) {
    LinRegResult r;
    r.n = static_cast<int>(std::min(x.size(), y.size()));
    if (r.n < 2) return r;

    double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    for (int i = 0; i < r.n; ++i) {
        sx += x[i]; sy += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
        syy += y[i] * y[i];
    }
    double denom = r.n * sxx - sx * sx;
    if (std::fabs(denom) < 1e-15) return r;

    r.slope = (r.n * sxy - sx * sy) / denom;
    r.intercept = (sy - r.slope * sx) / r.n;

    // R²
    double y_mean = sy / r.n;
    double ss_tot = 0, ss_res = 0;
    for (int i = 0; i < r.n; ++i) {
        double y_pred = r.slope * x[i] + r.intercept;
        ss_res += (y[i] - y_pred) * (y[i] - y_pred);
        ss_tot += (y[i] - y_mean) * (y[i] - y_mean);
    }
    r.r_squared = (ss_tot > 1e-15) ? (1.0 - ss_res / ss_tot) : 0.0;
    return r;
}

// ============================================================
// RMS 误差
// ============================================================

inline double computeRmsError(const std::vector<double>& cmd,
                              const std::vector<double>& fb) {
    int n = static_cast<int>(std::min(cmd.size(), fb.size()));
    if (n == 0) return 0;
    double sum_sq = 0;
    for (int i = 0; i < n; ++i) {
        double e = fb[i] - cmd[i];
        sum_sq += e * e;
    }
    return std::sqrt(sum_sq / n);
}

// ============================================================
// 稳态提取
// ============================================================

struct SteadyState {
    double mean = 0;
    double std_dev = 0;
    int sample_count = 0;
};

/// 从等间隔采样的 data 中，丢弃前 settle_s 秒的样本，取剩余均值和标准差
inline SteadyState extractSteadyState(const std::vector<double>& data,
                                      int freq_hz, double settle_s) {
    SteadyState ss;
    int skip = static_cast<int>(settle_s * freq_hz);
    if (skip >= static_cast<int>(data.size())) {
        // settle 时间超过数据长度，取最后 10% 数据
        skip = static_cast<int>(data.size() * 0.9);
    }

    ss.sample_count = static_cast<int>(data.size()) - skip;
    if (ss.sample_count <= 0) return ss;

    double sum = 0;
    for (int i = skip; i < static_cast<int>(data.size()); ++i)
        sum += data[i];
    ss.mean = sum / ss.sample_count;

    double var_sum = 0;
    for (int i = skip; i < static_cast<int>(data.size()); ++i) {
        double d = data[i] - ss.mean;
        var_sum += d * d;
    }
    ss.std_dev = std::sqrt(var_sum / ss.sample_count);
    return ss;
}

// ============================================================
// 通过标准判定
// ============================================================

struct KpKdVerdict {
    bool pass = false;
    double r_squared = 0;
    double slope = 0;
    double expected_slope = 0;
    double slope_err_pct = 0;  // |slope - expected| / expected * 100
    std::string summary;
};

/// Kp/Kd 线性验证: R² > r2_threshold 且斜率误差 < slope_tol_pct
inline KpKdVerdict judgeKpKd(const LinRegResult& reg, double expected_slope,
                             double r2_threshold = 0.98,
                             double slope_tol_pct = 10.0) {
    KpKdVerdict v;
    v.r_squared = reg.r_squared;
    v.slope = reg.slope;
    v.expected_slope = expected_slope;

    if (std::fabs(expected_slope) > 1e-12)
        v.slope_err_pct = std::fabs(reg.slope - expected_slope) / std::fabs(expected_slope) * 100.0;
    else
        v.slope_err_pct = std::fabs(reg.slope) * 100.0;

    v.pass = (v.r_squared >= r2_threshold) && (v.slope_err_pct <= slope_tol_pct);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << (v.pass ? "PASS" : "FAIL")
        << "  R²=" << v.r_squared
        << "  slope=" << v.slope
        << "  expected=" << v.expected_slope
        << "  err=" << std::setprecision(1) << v.slope_err_pct << "%";
    v.summary = oss.str();
    return v;
}

struct CurrentStepVerdict {
    bool pass = false;
    double load_ratio = 0;     // 负载比例 (0.20, 0.50, 0.80, 0.90)
    double tau_cmd = 0;        // 指令力矩
    double tau_fb_mean = 0;    // 反馈力矩均值
    double rms_error = 0;      // RMS 误差 (Nm)
    double rms_pct = 0;        // RMS 误差 %
    double threshold_pct = 0;  // 通过阈值 %
    std::string summary;
};

/// 获取负载比例对应的 RMS 通过阈值 (%)
inline double getCurrentStepThreshold(double load_ratio) {
    if (load_ratio <= 0.20) return 3.0;
    if (load_ratio <= 0.50) return 5.0;
    if (load_ratio <= 0.80) return 8.0;
    return 10.0;  // 0.90
}

/// 电流阶跃判定: RMS% < 阈值
inline CurrentStepVerdict judgeCurrentStep(double load_ratio, double tau_cmd,
                                           const std::vector<double>& tau_fb_samples,
                                           int freq_hz, double settle_s) {
    CurrentStepVerdict v;
    v.load_ratio = load_ratio;
    v.tau_cmd = tau_cmd;
    v.threshold_pct = getCurrentStepThreshold(load_ratio);

    auto ss = extractSteadyState(tau_fb_samples, freq_hz, settle_s);
    v.tau_fb_mean = ss.mean;

    // RMS on steady-state portion
    int skip = static_cast<int>(settle_s * freq_hz);
    if (skip >= static_cast<int>(tau_fb_samples.size()))
        skip = static_cast<int>(tau_fb_samples.size() * 0.9);

    std::vector<double> cmd_vec, fb_vec;
    for (int i = skip; i < static_cast<int>(tau_fb_samples.size()); ++i) {
        cmd_vec.push_back(tau_cmd);
        fb_vec.push_back(tau_fb_samples[i]);
    }
    v.rms_error = computeRmsError(cmd_vec, fb_vec);
    v.rms_pct = (std::fabs(tau_cmd) > 1e-6) ? (v.rms_error / std::fabs(tau_cmd) * 100.0) : 0;
    v.pass = (v.rms_pct <= v.threshold_pct);

    std::ostringstream oss;
    oss << std::fixed;
    oss << (v.pass ? "PASS" : "FAIL")
        << "  load=" << std::setprecision(0) << (load_ratio * 100) << "%"
        << "  tau_cmd=" << std::setprecision(2) << tau_cmd << "Nm"
        << "  tau_fb=" << std::setprecision(2) << v.tau_fb_mean << "Nm"
        << "  RMS=" << std::setprecision(2) << v.rms_pct << "%"
        << "  (< " << std::setprecision(0) << v.threshold_pct << "%)";
    v.summary = oss.str();
    return v;
}

// ============================================================
// 逗号分隔字符串解析
// ============================================================

inline std::vector<double> parseDoubleList(const std::string& s) {
    std::vector<double> result;
    std::istringstream iss(s);
    std::string token;
    while (std::getline(iss, token, ',')) {
        size_t start = token.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        try { result.push_back(std::stod(token.substr(start))); } catch (...) {}
    }
    return result;
}

}  // namespace motor_test
