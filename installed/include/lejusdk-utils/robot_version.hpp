#pragma once
#include <string>
#include <cstdint>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cstdlib>

namespace leju {

/// @brief 机器人版本号类
/// @details 机器人版本号类，用于表示机器人版本号
/// @note 机器人版本号格式为：
/// - MMMM 表示次版本号, 范围为0-9999
/// - N 表示修订号, 范围为0-9
/// - PPP 表示主版本号, 范围为0-9999
/// - BIGNUMBER: PPPPMMMMN
/// - STRING: PPPPMMMMN  Patch 为 0 时，不显示 Patch 部分, 比如 M(4), N(5),P(0) 表示为 45 而非 000000045 (0000,0004,5)
class RobotVersion
{
private:
    uint16_t major_ = 0;
    uint16_t minor_ = 0;
    uint16_t patch_ = 0;

public:
    /// @brief 默认构造函数
    constexpr RobotVersion() = default;

    /// @brief 拷贝构造函数
    constexpr RobotVersion(const RobotVersion&) = default;

    /// @brief 赋值运算符
    constexpr RobotVersion& operator=(const RobotVersion&) = default;

    /// @brief 从版本号数字创建 RobotVersion 对象
    /// @param major 主版本号 (0-9999)
    /// @param minor 次版本号 (0-9999)
    /// @param patch 补丁版本号 (0-9999), 默认为0
    /// @return RobotVersion 实例
    /// @throws std::invalid_argument 如果版本号超出有效范围
    constexpr RobotVersion(uint16_t major, uint16_t minor, uint16_t patch = 0)
        : major_(major), minor_(minor), patch_(patch) {
        if (major > 9999 || minor > 9999 || patch > 9999) {
            throw std::invalid_argument("Version components must be <= 9999");
        }
    }

    /// @brief 从大整数创建 RobotVersion 对象
    /// @param big_number 大整数
    /// @return RobotVersion 实例
    /// @throws std::invalid_argument 如果版本号超出有效范围
    static RobotVersion create(int big_number) {
        if (!is_valid(big_number)) {
            throw std::invalid_argument("Invalid version number: " + std::to_string(big_number));
        }

        // PPPPMMMMN
        uint16_t minor = big_number % 10;
        big_number /= 10;
        uint16_t major = big_number % 10000;
        uint16_t patch = big_number / 10000;

        return RobotVersion(major, minor, patch);
    }

    /// @brief 从环境变量 ROBOT_VERSION 获取版本号
    /// @return RobotVersion 实例
    /// @throws std::invalid_argument 如果环境变量不存在或版本号无效
    static RobotVersion from_env() {
        const char* env_version = std::getenv("ROBOT_VERSION");
        if (!env_version) {
            throw std::invalid_argument("Environment variable ROBOT_VERSION not set");
        }

        int version_number = std::stoi(env_version);
        return create(version_number);
    }

    /// @brief 判断版本号是否合法
    /// @param big_number 版本号数字
    /// @return 是否合法
    static constexpr bool is_valid(int big_number) {
        return big_number > 0 && big_number <= 999999999;
    }

    /// @brief 判断版本号是 major 版本系列
    /// @param major 主版本号
    /// @return 是否以 major 开头
    /// @note 例如：45, 100045, 4000045, 100000049 都是属于 4 代系列
    constexpr bool start_with(uint16_t major) const {
        return major_ == major;
    }

    /// @brief 判断版本号是 major.minor 版本系列
    /// @param major 主版本号
    /// @param minor 次版本号
    /// @return 是否以 major.minor 开头
    /// @note 例如：45, 100045, 4000045 都是属于 45 版本系列
    constexpr bool start_with(uint16_t major, uint16_t minor) const {
        return major_ == major && minor_ == minor;
    }

    /// @brief 获取版本号标准的 semantic version 字符串
    /// @return 版本号字符串 如 4.5.0, 10.5.20
    /// @note 例如：45 --> 4.5.0
    ///            100045 --> 4.5.1
    ///            2000105 --> 10.5.20
    /// @ref https://semver.org/lang/zh-CN/
    std::string version_name() const {
        std::ostringstream oss;
        oss << major_ << "." << minor_ << "." << patch_;
        return oss.str();
    }

    /// @brief 获取简洁版本号字符串
    /// @return 简洁版本号字符串，格式为 major.minor 或 major.minor.patch
    /// @note 当patch为0时，只显示major.minor格式；当patch不为0时，显示完整的三段式版本号
    /// @note 例如：45 --> "4.5"
    std::string version_name_short() const {
        std::ostringstream oss;
        oss << major_ << "." << minor_;
        if (patch_ != 0) {
            oss << "." << patch_;
        }
        return oss.str();
    }

    /// @brief 将版本号转换为字符串
    /// @return 版本号字符串 如 45, 100045, 4000045, 100000049
    std::string to_string() const {
        return std::to_string(minor_  + major_ * 10 + patch_*100000);
    }

    /// @brief 获取主版本号
    /// @return 主版本号
    constexpr uint16_t major() const { return major_; }

    /// @brief 获取次版本号
    /// @return 次版本号
    constexpr uint16_t minor() const { return minor_; }

    /// @brief 获取补丁版本号
    /// @return 补丁版本号
    constexpr uint16_t patch() const { return patch_; }

    /// @brief 相等运算符
    /// @param other 其他版本
    /// @return 是否相等
    constexpr bool operator==(const RobotVersion& other) const {
        return major_ == other.major_ && minor_ == other.minor_ && patch_ == other.patch_;
    }

    /// @brief 不等运算符
    /// @param other 其他版本
    /// @return 是否不等
    constexpr bool operator!=(const RobotVersion& other) const {
        return !(*this == other);
    }

    /// @brief 小于运算符
    /// @param other 其他版本
    /// @return 是否小于
    constexpr bool operator<(const RobotVersion& other) const {
        if (major_ != other.major_) return major_ < other.major_;
        if (minor_ != other.minor_) return minor_ < other.minor_;
        return patch_ < other.patch_;
    }

    /// @brief 小于等于运算符
    /// @param other 其他版本
    /// @return 是否小于等于
    constexpr bool operator<=(const RobotVersion& other) const {
        return *this < other || *this == other;
    }

    /// @brief 大于运算符
    /// @param other 其他版本
    /// @return 是否大于
    constexpr bool operator>(const RobotVersion& other) const {
        return !(*this <= other);
    }

    /// @brief 大于等于运算符
    /// @param other 其他版本
    /// @return 是否大于等于
    constexpr bool operator>=(const RobotVersion& other) const {
        return !(*this < other);
    }
};

/// @brief 输出版本号到流
/// @param os 输出流
/// @param version 版本号
/// @return 输出流引用
inline std::ostream& operator<<(std::ostream& os, const RobotVersion& version) {
    os << version.version_name();
    return os;
}

/// @namespace RobotVersions
/// @brief Leju 机器人预定义版本号常量
/// @details 包含所有支持的 Leju 机器人型号的版本号定义
namespace RobotVersions {

/**
 * @defgroup KuavoVersions Kuavo 系列版本
 * @brief Kuavo 系列机器人的预定义版本号
 * @{
 */

/*  LEJU Kuavo 4th Robots */
/// @brief Kuavo 4代 基础版 (短手基础版)
constexpr RobotVersion KUAVO4_BASE{4, 2, 0};

/// @brief Kuavo 4代 PRO 专业版 (灵巧手版本)
constexpr RobotVersion KUAVO4_PRO_DEXHAND{4, 5, 0};

/// @brief Kuavo 4代 UAE 版本
constexpr RobotVersion KUAVO4_UAE{4, 6, 0};

/// @brief Kuavo 4代 PRO 夹爪版
constexpr RobotVersion KUAVO4_PRO_GRIPPER{4, 7, 0};

/// @brief Kuavo 4代 EDU 教育版
constexpr RobotVersion KUAVO4_EDU{4, 9, 0};

/*  LEJU Kuavo 5th Robots */
/// @brief Kuavo 5代 基础版
constexpr RobotVersion KUAVO5_BASE{5, 2};
/** @} */

/**
 * @defgroup RobanVersions Roban 系列版本
 * @brief Roban 系列机器人的预定义版本号
 * @{
 */

/*  LEJU Roban 2th Robots */
/// @brief Roban 2代 基础版
constexpr RobotVersion ROBAN2_BASE{1, 4};
/** @} */

} // namespace RobotVersions

/************************************************/
/* Leju Legged Humanoid​ Robots                 */
/***********************************************/

/**
 * @defgroup RobotTypeMacros 机器人类型检测宏
 * @brief 用于快速检测机器人类型的宏定义
 * @{
 */

/// @defgroup KuavoLeggedMacros Kuavo 足式机器人检测宏
/// @brief 检测 Kuavo 系列足式机器人
/// @{

/// @brief 检测是否为 Kuavo 足式机器人 (4代或5代)
/// @param rb RobotVersion 对象
/// @return 如果是 Kuavo 4代或5代则返回 true
#define IS_KUAVO_LEGGED(rb) (rb.major() == 4 || rb.major() == 5)

/// @brief 检测是否为 Kuavo 4代足式机器人
/// @param rb RobotVersion 对象
/// @return 如果是 Kuavo 4代则返回 true
#define IS_KUAVO4_LEGGED(rb) (rb.major() == 4)

/// @brief 检测是否为 Kuavo 5代足式机器人
/// @param rb RobotVersion 对象
/// @return 如果是 Kuavo 5代则返回 true
#define IS_KUAVO5_LEGGED(rb) (rb.major() == 5)

/// @brief 检测是否为 Kuavo 4代 PRO 系列足式机器人
/// @param rb RobotVersion 对象
/// @return 如果是 Kuavo 4代 PRO 系列 (45, 46, 47, 48, 49) 则返回 true
#define IS_KUAVO4PRO_LEGGED(rb) (rb.major() == 4 && (rb.minor() >= 5))
/// @}

/// @defgroup RobanLeggedMacros Roban 足式机器人检测宏
/// @brief 检测 Roban 系列足式机器人
/// @{

/// @brief 检测是否为 Roban 足式机器人
/// @param rb RobotVersion 对象
/// @return 如果是 Roban 系列则返回 true
#define IS_ROBAN_LEGGED(rb) (rb.major() == 1)

/// @brief 检测是否为 Roban 2代足式机器人
/// @param rb RobotVersion 对象
/// @return 如果是 Roban 2代则返回 true
#define IS_ROBAN2_LEGGED(rb) (rb.major() == 1)

/// @brief 检测是否为 Roban 2.1 代足式机器人 (BIGNUMBER < 17, 即 minor < 7)
/// @param rb RobotVersion 对象
/// @return 如果是 Roban 2.1 代则返回 true
#define IS_ROBAN2_1_LEGGED(rb) (rb.major() == 1 && rb.minor() < 7)

/// @brief 检测是否为 Roban 2.2 代足式机器人 (BIGNUMBER >= 17, 即 minor >= 7)
/// @param rb RobotVersion 对象
/// @return 如果是 Roban 2.2 代则返回 true
#define IS_ROBAN2_2_LEGGED(rb) (rb.major() == 1 && rb.minor() >= 7)
/// @}

/// @defgroup KuavoWheeledMacros Kuavo 轮式机器人检测宏
/// @brief 检测 Kuavo 系列轮式机器人
/// @{

/// @brief 检测是否为 Kuavo 轮式机器人
/// @param rb RobotVersion 对象
/// @return 如果是 Kuavo 轮式系列 (主版本号为 6) 则返回 true
#define IS_KUAVO_WHEELED(rb) (rb.major() == 6)
/// @}

////////////////////////////////////////////////////

/************************************************/
/* Leju Wheeled Humanoid​ Robots                 */
/***********************************************/

/// @defgroup BrandMacros 产品系列检测宏
/// @brief 检测机器人产品系列
/// @{

/// @brief 检测是否为 Kuavo 系列 (足式或轮式)
/// @param rb RobotVersion 对象
/// @return 如果是 Kuavo 系列 (足式或轮式) 则返回 true
#define IS_KUAVO(rb) (IS_KUAVO_LEGGED(rb) || IS_KUAVO_WHEELED(rb))

/// @brief 检测是否为 Roban 系列
/// @param rb RobotVersion 对象
/// @return 如果是 Roban 系列则返回 true
#define IS_ROBAN(rb) (IS_ROBAN_LEGGED(rb))
/// @}

/** @} */ // end of RobotTypeMacros

} // namespace leju