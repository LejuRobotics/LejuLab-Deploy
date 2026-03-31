#pragma once

#ifndef KUAVO_COMMON_H
#define KUAVO_COMMON_H

#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/utils.h"
#include "kuavo_common/common/json_config_reader.hpp"

namespace HighlyDynamic
{
using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
class KuavoCommon
    {
    public:
        /**
         * @brief Get the Instance object
         * 
         * @param robot_name            "kuavo_v45", "kuavo_v42"....
         * @param kuavo_assets_path     "/home/lab/kuavo_assets"
         * 
         * @note config file path: $kuavo_assets_path/$robot_name/kuavo.json
         *       example: /home/lab/kuavo_assets/kuavo_v45/kuavo.json
         * 
         * @return KuavoCommon& 
         */
        static KuavoCommon &getInstance(const std::string& robot_name, const std::string& kuavo_assets_path);
        static KuavoCommon *getInstancePtr(const std::string& robot_name, const std::string& kuavo_assets_path);

        virtual ~KuavoCommon();

        KuavoCommon(const KuavoCommon &) = delete;
        KuavoCommon &operator=(const KuavoCommon &) = delete;
        inline const KuavoSettings &getKuavoSettings() const { return kuavo_settings_; }
        JSONConfigReader *getRobotConfig() const { return robot_config_; }
    private:
        KuavoCommon(const std::string& robot_name, const std::string& kuavo_assets_path);
        static std::shared_ptr<KuavoCommon> instance;

    private:
        KuavoSettings kuavo_settings_;
        JSONConfigReader *robot_config_;
    };

}
#endif