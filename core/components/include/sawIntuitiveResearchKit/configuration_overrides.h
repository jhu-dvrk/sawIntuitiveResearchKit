/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2026-11-03

  (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/* Configuration overrides shared by the system loader and arm components. */
#ifndef _dvrk_configuration_overrides_h
#define _dvrk_configuration_overrides_h

#include <cisstCommon/cmnLogger.h>
#include <json/json.h>
#include <cstdlib>
#include <list>
#include <set>
#include <string>

namespace dvrk {
    // Merge objects recursively; arrays, scalars and null replace whole values.
    // An absent override leaves the existing arm-file behavior unchanged.
    inline void apply_arm_overrides(Json::Value & config, const Json::Value & overrides)
    {
        if (overrides.isNull()) {
            return;
        }
        if (!overrides.isObject() || !config.isObject()) {
            CMN_LOG_INIT_ERROR << "Configuration overrides: " << "arm_overrides and the arm configuration must be JSON objects" << std::endl;
            exit(EXIT_FAILURE);
        }
        for (const auto & key : overrides.getMemberNames()) {
            if (config[key].isObject() && overrides[key].isObject()) {
                apply_arm_overrides(config[key], overrides[key]);
            } else {
                config[key] = overrides[key];
            }
        }
    }

    inline bool supports_arm_overrides(const std::string & type)
    {
        return type == "PSM" || type == "PSM_DERIVED"
            || type == "ECM" || type == "ECM_DERIVED"
            || type == "MTM" || type == "MTM_DERIVED";
    }

    // Apply CLI selections before deserializing the system or creating any
    // components. The arm-file defaults are merged later by the arm itself.
    inline void apply_tool_overrides(Json::Value & config,
                                     const std::list<std::string> & selections)
    {
        if (!config.isObject()) {
            CMN_LOG_INIT_ERROR << "Configuration overrides: " << "system configuration must be a JSON object" << std::endl;
            exit(EXIT_FAILURE);
        }
        auto & arms = config["arms"];
        if (!arms.isNull() && !arms.isArray()) {
            CMN_LOG_INIT_ERROR << "Configuration overrides: " << "system arms must be a JSON array" << std::endl;
            exit(EXIT_FAILURE);
        }
        for (const auto & arm : arms) {
            if (!arm.isObject() || !arm["name"].isString() || !arm["type"].isString()) {
                CMN_LOG_INIT_ERROR << "Configuration overrides: " << "system arms must be objects with string name and type fields" << std::endl;
                exit(EXIT_FAILURE);
            }
            if (arm.isMember("arm_overrides")) {
                if (!arm["arm_overrides"].isObject()) {
                    CMN_LOG_INIT_ERROR << "Configuration overrides: " << "arm_overrides for " << arm["name"].asString() << " must be a JSON object" << std::endl;
                    exit(EXIT_FAILURE);
                }
                if (!supports_arm_overrides(arm["type"].asString())) {
                    CMN_LOG_INIT_ERROR << "Configuration overrides: " << "arm_overrides is only supported for native or derived PSM, ECM and MTM arms" << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
        std::set<std::string> selected;
        for (const auto & selection : selections) {
            const auto separator = selection.find('=');
            if (separator == std::string::npos || separator == 0
                || separator + 1 == selection.size()
                || selection.find('=', separator + 1) != std::string::npos) {
                CMN_LOG_INIT_ERROR << "Configuration overrides: " << "Invalid --tool '" << selection << "'; expected ARM=TOOL" << std::endl;
                exit(EXIT_FAILURE);
            }
            const auto name = selection.substr(0, separator);
            const auto tool = selection.substr(separator + 1);
            if (!selected.insert(name).second) {
                CMN_LOG_INIT_ERROR << "Configuration overrides: " << "Duplicate --tool selection for arm " << name << std::endl;
                exit(EXIT_FAILURE);
            }
            Json::Value * target = nullptr;
            for (auto & arm : arms) {
                if (arm["name"].asString() == name) {
                    if (target) {
                        CMN_LOG_INIT_ERROR << "Configuration overrides: " << "Ambiguous --tool arm name " << name << std::endl;
                        exit(EXIT_FAILURE);
                    }
                    target = &arm;
                }
            }
            if (!target) {
                CMN_LOG_INIT_ERROR << "Configuration overrides: " << "Unknown --tool arm " << name << std::endl;
                exit(EXIT_FAILURE);
            }
            const auto type = (*target)["type"].asString();
            if (type != "PSM" && type != "PSM_DERIVED") {
                CMN_LOG_INIT_ERROR << "Configuration overrides: " << "--tool requires a native or derived PSM arm: " << name << std::endl;
                exit(EXIT_FAILURE);
            }
            auto & overrides = (*target)["arm_overrides"];
            overrides["tool_detection"] = "FIXED";
            overrides["tool"] = tool;
        }
    }
}

#endif // _dvrk_configuration_overrides_h
