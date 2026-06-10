#pragma once

#include "core/status.h"

#include <nlohmann/json.hpp>

#include <string>

namespace bt {

class DesktopUiController {
public:
    virtual ~DesktopUiController() = default;

    virtual nlohmann::json GetStateJson() = 0;
    virtual nlohmann::json HandleCommand(const std::string& command, const nlohmann::json& payload) = 0;
    virtual void OnUiClosed() {}
};

Status RunDesktopUi(DesktopUiController& controller, const std::string& title);

} // namespace bt
