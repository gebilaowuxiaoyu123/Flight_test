#pragma once

#include <string>

// PX4 nav_state（读数）→ 人看的模式名
std::string modeNameOf(int nav_state);

// 按钮/命令里的模式名 → PX4 主模式号（DO_SET_MODE 的 param2），0 表示未知。
// 注意这套编号和 nav_state 不是同一套：主模式只有 1~7。
int mainModeOf(const std::string & name);
