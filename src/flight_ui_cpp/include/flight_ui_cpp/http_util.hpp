#pragma once

#include <string>

#include <json/json.h>

std::string jsonString(const Json::Value & value);
bool readFile(const std::string & path, std::string & content);
std::string contentTypeOf(const std::string & path);
