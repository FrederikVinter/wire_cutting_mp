#ifndef WC_UTILS_H
#define WC_UTILS_H

#pragma once

#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_client.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP


tesseract_common::VectorIsometry3d loadToolPosesFromPrg(const std::string& file);

#endif