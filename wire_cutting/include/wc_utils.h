#ifndef WC_UTILS_H
#define WC_UTILS_H

#pragma once

#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP


std::vector<tesseract_common::VectorIsometry3d> loadToolPosesFromPrg(const std::string& file);
tesseract_common::VectorIsometry3d loadToolPoses();

#endif