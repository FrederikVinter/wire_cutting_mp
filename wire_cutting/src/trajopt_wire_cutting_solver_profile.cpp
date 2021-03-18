/**
 * @file trajopt_default_solver_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date December 13, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "trajopt_wire_cutting_solver_profile.h"

TrajOptWireCuttingSolverProfile::TrajOptWireCuttingSolverProfile(const tinyxml2::XMLElement& xml_element)
{  
    /*const tinyxml2::XMLElement* improve_ratio_threshold_element = xml_element.FirstChildElement("ImproveRatioTreshold");
    const tinyxml2::XMLElement* min_trust_box_size_element = xml_element.FirstChildElement("MinTrustBoxSize");
    const tinyxml2::XMLElement* min_approx_improve_element = xml_element.FirstChildElement("MinApproxImprove");
    const tinyxml2::XMLElement* min_approx_improve_frac_element = xml_element.FirstChildElement("MinApproxImproveFrac");
    const tinyxml2::XMLElement* max_iter_element = xml_element.FirstChildElement("MaxIterations");
    const tinyxml2::XMLElement* trust_shrink_ratio_element = xml_element.FirstChildElement("TrustShrinkRatio");
    const tinyxml2::XMLElement* trust_expand_ratio_element = xml_element.FirstChildElement("TrustExpandRatio");
    const tinyxml2::XMLElement* cnt_tolerance_element = xml_element.FirstChildElement("ConstraintTolerance");
    const tinyxml2::XMLElement* max_merit_coeff_increases_element = xml_element.FirstChildElement("MaxMeritCoeffIncreases");
    const tinyxml2::XMLElement* merit_coeff_increase_ratio_element = xml_element.FirstChildElement("MeritCoeffIncreaseRatio");
    const tinyxml2::XMLElement* initial_merit_error_coeff = xml_element.FirstChildElement("InitialMeritErrorCoeff");
    const tinyxml2::XMLElement* inflate_constraints_individually_element = xml_element.FirstChildElement("InflateConstraintsIndividually"); 
    const tinyxml2::XMLElement* trust_box_size_element = xml_element.FirstChildElement("TrustBoxSize");
 
    if (longest_valid_seg_length_element)
    {
        std::string long_valid_seg_len_string;
        status = tesseract_common::QueryStringText(longest_valid_seg_length_element, long_valid_seg_len_string);
        if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("TrajoptCompositeProfile: Error parsing LongestValidSegmentLength string");

        if (!tesseract_common::isNumeric(long_valid_seg_len_string))
        throw std::runtime_error("TrajoptCompositeProfile: LongestValidSegmentLength is not a numeric values.");

        tesseract_common::toNumeric<double>(long_valid_seg_len_string, longest_valid_segment_length);
    }*/
}

void TrajOptWireCuttingSolverProfile::apply(trajopt::ProblemConstructionInfo& pci) const
{
    pci.basic_info.convex_solver = convex_solver;
    pci.opt_info = opt_info;
}

tinyxml2::XMLElement* TrajOptWireCuttingSolverProfile::toXML(tinyxml2::XMLDocument& doc) const
{

}


