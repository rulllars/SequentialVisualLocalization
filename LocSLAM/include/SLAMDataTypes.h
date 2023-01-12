/**
* This file is part of LocSLAM.
*
* Copyright (C) 2021 Lars Hammarstrand <lars.hammarstrand at chalmers dot se> (Chalmers University of Technology)
* For more information see <https://github.com/rulllars/SequentialLocalization>
*
* LocSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LocSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with locSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SLAMDATATYPES_H
#define SLAMDATATYPES_H

#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/se3quat.h"

namespace LocSLAM
{

    struct Correspondence2d3d
    {
        Eigen::Vector2d imagePoint;
        Eigen::Vector3d mapPoint;

        Eigen::Matrix2d Information = 1 * Eigen::Matrix2d::Identity();

        bool isBad = false;
    };

    struct FixedMapMatches
    {
        g2o::SE3Quat Tcf;

        double sigmaPriorPosition;
        double sigmaPriorHeading;

        unsigned int nMatches = 0;
        std::vector<Correspondence2d3d> vMatches;

        double sigmaFeature = 6.0; // Standard deviation of re-projection in pixels
        Eigen::Matrix2d informationMatrix = 1.0 / pow(sigmaFeature, 2.0) * Eigen::Matrix2d::Identity();
    };

}; // namespace LocSLAM

#endif // SLAMDATATYPES_H
