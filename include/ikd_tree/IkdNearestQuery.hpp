/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Project-level nearest-neighbor query wrapper backed by ikd-tree.
 */

#ifndef IKD_NEAREST_QUERY_H
#define IKD_NEAREST_QUERY_H

#include <vector>

#include "support/common_lib.h"

class IkdNearestQuery final {
public:
    bool searchKnn(const PointType &query_point_world,
                   int k_nearest,
                   PointVector &nearest_points,
                   std::vector<float> &distance_sq) const {
        ikdtree.Nearest_Search(query_point_world, k_nearest, nearest_points, distance_sq);
        return nearest_points.size() >= static_cast<size_t>(k_nearest);
    }
};

#endif // IKD_NEAREST_QUERY_H
