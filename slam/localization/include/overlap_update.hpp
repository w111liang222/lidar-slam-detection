// SPDX-License-Identifier: BSD-2-Clause

#ifndef OVERLAP_UPDATER_HPP
#define OVERLAP_UPDATER_HPP

#include "slam_base.h"
#include "overlap_removal.hpp"
#include "overlap_merge.hpp"
#include "backend_api.h"
#include "Logger.h"

class OverlapUpdator {
public:

  /**
   * @brief constructor
   */
  OverlapUpdator() {

  }

  std::vector<std::shared_ptr<KeyFrame>> detect(const std::vector<std::shared_ptr<KeyFrame>> &keyframes) {
    return overlap_remover.detect(keyframes);
  }

  std::vector<std::shared_ptr<KeyFrame>> update(const std::vector<std::shared_ptr<KeyFrame>> &keyframes, const std::vector<std::shared_ptr<KeyFrame>> &regions) {
    return overlap_remover.remove(keyframes, regions);
  }

  std::vector<Overlap::Ptr> post_process(const std::vector<std::shared_ptr<KeyFrame>> &keyframes) {
    std::vector<EdgeType> connection;
    graph_get_edges(connection);

    // build connection map
    std::map<int, std::set<int>> connection_map;
    for(size_t i = 0; i < connection.size(); i++) {
      if (connection_map.find(connection[i].prev) == connection_map.end()) {
        connection_map[connection[i].prev] = std::set<int>();
      }
      if (connection_map.find(connection[i].next) == connection_map.end()) {
        connection_map[connection[i].next] = std::set<int>();
      }
      connection_map[connection[i].prev].insert(connection[i].next);
      connection_map[connection[i].next].insert(connection[i].prev);
    }

    // build keyframe id map
    std::map<int, int> keyframe_id_map;
    for (size_t i = 0; i < keyframes.size(); i++) {
      keyframe_id_map[keyframes[i]->mId] = i;
    }

    std::vector<std::shared_ptr<KeyFrame>> alone_frames;
    for(auto &conn : connection_map) {
      if (conn.second.size() <= 2) {
        alone_frames.push_back(keyframes[keyframe_id_map[conn.first]]);
      }
    }

    return overlap_detector.detect(keyframes, alone_frames);
  }

private:
  OverlapRemover overlap_remover;
  OverlapDetector overlap_detector;
};

#endif  // OVERLAP_UPDATER_HPP
