#pragma once
#include <Arduino.h>
#include <FS.h>
#include "../../include/Config.h"

namespace rl {

// Small tabular Q-learning agent for discrete 2D state (dx,dy) and a few actions.
class RLAgent {
public:
  // state bins per axis (odd number so 0 is center)
  static constexpr int BINS = 9; // -4..4
  static constexpr int ACTIONS = 5; // left,right,up,down,stay

  RLAgent(const char *path = "/database/rl_q.csv");
  void begin();

  // epsilon-greedy action selection
  int selectAction(int sx, int sy, float epsilon = 0.1f);

  // update Q-table (simple Q-learning): Q[s,a] += alpha*(r + gamma*maxQ(next) - Q[s,a])
  void update(int sx, int sy, int action, int nsx, int nsy, float reward, float alpha = 0.2f, float gamma = 0.9f);

  // convenience: discretize continuous error into bin index
  void discretize(float dx, float dy, int &sx, int &sy) const;

  // save/load to LittleFS
  bool save();
  bool load();

  // direct access for diagnostics
  float getQ(int sx, int sy, int action) const;

private:
  String _path;
  float _q[BINS][BINS][ACTIONS];
};

} // namespace rl
