#include "RLAgent.h"
#include <LittleFS.h>
#include <Arduino.h>
#include <SPI.h>

namespace rl {

RLAgent::RLAgent(const char *path) : _path(path) {
  // init q with zeros
  for (int x = 0; x < BINS; ++x)
    for (int y = 0; y < BINS; ++y)
      for (int a = 0; a < ACTIONS; ++a)
        _q[x][y][a] = 0.0f;
}

void RLAgent::begin() {
  // ensure storage is mounted (STORAGE is LittleFS via Config.h)
  STORAGE.begin(true);
  load();
}

int RLAgent::selectAction(int sx, int sy, float epsilon) {
  // bounds check
  sx = constrain(sx, 0, BINS - 1);
  sy = constrain(sy, 0, BINS - 1);
  if (random(1000) < (int)(epsilon * 1000)) {
    return random(ACTIONS);
  }
  // greedy
  int best = 0;
  float bestv = _q[sx][sy][0];
  for (int a = 1; a < ACTIONS; ++a) {
    if (_q[sx][sy][a] > bestv) {
      bestv = _q[sx][sy][a];
      best = a;
    }
  }
  return best;
}

void RLAgent::update(int sx, int sy, int action, int nsx, int nsy, float reward, float alpha, float gamma) {
  // bounds check
  sx = constrain(sx, 0, BINS - 1);
  sy = constrain(sy, 0, BINS - 1);
  nsx = constrain(nsx, 0, BINS - 1);
  nsy = constrain(nsy, 0, BINS - 1);

  float cur = _q[sx][sy][action];
  float maxNext = _q[nsx][nsy][0];
  for (int a = 1; a < ACTIONS; ++a) if (_q[nsx][nsy][a] > maxNext) maxNext = _q[nsx][nsy][a];
  float target = reward + gamma * maxNext;
  _q[sx][sy][action] = cur + alpha * (target - cur);
}

void RLAgent::discretize(float dx, float dy, int &sx, int &sy) const {
  // dx/dy are expected in degrees or normalized units where 0 is centered; we map continuous error into bins
  // We'll treat dx,dy roughly in range [-90,90] for mapping; clamp if needed
  const float MAXERR = 90.0f;
  float nx = constrain(dx, -MAXERR, MAXERR) / MAXERR; // -1..1
  float ny = constrain(dy, -MAXERR, MAXERR) / MAXERR;
  // map -1..1 -> 0..BINS-1
  sx = (int)round((nx + 1.0f) * 0.5f * (BINS - 1));
  sy = (int)round((ny + 1.0f) * 0.5f * (BINS - 1));
  sx = constrain(sx, 0, BINS - 1);
  sy = constrain(sy, 0, BINS - 1);
}

bool RLAgent::save() {
  File f = STORAGE.open(_path.c_str(), FILE_WRITE);
  if (!f) return false;
  // simple csv: rows of BINS*BINS lines, each with ACTIONS comma separated
  for (int x = 0; x < BINS; ++x) {
    for (int y = 0; y < BINS; ++y) {
      for (int a = 0; a < ACTIONS; ++a) {
        f.print(_q[x][y][a], 6);
        if (a + 1 < ACTIONS) f.print(',');
      }
      f.println();
    }
  }
  f.close();
  return true;
}

bool RLAgent::load() {
  if (!STORAGE.exists(_path.c_str())) return false;
  File f = STORAGE.open(_path.c_str(), FILE_READ);
  if (!f) return false;
  String line;
  for (int x = 0; x < BINS; ++x) {
    for (int y = 0; y < BINS; ++y) {
      if (!f.available()) { f.close(); return false; }
      line = f.readStringUntil('\n');
      int start = 0;
      for (int a = 0; a < ACTIONS; ++a) {
        int comma = line.indexOf(',', start);
        String token;
        if (comma == -1) token = line.substring(start);
        else token = line.substring(start, comma);
        _q[x][y][a] = token.toFloat();
        if (comma == -1) break;
        start = comma + 1;
      }
    }
  }
  f.close();
  return true;
}

float RLAgent::getQ(int sx, int sy, int action) const {
  sx = constrain(sx, 0, BINS - 1);
  sy = constrain(sy, 0, BINS - 1);
  action = constrain(action, 0, ACTIONS - 1);
  return _q[sx][sy][action];
}

} // namespace rl
