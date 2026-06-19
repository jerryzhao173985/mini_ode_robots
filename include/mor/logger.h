/* mini_ode_robots — logger.h
 *
 * A minimal, header-only CSV logger for headless experiments: define columns once,
 * then write one row per step. lpzrobots' guilogger is GUI/IPC-heavy and was dropped;
 * this is the dependency-free essence — run a sim, record sensors/trajectory, analyze
 * the CSV with any tool. (~30 lines, only <cstdio>.)
 *
 *   Logger log("traj.csv", {"x","y","ray"});
 *   for (...) { sim.step(); log.log(sim.getTime(), { p.x(), p.y(), ray.get() }); }
 */
#ifndef MOR_LOGGER_H
#define MOR_LOGGER_H

#include <cstdio>
#include <string>
#include <vector>

namespace mor {

class Logger {
public:
  Logger(const std::string& path, const std::vector<std::string>& columns) {
    f = std::fopen(path.c_str(), "w");
    if (f) {
      std::fprintf(f, "time");
      for (const auto& c : columns) std::fprintf(f, ",%s", c.c_str());
      std::fprintf(f, "\n");
      ncols = (int)columns.size();
    }
  }
  ~Logger() { if (f) std::fclose(f); }
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  /// write one row: time then the values (must match the column count)
  void log(double time, const std::vector<double>& values) {
    if (!f) return;
    std::fprintf(f, "%g", time);
    for (double x : values) std::fprintf(f, ",%g", x);
    std::fprintf(f, "\n");
    ++rows;
  }
  bool ok()   const { return f != nullptr; }  ///< false if the file could not be opened
  long count() const { return rows; }          ///< rows written so far

private:
  std::FILE* f = nullptr;
  int  ncols = 0;
  long rows  = 0;
};

} // namespace mor
#endif
