// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_system_utils/profiler.h"

namespace srrg2_core {

  bool Profiler::enable_logging = false;

  Profiler::~Profiler() {
    if (Profiler::enable_logging) {
      // show complete time statistics for all registered timers
      for (auto& iterator : _runtimes_per_module) {
        const std::vector<double>& runtimes_seconds(iterator.second);
        const double total_runtime_seconds =
          std::accumulate(runtimes_seconds.begin(), runtimes_seconds.end(), 0.0);
        const double mean_runtime_seconds = total_runtime_seconds / runtimes_seconds.size();
        std::cerr << "[" << this << "][" << iterator.first
                  << "] total (s): " << total_runtime_seconds
                  << " | samples (#): " << runtimes_seconds.size()
                  << " | mean (ms): " << mean_runtime_seconds * 1e3 << std::endl;
      }
    }
    _runtimes_per_module.clear();
  }

  Profiler::Timer::Timer(Profiler* handle_, const std::string& name_) :
    handle(handle_),
    name(name_),
    start_time_seconds(getTime()) {
  }

  Profiler::Timer::~Timer() {
    assert(handle);

    // store livetime in scope in the statistics buffer
    const double duration_seconds = getTime() - start_time_seconds;
    assert(duration_seconds >= 0);
    auto iterator = handle->_runtimes_per_module.find(name);
    if (iterator != handle->_runtimes_per_module.end()) {
      iterator->second.push_back(duration_seconds);
    } else {
      handle->_runtimes_per_module.insert(
        std::make_pair(name, std::vector<double>(1, duration_seconds)));
    }
  }

} // namespace srrg2_core
