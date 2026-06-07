#pragma once

namespace stop_n_go::core
{

/// Controls how much Stop-N-Go search progress is printed to stdout.
enum class LogLevel
{
  Quiet,
  Summary,
  Debug,
};

}  // namespace stop_n_go::core
