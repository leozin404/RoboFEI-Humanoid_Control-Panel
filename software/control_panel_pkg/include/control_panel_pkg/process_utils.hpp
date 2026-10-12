#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace control_panel_pkg::process_utils {

// One external process the panel can start/stop (a `ros2 launch ...` or a
// `ros2 run ...`).
struct ProcSpec {
  ProcSpec() = default;
  ProcSpec(std::string n, std::vector<std::string> a, std::string m = "")
      : name(std::move(n)), argv(std::move(a)), match(std::move(m)) {}

  std::string name;                // also names the log: /tmp/panel_<name>.log
  std::vector<std::string> argv;   // e.g. {"ros2","launch","robot_bringup","x.launch.py"}
  std::string match;               // pgrep/pkill pattern; empty = last argv token

  std::string pattern() const {
    if (!match.empty()) return match;
    return argv.empty() ? std::string() : argv.back();
  }
};

// `ros2 launch robot_bringup <launchFile> [extraArgs...]`. The pattern is
// always the launch file, so extra args (e.g. team_number:=7) never matter
// when checking or stopping it.
inline ProcSpec launchSpec(const std::string &name, const std::string &launchFile,
                           const std::vector<std::string> &extraArgs = {}) {
  ProcSpec spec(name, {"ros2", "launch", "robot_bringup", launchFile}, launchFile);
  spec.argv.insert(spec.argv.end(), extraArgs.begin(), extraArgs.end());
  return spec;
}

inline std::vector<std::string> splitWords(const std::string &s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ' ' || c == '\t') {
      if (!cur.empty()) { out.push_back(cur); cur.clear(); }
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

inline std::string expandHome(const std::string &path) {
  if (!path.empty() && path[0] == '~') {
    const char *home = std::getenv("HOME");
    if (home != nullptr) return std::string(home) + path.substr(1);
  }
  return path;
}

inline bool repoDirExists(const std::string &repoDir) {
  struct stat st{};
  const std::string dir = expandHome(repoDir);
  return stat(dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// "[d]ecision_bringup.launch.py": the bracket keeps pgrep/pkill from matching
// the `sh -c` wrapper that std::system spawns, whose own command line would
// otherwise contain the very pattern being searched.
inline std::string safePattern(const std::string &s) {
  if (s.empty()) return s;
  return std::string("[") + s[0] + "]" + s.substr(1);
}

inline bool isRunning(const ProcSpec &spec) {
  if (spec.pattern().empty()) return false;
  const std::string cmd = "pgrep -f '" + safePattern(spec.pattern()) + "' > /dev/null";
  const int rc = std::system(cmd.c_str());
  return rc == 0;
}

// Same effect as Ctrl+C in the terminal running the process.
inline void stop(const ProcSpec &spec) {
  if (spec.pattern().empty()) return;
  const std::string cmd = "pkill -INT -f '" + safePattern(spec.pattern()) + "'";
  const int rc = std::system(cmd.c_str());
  (void)rc;
}

// Starts spec detached, from repoDir (`control` resolves its motion JSON
// relative to cwd). Uses fork/exec instead of `sh -c "... &"` on purpose: a
// non-interactive shell starts async children with SIGINT ignored, and Python
// (ros2 launch) then never sees the Ctrl+C that stop() sends. Double fork
// leaves no zombie behind in the calling node.
// Returns false, and starts nothing, if repoDir does not exist: that failure
// used to happen inside the child, after start() had already reported success.
inline bool start(const std::string &repoDir, const ProcSpec &spec) {
  if (spec.argv.empty() || !repoDirExists(repoDir)) return false;

  // Everything the child needs is built BEFORE fork (no allocation after it).
  const std::string dir = expandHome(repoDir);
  const std::string logPath = "/tmp/panel_" + spec.name + ".log";
  std::vector<char *> args;
  for (const auto &a : spec.argv) args.push_back(const_cast<char *>(a.c_str()));
  args.push_back(nullptr);

  const pid_t first = fork();
  if (first < 0) return false;

  if (first == 0) {
    const pid_t second = fork();
    if (second != 0) _exit(second < 0 ? 1 : 0);

    setsid();
    const int log = open(logPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log >= 0) {
      dup2(log, STDOUT_FILENO);
      dup2(log, STDERR_FILENO);
      close(log);
    }
    const int in = open("/dev/null", O_RDONLY);
    if (in >= 0) {
      dup2(in, STDIN_FILENO);
      close(in);
    }

    static const char kCdErr[] = "panel: cannot chdir to repo_dir\n";
    static const char kExecErr[] = "panel: exec failed\n";
    if (chdir(dir.c_str()) != 0) {
      const ssize_t w = write(STDERR_FILENO, kCdErr, sizeof(kCdErr) - 1);
      (void)w;
      _exit(127);
    }
    execvp(args[0], args.data());
    const ssize_t w = write(STDERR_FILENO, kExecErr, sizeof(kExecErr) - 1);
    (void)w;
    _exit(127);
  }

  int status = 0;
  waitpid(first, &status, 0);
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

// The "Robot Bringup" button: starts the vital nodes as separate processes
// (like the team does today, one terminal each) and stops them all together.
class BringupGroup {
 public:
  using Clock = std::chrono::steady_clock;

  BringupGroup(std::string repoDir, std::vector<ProcSpec> members,
               std::chrono::milliseconds grace = std::chrono::milliseconds(5000))
      : repo_dir_(std::move(repoDir)), members_(std::move(members)), grace_(grace) {}

  // True while the group counts as up. Self-heals: if every member died
  // (crash, manual Ctrl+C) the flag drops back to false, but only after a
  // grace period, because right after toggle() the children may not have
  // exec'd yet and would look "not running".
  bool active() {
    if (active_ && (Clock::now() - since_) > grace_ && !anyRunning()) active_ = false;
    return active_;
  }

  // Returns the new state (true = now on). Turning on starts only members
  // that are not already running, so nothing gets launched twice.
  bool toggle() {
    if (active()) {
      for (const auto &m : members_) stop(m);
      active_ = false;
    } else {
      failed_.clear();
      for (const auto &m : members_) {
        if (!isRunning(m) && !start(repo_dir_, m)) failed_.push_back(m.name);
      }
      // If nothing came up, do not claim the bringup is active: that would
      // block the decision toggle for a bringup that does not exist.
      active_ = failed_.size() < members_.size();
      since_ = Clock::now();
    }
    return active_;
  }

 // Names of members that could not be started by the last toggle() to ON.
  const std::vector<std::string> &failedToStart() const { return failed_; }

 private:
  bool anyRunning() const {
    for (const auto &m : members_) {
      if (isRunning(m)) return true;
    }
    return false;
  }

  std::string repo_dir_;
  std::vector<ProcSpec> members_;
  std::chrono::milliseconds grace_;
  bool active_ = false;
  std::vector<std::string> failed_;
  Clock::time_point since_{};
};

}  // namespace control_panel_pkg::process_utils
