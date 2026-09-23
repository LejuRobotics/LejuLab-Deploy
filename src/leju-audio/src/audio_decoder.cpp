#include "audio_decoder.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace leju {
namespace audio {

namespace {
bool fileExists(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}
}  // namespace

bool decodeFile(const std::string& path, const DecodeParams& params,
                std::vector<uint8_t>& out, std::string& error_msg) {
  out.clear();
  if (!fileExists(path)) {
    error_msg = "file not found: " + path;
    return false;
  }

  int pipefd[2];
  if (pipe(pipefd) != 0) {
    error_msg = std::string("pipe() failed: ") + std::strerror(errno);
    return false;
  }

  pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    error_msg = std::string("fork() failed: ") + std::strerror(errno);
    return false;
  }

  if (pid == 0) {
    // 子进程：stdout → pipe，exec ffmpeg
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[1]);
    // stderr 静默（避免污染节点日志），ffmpeg 错误用退出码判断
    int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      dup2(devnull, STDERR_FILENO);
      close(devnull);
    }

    std::string rate = std::to_string(params.sample_rate);
    std::string chans = std::to_string(params.channels);
    std::string vol;
    std::vector<const char*> argv;
    argv.push_back("ffmpeg");
    argv.push_back("-hide_banner");
    argv.push_back("-loglevel");
    argv.push_back("error");
    argv.push_back("-i");
    argv.push_back(path.c_str());
    if (params.volume != 100) {
      vol = "volume=" + std::to_string(params.volume / 100.0);
      argv.push_back("-af");
      argv.push_back(vol.c_str());
    }
    argv.push_back("-f");
    argv.push_back("s16le");
    argv.push_back("-acodec");
    argv.push_back("pcm_s16le");
    argv.push_back("-ar");
    argv.push_back(rate.c_str());
    argv.push_back("-ac");
    argv.push_back(chans.c_str());
    argv.push_back("-");  // 输出到 stdout
    argv.push_back(nullptr);
    execvp("ffmpeg", const_cast<char* const*>(argv.data()));
    _exit(127);  // exec 失败
  }

  // 父进程：读 pipe
  close(pipefd[1]);
  uint8_t buf[16384];
  ssize_t n;
  while ((n = read(pipefd[0], buf, sizeof(buf))) > 0) {
    out.insert(out.end(), buf, buf + n);
  }
  close(pipefd[0]);

  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (code == 127) {
      error_msg = "ffmpeg not found (install ffmpeg)";
    } else {
      error_msg = "ffmpeg failed (exit=" + std::to_string(code) + ")";
    }
    out.clear();
    return false;
  }
  if (out.empty()) {
    error_msg = "ffmpeg produced no audio data";
    return false;
  }
  return true;
}

}  // namespace audio
}  // namespace leju
