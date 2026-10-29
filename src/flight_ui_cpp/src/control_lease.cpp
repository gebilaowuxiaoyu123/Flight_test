// control_lease.cpp —— 控制权租约（网页后端 vs 终端摇杆守护，同一时刻只有一方发杆量）
// 对外：control_lease::acquire / release / read
// 约定：/tmp/flight_control.lease 内容 <owner> <prio> <pid> <unix_ms>，tmp+rename 原子写

#include "flight_ui_cpp/control_lease.hpp"

#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace control_lease {
namespace {

long long nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool pidAlive(int pid)
{
    if (pid <= 0) {
        return false;
    }
    // kill(pid, 0) 只做存在性检查，不发信号
    return ::kill(pid, 0) == 0 || errno == EPERM;
}

void writeEntry(const std::string & owner, int prio, int pid)
{
    // 临时文件名带本进程 PID：网页后端(本文件)和终端摇杆守护(stick_keeper.py)
    // 用同一份租约，若共用一个固定的 .tmp，两边同时写会把内容交织进同一个文件，
    // 再被其中一方 rename 成正式租约 —— 读端解析失败会当成"没有租约"而双双发杆。
    const std::string tmp = std::string(kPath) + ".tmp." + std::to_string(::getpid());
    {
        std::ofstream out(tmp, std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            return;
        }
        out << owner << ' ' << prio << ' ' << pid << ' ' << nowMs() << '\n';
    }
    // 原子替换：读端要么看到旧内容，要么看到新内容。
    // 前提是 tmp 和目标在同一文件系统（这里同目录，天然满足）。
    std::rename(tmp.c_str(), kPath);
}

}  // namespace

Entry read()
{
    Entry e;
    std::ifstream in(kPath);
    if (!in.is_open()) {
        return e;
    }
    std::string line;
    std::getline(in, line);
    std::istringstream ss(line);
    if (!(ss >> e.owner >> e.prio >> e.pid >> e.ts_ms)) {
        return e;                       // 格式不对 = 当成没有
    }
    e.valid = true;
    return e;
}

bool acquire(const std::string & me, int prio)
{
    const int pid = static_cast<int>(::getpid());
    const Entry e = read();

    const bool free_slot =
        !e.valid || (nowMs() - e.ts_ms) > static_cast<long long>(kStaleSec * 1000) || !pidAlive(e.pid);

    if (free_slot || e.owner == me || e.prio < prio) {
        writeEntry(me, prio, pid);
        return true;
    }
    return false;
}

void release(const std::string & me)
{
    const Entry e = read();
    if (e.valid && e.owner == me) {
        std::remove(kPath);
    }
}

}  // namespace control_lease
