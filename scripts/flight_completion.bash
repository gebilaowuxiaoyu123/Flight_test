# flight / fly 的 Tab 补全（bash）
#
#   source 本文件后：
#     flight <Tab>         → link sim remote views perc clean start stop ...
#     flight logs <Tab>    → agent sitl stick
#     flight mode <Tab>    → MANUAL ALTCTL POSCTL AUTO ACRO OFFBOARD STABILIZED
#     fly <Tab>            → 同上（fly 是 fly.sh 的直连入口）
#     flight <Tab><Tab>    → 把 5 个控制台命令的用途也列出来
#
# 由 scripts/flight_env.sh 自动 source，不需要手动加载。

_flight_subcommands="link sim remote views perc clean start stop status watch gui logs mode arm takeoff up down fwd back left right spin land"

_flight_console_hint() {
  # 空补全 + 提示：只列一次，不要重复刷屏
  local cur="${COMP_WORDS[COMP_CWORD]}"
  if [ -z "${cur}" ]; then
    printf '\n'
    printf '  link    ① 通信连接（agent + 节点/频率）\n'
    printf '  sim     ② 仿真（PX4 + Gazebo）\n'
    printf '  remote  ③ 网页遥控（后端日志 + 浏览器）\n'
    printf '  views   ④ 三视角窗口\n'
    printf '  perc    ⑤ 感知（深度 / 光流 CSV）\n'
    printf '  status  一行状态      watch  实时姿态    clean  清 PX4 日志\n'
  fi
}

_flight_complete() {
  local cur prev
  cur="${COMP_WORDS[COMP_CWORD]}"
  prev="${COMP_WORDS[COMP_CWORD-1]}"

  # 第二个参数：按第一个参数给候选项
  if [ "${COMP_CWORD}" -ge 2 ]; then
    case "${prev}" in
      logs)
        COMPREPLY=($(compgen -W "agent sitl stick" -- "${cur}"))
        return 0
        ;;
      mode)
        COMPREPLY=($(compgen -W "MANUAL ALTCTL POSCTL AUTO ACRO OFFBOARD STABILIZED" -- "${cur}"))
        return 0
        ;;
      clean)
        COMPREPLY=($(compgen -W "--all --dry-run --days" -- "${cur}"))
        return 0
        ;;
      up|down|fwd|back|left|right|spin|takeoff)
        COMPREPLY=($(compgen -W "1 2 3 5" -- "${cur}"))
        return 0
        ;;
    esac
  fi

  # 第一个参数：子命令 + ros2 launch 的参数写法
  if [ "${COMP_CWORD}" -eq 1 ]; then
    COMPREPLY=($(compgen -W "${_flight_subcommands} help headless:=false remote:=false port:=8000" -- "${cur}"))
    [ ${#COMPREPLY[@]} -gt 0 ] || _flight_console_hint
    return 0
  fi

  COMPREPLY=()
  return 0
}

# flight 是函数、fly 也是函数，complete 对函数同样有效
complete -F _flight_complete flight
complete -F _flight_complete fly
complete -F _flight_complete flystatus
