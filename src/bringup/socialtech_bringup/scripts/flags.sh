#!/bin/bash
# Source it, then `normalize_flags NAV BASE`: a flag variable is ON when set to anything but "", 0, false, no or off (any case), and is then exported as 1;
# otherwise it is unset. The scripts tested the flags with "is it non-empty", so NAV=0 turned Nav2 ON and BASE=0 armed the Tracer base.
normalize_flags() {
  local f v
  for f in "$@"; do
    v=${!f:-}; v=${v,,}
    case "$v" in
      ""|0|false|no|off) unset "$f" ;;
      *) export "$f=1" ;;
    esac
  done
}
