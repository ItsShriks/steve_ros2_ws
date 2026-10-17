#!/usr/bin/env bash
# Manage the package repositories listed in steve.repos (cloned into src/).
#
# Usage: ./scripts/repos.sh [command]
#   sync     (default) clone missing repos, check out their branch, pull latest
#   status   show branch, ahead/behind and local changes for every repo
#   push     push every repo that has local commits on its branch
#
# Only plain git is needed. steve.repos is also vcstool-compatible.

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS_FILE="${REPOS_FILE:-$WS_DIR/steve.repos}"
SRC_DIR="$WS_DIR/src"

if [ -t 1 ]; then
  BOLD=$'\e[1m'; RED=$'\e[31m'; GREEN=$'\e[32m'; YELLOW=$'\e[33m'; RESET=$'\e[0m'
else
  BOLD=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi

info() { echo "${BOLD}== $*${RESET}"; }
ok()   { echo "   ${GREEN}$*${RESET}"; }
warn() { echo "   ${YELLOW}$*${RESET}"; }
err()  { echo "   ${RED}$*${RESET}"; }

# Prints "name url version" per repository in steve.repos.
parse_repos() {
  awk '
    /^[[:space:]]*#/ { next }
    /^  [A-Za-z0-9_.-]+:[[:space:]]*$/ {
      if (name != "") print name, url, version
      name = $1; sub(/:$/, "", name); url = ""; version = ""
    }
    /^    url:/     { url = $2 }
    /^    version:/ { version = $2 }
    END { if (name != "") print name, url, version }
  ' "$REPOS_FILE"
}

sync_repo() {
  local name=$1 url=$2 branch=$3 dir="$SRC_DIR/$1"
  info "$name ($branch)"

  if [ ! -e "$dir/.git" ]; then
    git clone --recurse-submodules --branch "$branch" "$url" "$dir"
    ok "cloned"
    return
  fi

  git -C "$dir" fetch --quiet origin

  local current
  current=$(git -C "$dir" branch --show-current)
  if [ "$current" != "$branch" ]; then
    if [ -n "$(git -C "$dir" status --porcelain --untracked-files=no)" ]; then
      err "on '${current:-detached HEAD}' with uncommitted changes, not switching to '$branch'"
      return 1
    fi
    git -C "$dir" checkout --quiet "$branch"
    warn "switched from '${current:-detached HEAD}' to '$branch'"
  fi

  if ! git -C "$dir" merge --ff-only --quiet "origin/$branch" 2>/dev/null; then
    err "local '$branch' has diverged from origin/$branch; pull/rebase it manually"
    return 1
  fi

  # Packages with their own submodules (e.g. steve_essentials) keep those at the pinned commits.
  if [ -f "$dir/.gitmodules" ]; then
    git -C "$dir" submodule update --init --recursive --quiet
  fi
  ok "up to date at $(git -C "$dir" log -1 --format='%h %s')"
}

status_repo() {
  local name=$1 branch=$3 dir="$SRC_DIR/$1"
  if [ ! -e "$dir/.git" ]; then
    printf "%-28s %s\n" "$name" "${RED}missing (run sync)${RESET}"
    return
  fi
  local current ahead behind changes=""
  current=$(git -C "$dir" branch --show-current)
  read -r ahead behind < <(git -C "$dir" rev-list --left-right --count "HEAD...origin/$branch" 2>/dev/null || echo "? ?")
  [ -n "$(git -C "$dir" status --porcelain)" ] && changes=" ${YELLOW}[modified]${RESET}"
  local where="${current:-detached}"
  [ "$where" != "$branch" ] && where="${RED}$where (expected $branch)${RESET}"
  printf "%-28s %-12s ahead %s, behind %s%s\n" "$name" "$where" "$ahead" "$behind" "$changes"
}

push_repo() {
  local name=$1 branch=$3 dir="$SRC_DIR/$1"
  [ -e "$dir/.git" ] || return 0
  local ahead
  ahead=$(git -C "$dir" rev-list --count "origin/$branch..$branch" 2>/dev/null || echo 0)
  if [ "$ahead" -gt 0 ]; then
    info "$name: pushing $ahead commit(s) to origin/$branch"
    git -C "$dir" push origin "$branch"
  fi
}

main() {
  local cmd=${1:-sync} failed=0
  [ -f "$REPOS_FILE" ] || { echo "No repos file at $REPOS_FILE" >&2; exit 1; }
  mkdir -p "$SRC_DIR"

  case "$cmd" in
    sync|status|push) ;;
    -h|--help|help) sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown command: $cmd (use sync, status or push)" >&2; exit 1 ;;
  esac

  while read -r name url version; do
    "${cmd}_repo" "$name" "$url" "$version" || failed=1
  done < <(parse_repos)

  if [ "$failed" -ne 0 ]; then
    echo; err "Some repositories need attention (see above)."
    exit 1
  fi
}

main "$@"
