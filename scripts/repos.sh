#!/usr/bin/env bash
# Manage the package repositories cloned into src/. Each profile has its own list:
# steve_test.repos and steve_dev.repos.
#
# Usage: ./scripts/repos.sh [command] [dev|test]
#   sync     (default) clone missing repos, check out their branch, pull latest; packages
#            only the other profile lists are removed from src/
#   status   show branch, ahead/behind and local changes for every repo
#   push     push every repo that has local commits on its branch
# The profile defaults to the last one synced (test at first). Shortcuts: git go-dev / git go-test
#
# Only plain git is needed. The .repos files are also vcstool-compatible.

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$WS_DIR/src"
# Last synced profile, kept inside the git dir so it is never committed.
PROFILE_FILE="$(git -C "$WS_DIR" rev-parse --absolute-git-dir)/steve-repos-profile"

if [ -t 1 ]; then
  BOLD=$'\e[1m'; RED=$'\e[31m'; GREEN=$'\e[32m'; YELLOW=$'\e[33m'; RESET=$'\e[0m'
else
  BOLD=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi

info() { echo "${BOLD}== $*${RESET}"; }
ok()   { echo "   ${GREEN}$*${RESET}"; }
warn() { echo "   ${YELLOW}$*${RESET}"; }
err()  { echo "   ${RED}$*${RESET}"; }

# Prints "name url version" per repository in $REPOS_FILE (or in the file given).
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
  ' "${1:-$REPOS_FILE}"
}

# Names of the packages listed in any profile's .repos file.
known_repos() {
  local f
  for f in "$WS_DIR"/steve_*.repos; do
    parse_repos "$f" | awk '{ print $1 }'
  done | sort -u
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
  sync_repo_dir "$dir" "$branch"
}

# Switches an existing clone to <branch> and fast-forwards it to origin/<branch>.
sync_repo_dir() {
  local dir=$1 branch=$2

  local current
  current=$(git -C "$dir" branch --show-current)
  if [ "$current" != "$branch" ]; then
    if [ -n "$(git -C "$dir" status --porcelain --untracked-files=no)" ]; then
      err "on '${current:-detached HEAD}' with uncommitted changes, not switching to '$branch'"
      return 1
    fi
    local create=()
    git -C "$dir" rev-parse --verify --quiet "refs/heads/$branch" >/dev/null \
      || create=(--create "$branch" --track "origin/$branch")
    if ! git -C "$dir" switch --quiet "${create[@]:-$branch}"; then
      err "could not switch to '$branch'"
      return 1
    fi
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

# Packages another profile lists but this one does not are deleted from src/; syncing that profile
# clones them again. Clones with uncommitted changes or unpushed commits are kept.
remove_repos() {
  local name listed dir
  listed=$(parse_repos | awk '{ print $1 }')
  for name in "$@"; do
    grep -qx "$name" <<<"$listed" && continue
    dir="$SRC_DIR/$name"
    [ -e "$dir" ] || continue
    info "$name"
    if [ -n "$(git -C "$dir" status --porcelain 2>/dev/null)" ] \
      || [ -n "$(git -C "$dir" log --branches --not --remotes --oneline 2>/dev/null)" ]; then
      err "not in the $PROFILE profile, but has uncommitted or unpushed work; commit/push it, then rerun"
      failed=1
      continue
    fi
    rm -rf "$dir"
    warn "not in the $PROFILE profile, removed"
  done
}

# SSH keys git uses for the clones: $SSH_KEYS (space-separated) or, by default, the IdentityFile(s)
# ssh picks for github.com plus ~/.ssh/id_shriks (the robot's key) — whichever exist.
ssh_keys() {
  if [ -n "${SSH_KEYS:-}" ]; then
    echo "$SSH_KEYS"
    return
  fi
  { ssh -G github.com 2>/dev/null | awk '$1 == "identityfile" { print $2 }'; echo "~/.ssh/id_shriks"; } \
    | sed "s|^~|$HOME|" | sort -u
}

# Every sync/push asks for the key passphrase (once per run, not per repository): the run gets its
# own empty ssh-agent, so a key the desktop keyring or another agent already holds is not used.
unlock_ssh_keys() {
  local key added=0
  eval "$(ssh-agent -s)" >/dev/null
  trap 'ssh-agent -k >/dev/null' EXIT
  for key in $(ssh_keys); do
    [ -f "$key" ] || continue
    info "SSH key $key"
    ssh-add -q "$key" </dev/tty && added=1 || warn "not unlocked"
  done
  if [ "$added" -eq 0 ]; then
    err "no SSH key unlocked; nothing pushed or pulled"
    exit 1
  fi
}

main() {
  local cmd=${1:-sync} failed=0
  case "$cmd" in
    sync|push|status) ;;
    -h|--help|help) sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown command: $cmd (use sync, status or push)" >&2; exit 1 ;;
  esac

  PROFILE=${2:-$(cat "$PROFILE_FILE" 2>/dev/null || echo test)}
  REPOS_FILE="$WS_DIR/steve_$PROFILE.repos"
  [ -f "$REPOS_FILE" ] || { echo "No repos file for profile '$PROFILE' at $REPOS_FILE" >&2; exit 1; }
  mkdir -p "$SRC_DIR"
  echo "${BOLD}Profile: $PROFILE${RESET} ($(basename "$REPOS_FILE"))"

  [ "$cmd" = status ] || unlock_ssh_keys

  while read -r name url version; do
    "${cmd}_repo" "$name" "$url" "$version" || failed=1
  done < <(parse_repos)

  if [ "$cmd" = sync ]; then
    echo "$PROFILE" >"$PROFILE_FILE"
    remove_repos $(known_repos)
  fi

  if [ "$failed" -ne 0 ]; then
    echo; err "Some repositories need attention (see above)."
    exit 1
  fi
}

main "$@"
