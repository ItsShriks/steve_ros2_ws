#!/usr/bin/env bash
# Manage the package repositories cloned into src/. Each profile has its own list:
# steve_test.repos and steve_dev.repos.
#
# Usage: ./scripts/repos.sh [command] [dev|test]
#   sync     (default) clone missing repos, check out their branch, pull latest; packages
#            only the other profile lists are removed from src/, build/ and install/. Given a
#            profile, sync first moves the workspace repo to the branch of the same name and
#            deletes the local branch it left (unless that has unpushed commits).
#   status   show branch, ahead/behind and local changes for every repo
#   push     push every repo that has local commits on its branch
#   mirror   copy all branches and tags of every repository (workspace, packages of both profiles,
#            steve_essentials' submodules) from the fetch host to the other one; branches that
#            diverged are reported, never overwritten
# The profile defaults to the workspace's current branch. Shortcuts: git go-dev / git go-test
# (any run adds them to the clone's git config)
#
# Every repository is on GitLab (hrl_software/steve-moma, `url` in the .repos files) and on GitHub
# (`github`). Clones fetch from one host and push to both. The fetch host is $REPOS_HOST (gitlab or
# github) or, if unset, the host the workspace's origin points at; sync moves every clone to it.
#
# Only plain git is needed. The .repos files are also vcstool-compatible.

set -euo pipefail

# Run from a git alias, git exports GIT_DIR for the workspace (it is a submodule of Master_Thesis),
# which would make every `git -C src/<package>` act on the workspace repo instead.
unset GIT_DIR GIT_WORK_TREE GIT_INDEX_FILE GIT_PREFIX GIT_COMMON_DIR GIT_OBJECT_DIRECTORY

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$WS_DIR/src"
# The workspace repo itself; the packages' URLs are in the .repos files.
WS_GITLAB="git@gitlab.igg.uni-bonn.de:hrl_software/steve-moma/steve_ros2_ws.git"
WS_GITHUB="git@github.com:ItsShriks/steve_ros2_ws.git"

if [ -t 1 ]; then
  BOLD=$'\e[1m'; RED=$'\e[31m'; GREEN=$'\e[32m'; YELLOW=$'\e[33m'; RESET=$'\e[0m'
else
  BOLD=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi

info() { echo "${BOLD}== $*${RESET}"; }
ok()   { echo "   ${GREEN}$*${RESET}"; }
warn() { echo "   ${YELLOW}$*${RESET}"; }
err()  { echo "   ${RED}$*${RESET}"; }

# Prints "name url version github" per repository in $REPOS_FILE (or in the file given).
parse_repos() {
  awk '
    /^[[:space:]]*#/ { next }
    /^  [A-Za-z0-9_.-]+:[[:space:]]*$/ {
      if (name != "") print name, url, version, github
      name = $1; sub(/:$/, "", name); url = ""; version = ""; github = ""
    }
    /^    url:/     { url = $2 }
    /^    github:/  { github = $2 }
    /^    version:/ { version = $2 }
    END { if (name != "") print name, url, version, github }
  ' "${1:-$REPOS_FILE}"
}

# The URL of <gitlab-url> <github-url> on host <gitlab|github>.
host_url() {
  if [ "$1" = github ] && [ -n "$3" ]; then echo "$3"; else echo "$2"; fi
}

# Points origin of the clone <dir> at the fetch host and makes it push to both hosts.
set_remotes() {
  local dir=$1 gitlab=$2 github=$3 fetch
  fetch=$(host_url "$HOST" "$gitlab" "$github")
  if [ "$(git -C "$dir" remote get-url origin 2>/dev/null)" != "$fetch" ]; then
    git -C "$dir" remote set-url origin "$fetch"
    warn "origin set to $fetch"
  fi
  [ -n "$github" ] || return 0
  if [ "$(git -C "$dir" config --get-all remote.origin.pushurl | tr '\n' ' ')" != "$gitlab $github " ]; then
    git -C "$dir" config --unset-all remote.origin.pushurl || true
    git -C "$dir" config --add remote.origin.pushurl "$gitlab"
    git -C "$dir" config --add remote.origin.pushurl "$github"
  fi
}

# Names of the packages listed in any profile's .repos file.
known_repos() {
  local f
  for f in "$WS_DIR"/steve_*.repos; do
    parse_repos "$f" | awk '{ print $1 }'
  done | sort -u
}

sync_repo() {
  local name=$1 url=$2 branch=$3 github=${4:-} dir="$SRC_DIR/$1"
  info "$name ($branch)"

  if [ ! -e "$dir/.git" ]; then
    git clone --recurse-submodules --branch "$branch" "$(host_url "$HOST" "$url" "$github")" "$dir"
    set_remotes "$dir" "$url" "$github"
    ok "cloned"
    return
  fi

  set_remotes "$dir" "$url" "$github"
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
  # Their URLs are relative (../<name>.git), so sync points them at the package's fetch host.
  if [ -f "$dir/.gitmodules" ]; then
    git -C "$dir" submodule sync --recursive --quiet
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
    info "$name: pushing $ahead commit(s) to origin/$branch (GitLab and GitHub)"
    git -C "$dir" push origin "$branch"
  fi
}

# Copies all branches and tags of one repository from <from> to <to> through a bare cache in
# ~/.cache/steve_repos_mirror, so later runs only download what is new.
mirror_one() {
  local name=$1 from=$2 to=$3 cache="${XDG_CACHE_HOME:-$HOME/.cache}/steve_repos_mirror/$1.git"
  info "$name: $from -> $to"
  [ -d "$cache" ] || git init --quiet --bare "$cache"
  if ! git -C "$cache" fetch --quiet --prune "$from" '+refs/heads/*:refs/heads/*' '+refs/tags/*:refs/tags/*'; then
    err "could not fetch $from"
    return 1
  fi
  if git -C "$cache" push --quiet "$to" 'refs/heads/*:refs/heads/*' 'refs/tags/*:refs/tags/*'; then
    ok "$(git -C "$cache" for-each-ref refs/heads | wc -l) branch(es), $(git -C "$cache" for-each-ref refs/tags | wc -l) tag(s)"
  else
    err "push to $to rejected (diverged branch, or the project does not exist / no access)"
    return 1
  fi
}

# <url> with its last path component replaced by <name>.git, for relative submodule URLs.
sibling_url() { echo "${1%/*}/$2.git"; }

mirror_all() {
  local other=gitlab name url version github sub rel
  [ "$HOST" = gitlab ] && other=github
  mirror_one steve_ros2_ws "$(host_url "$HOST" "$WS_GITLAB" "$WS_GITHUB")" "$(host_url "$other" "$WS_GITLAB" "$WS_GITHUB")" || failed=1
  while read -r name url version github; do
    [ -n "$github" ] || { warn "$name has no github URL, skipped"; continue; }
    mirror_one "$name" "$(host_url "$HOST" "$url" "$github")" "$(host_url "$other" "$url" "$github")" || failed=1
    [ -f "$SRC_DIR/$name/.gitmodules" ] || continue
    while read -r sub rel; do
      case "$rel" in ../*) ;; *) continue ;; esac
      sub=${rel#../}; sub=${sub%.git}
      mirror_one "$sub" "$(sibling_url "$(host_url "$HOST" "$url" "$github")" "$sub")" \
        "$(sibling_url "$(host_url "$other" "$url" "$github")" "$sub")" || failed=1
    done < <(git config -f "$SRC_DIR/$name/.gitmodules" --get-regexp '^submodule\..*\.url$')
  done < <(for f in "$WS_DIR"/steve_*.repos; do parse_repos "$f"; done | sort -u -k1,1)
}

# Deletes build/<pkg> and install/<pkg> of every ROS package in the clone <dir>, so a removed
# package is not still found in the overlay.
remove_builds() {
  local pkg
  for pkg in $(find "$1" -name package.xml -not -path '*/.git/*' \
      -exec sed -n '/<name>/{s|.*<name>\(.*\)</name>.*|\1|p;q}' {} \;); do
    rm -rf "$WS_DIR/build/$pkg" "$WS_DIR/install/$pkg"
  done
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
    remove_builds "$dir"
    rm -rf "$dir"
    warn "not in the $PROFILE profile, removed (with its build/ and install/)"
  done
}

# SSH keys git uses for the clones: $SSH_KEYS (space-separated) or, by default, the IdentityFile(s)
# ssh picks for gitlab.igg.uni-bonn.de and github.com plus ~/.ssh/id_shriks (the robot's key) —
# whichever exist.
ssh_keys() {
  if [ -n "${SSH_KEYS:-}" ]; then
    echo "$SSH_KEYS"
    return
  fi
  { for h in gitlab.igg.uni-bonn.de github.com; do
      ssh -G "$h" 2>/dev/null | awk '$1 == "identityfile" { print $2 }'
    done; echo "~/.ssh/id_shriks"; } \
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
    sync|push|status|mirror) ;;
    -h|--help|help) awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown command: $cmd (use sync, status, push or mirror)" >&2; exit 1 ;;
  esac

  PROFILE=${2:-$(git -C "$WS_DIR" branch --show-current)}
  [ -f "$WS_DIR/steve_$PROFILE.repos" ] || PROFILE=${2:-test}
  REPOS_FILE="$WS_DIR/steve_$PROFILE.repos"
  [ -f "$REPOS_FILE" ] || { echo "No repos file for profile '$PROFILE' at $REPOS_FILE" >&2; exit 1; }
  mkdir -p "$SRC_DIR"

  # The shortcuts live in the clone's own config, so each fresh clone gets them on its first run.
  git -C "$WS_DIR" config alias.go-dev '!./scripts/repos.sh sync dev'
  git -C "$WS_DIR" config alias.go-test '!./scripts/repos.sh sync test'

  HOST=${REPOS_HOST:-}
  if [ -z "$HOST" ]; then
    case "$(git -C "$WS_DIR" remote get-url origin 2>/dev/null)" in
      *github.com*) HOST=github ;;
      *) HOST=gitlab ;;
    esac
  fi
  case "$HOST" in
    gitlab|github) ;;
    *) echo "REPOS_HOST must be gitlab or github, not '$HOST'" >&2; exit 1 ;;
  esac

  echo "${BOLD}Profile: $PROFILE${RESET} ($(basename "$REPOS_FILE")), fetching from $HOST, pushing to both"

  [ "$cmd" = status ] || unlock_ssh_keys

  if [ "$cmd" = mirror ]; then
    mirror_all
    [ "$failed" -eq 0 ] || { echo; err "Some repositories were not mirrored (see above)."; exit 1; }
    exit 0
  fi

  if [ "$cmd" = sync ]; then
    set_remotes "$WS_DIR" "$WS_GITLAB" "$WS_GITHUB"
  fi

  # The workspace branch follows the profile: dev settings live on dev, test settings on test.
  if [ "$cmd" = sync ] && [ "$(git -C "$WS_DIR" branch --show-current)" != "$PROFILE" ]; then
    local previous
    previous=$(git -C "$WS_DIR" branch --show-current)
    info "workspace ($PROFILE)"
    git -C "$WS_DIR" fetch --quiet origin
    sync_repo_dir "$WS_DIR" "$PROFILE" || exit 1
    # Drop the branch we left, so switching back creates it fresh from origin (even if origin was
    # rewritten). A branch with commits origin does not have is kept.
    if [ -n "$previous" ]; then
      if [ -z "$(git -C "$WS_DIR" rev-list "origin/$previous..$previous" 2>/dev/null)" ] \
        && git -C "$WS_DIR" rev-parse --verify --quiet "origin/$previous" >/dev/null; then
        git -C "$WS_DIR" branch --quiet -D "$previous"
        warn "deleted local '$previous' (on origin; recreated on switch back)"
      else
        warn "kept local '$previous': it has commits not on origin/$previous"
      fi
    fi
  fi

  while read -r name url version github; do
    "${cmd}_repo" "$name" "$url" "$version" "$github" || failed=1
  done < <(parse_repos)

  if [ "$cmd" = sync ]; then
    remove_repos $(known_repos)
  fi

  if [ "$failed" -ne 0 ]; then
    echo; err "Some repositories need attention (see above)."
    exit 1
  fi
}

main "$@"
