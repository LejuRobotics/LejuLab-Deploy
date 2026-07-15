#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
CI_SCRIPTS_DIR=$(realpath "$SCRIPT_DIR/../")
PROJECT_DIR=$(realpath "$CI_SCRIPTS_DIR/../") # project: lejulab

# Check if required parameters are provided
if [ $# -ne 4 ]; then
    echo -e "\033[31mUsage: $0 <platform> <repo_url> <repo_dir> <branch>\033[0m"
    echo -e "\033[31mInvalid number of arguments\033[0m"
    exit 1
fi

PLATFORM="$1"
REPO_URL="$2"
REPO_DIR="$3"
BRANCH="$4"

###############################################
# Helper functions
###############################################
echo_warn() {
    echo -e "\033[33m$1\033[0m"  # Yellow text
}

echo_success() {
    echo -e "\033[32m$1\033[0m"  # Green text
}

echo_error() {
    echo -e "\033[31m$1\033[0m"  # Red text
}

exit_with_failure() {
    echo_error "Error: $1"
    exit 1
}

###############################################
# Git network retry configuration (issue #1918)
#
# CI runners intermittently fail to reach remotes over SSH (e.g. gitee.com:22
# "Connection timed out"). Wrap every network-bound git call with git_retry so
# transient failures are retried with exponential backoff instead of failing
# the whole sync job on the first blip.
###############################################
MAX_GIT_RETRIES=3
GIT_RETRY_BACKOFF=5

###############################################
# Make sure SSH operations time out promptly so network retries can kick in.
# Preserves any caller-provided GIT_SSH_COMMAND (e.g. the GitHub deploy key
# set in .gitlab-ci.yml) and only appends connect/keepalive options.
###############################################
ensure_ssh_timeout() {
    local timeout_opts="-o ConnectTimeout=20 -o ServerAliveInterval=15 -o ServerAliveCountMax=3"
    if [ -z "$GIT_SSH_COMMAND" ]; then
        export GIT_SSH_COMMAND="ssh $timeout_opts -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"
    else
        case "$GIT_SSH_COMMAND" in
            *ConnectTimeout*) ;;
            *) export GIT_SSH_COMMAND="$GIT_SSH_COMMAND $timeout_opts" ;;
        esac
    fi
    echo "SSH timeout options configured (ConnectTimeout/ServerAlive)"
}

###############################################
# Run a git command with retry on transient network errors.
#
# Retries up to MAX_GIT_RETRIES times with exponential backoff
# (GIT_RETRY_BACKOFF, GIT_RETRY_BACKOFF*2, ...) but ONLY when the failure looks
# like a transient network problem (timeouts, connection reset, DNS failures,
# early EOF, ...). Non-transient failures -- authentication/permission errors,
# repo-not-found, refspec/arg errors, remote/hook rejection -- abort immediately
# so CI fails fast instead of wasting time and masking the real cause.
#
# Usage: git_retry git <args...>
# Returns: 0 on success, 1 after exhausting retries or on a non-transient error.
###############################################
git_retry() {
    local attempt=1
    local backoff=$GIT_RETRY_BACKOFF
    local stderr_file
    stderr_file=$(mktemp)
    local exit_status
    local err_text

    # Transient network errors worth retrying. Anything else fails fast.
    local transient_re="connection timed out|connection reset|could not resolve hostname|network is unreachable|operation timed out|early eof|the remote end hung up unexpectedly|kex_exchange_identification|connection refused|temporary failure in name resolution"

    while [ "$attempt" -le "$MAX_GIT_RETRIES" ]; do
        echo ">>> Git network attempt $attempt/$MAX_GIT_RETRIES: $*"

        "$@" 2>"$stderr_file"
        exit_status=$?
        if [ "$exit_status" -eq 0 ]; then
            rm -f "$stderr_file"
            if [ "$attempt" -gt 1 ]; then
                echo_success ">>> Git command succeeded on attempt $attempt"
            fi
            return 0
        fi
        err_text=$(cat "$stderr_file" 2>/dev/null)
        rm -f "$stderr_file"

        echo_warn ">>> Git attempt $attempt/$MAX_GIT_RETRIES failed (exit $exit_status)"
        echo "$err_text" >&2

        # Authentication / permission failures are not transient - stop now.
        if echo "$err_text" | grep -qiE "permission denied|authentication failed|unauthorized|not authorized|could not read username"; then
            echo_error ">>> Authentication/permission failure, aborting retries"
            return 1
        fi

        # Only retry transient network errors; fail fast on anything else
        # (repo not found, refspec/arg errors, remote/hook rejection, ...).
        if ! echo "$err_text" | grep -qiE "$transient_re"; then
            echo_error ">>> Non-transient git failure, aborting retries"
            return 1
        fi

        if [ "$attempt" -lt "$MAX_GIT_RETRIES" ]; then
            echo_warn ">>> Transient network error, retrying in ${backoff}s..."
            sleep "$backoff"
            backoff=$((backoff * 2))
        fi
        attempt=$((attempt + 1))
    done

    echo_error ">>> Git command failed after $MAX_GIT_RETRIES attempts"
    return 1
}

###############################################
# Check whether a branch exists on origin, retrying on network errors.
#
# Three-state result so callers can tell a verified "absent" apart from a
# network failure: mistaking the latter for "absent" and then creating the
# branch would let a later `git push -f` overwrite an existing remote branch.
#
# Usage: remote_branch_exists <branch>
# Returns: 0 = branch exists on origin
#          1 = branch confirmed absent on origin
#          2 = could not reach origin after retries (unknown)
###############################################
remote_branch_exists() {
    local branch=$1
    local attempt=1
    local backoff=$GIT_RETRY_BACKOFF
    local out_file
    out_file=$(mktemp)
    local exit_status

    while [ "$attempt" -le "$MAX_GIT_RETRIES" ]; do
        git ls-remote --heads origin "$branch" >"$out_file" 2>&1
        exit_status=$?
        if [ "$exit_status" -eq 0 ]; then
            if grep -q "$branch" "$out_file"; then
                rm -f "$out_file"
                return 0
            fi
            rm -f "$out_file"
            return 1
        fi

        echo_warn ">>> ls-remote attempt $attempt/$MAX_GIT_RETRIES failed (exit $exit_status)"
        cat "$out_file" >&2
        rm -f "$out_file"

        if [ "$attempt" -lt "$MAX_GIT_RETRIES" ]; then
            sleep "$backoff"
            backoff=$((backoff * 2))
        fi
        attempt=$((attempt + 1))
    done

    echo_error ">>> Could not verify remote branch '$branch' after $MAX_GIT_RETRIES attempts (network unreachable)"
    return 2
}

######################################################

clone_check_repo() {
    local repo_url=$1
    local repo_dir=$2
    local branch=$3

    echo_success ">>> Checking $PLATFORM repository..."

    if [ -d "$repo_dir" ]; then
        echo_warn "Repository exists at $repo_dir, resetting..."

        # Change to repository directory
        cd "$repo_dir" || exit_with_failure "Failed to enter repository directory: $repo_dir"

        # Check if it's a git repository
        if [ ! -d ".git" ]; then
            echo_warn "Directory exists but is not a git repository, removing and cloning..."
            cd "$PROJECT_DIR"
            git_retry git clone "$repo_url" "$repo_dir" || exit_with_failure "Failed to clone $PLATFORM repository"
            cd "$repo_dir"
        else
            # Repository exists, fetch latest changes
            echo ">>> Fetching latest changes..."
            git_retry git fetch origin || exit_with_failure "Failed to fetch from origin"
        fi
    else
        echo ">>> Repository does not exist, cloning..."
        mkdir -p "$repo_dir"
        git_retry git clone "$repo_url" "$repo_dir" || exit_with_failure "Failed to clone $PLATFORM repository"
        cd "$repo_dir"
    fi

    # Ensure we're on the correct branch and clean up
    echo ">>> Switching to branch: $branch"

    # Check if branch exists locally or remotely
    if git show-ref --verify --quiet refs/heads/"$branch"; then
        # Branch exists locally
        echo ">>> Branch exists locally"
        git checkout "$branch" || exit_with_failure "Failed to checkout branch: $branch"
    else
        remote_branch_exists "$branch"
        case $? in
            0)
                echo ">>> Branch exists remotely"
                # Branch exists remotely, create and track it
                git checkout -b "$branch" origin/"$branch" || exit_with_failure "Failed to checkout branch: $branch"
                ;;
            1)
                # Branch confirmed absent remotely - create from default branch (try master, then main)
                local default_branch=""
                if git show-ref --verify --quiet refs/remotes/origin/master; then
                    default_branch="master"
                elif git show-ref --verify --quiet refs/remotes/origin/main; then
                    default_branch="main"
                fi

                if [ -n "$default_branch" ]; then
                    echo_warn "Branch '$branch' does not exist remotely, creating new branch from $default_branch"
                    git checkout "$default_branch" || exit_with_failure "Failed to checkout $default_branch"
                    git_retry git pull origin "$default_branch" || echo_warn "Failed to pull latest $default_branch changes"
                    git checkout -b "$branch" || exit_with_failure "Failed to create branch: $branch"
                else
                    echo_warn "No default branch found, creating branch '$branch' from HEAD"
                    git checkout -b "$branch" || exit_with_failure "Failed to create branch: $branch"
                fi
                ;;
            2)
                # Could not reach origin - do NOT create a branch, otherwise a later
                # `git push -f` could overwrite an existing remote branch we failed to see.
                exit_with_failure "Failed to verify whether remote branch '$branch' exists (network unreachable); aborting to avoid overwriting a remote branch"
                ;;
        esac
    fi

    # Clean up any uncommitted changes
    git clean -dfx || exit_with_failure "Failed to clean working directory"
    git checkout . || exit_with_failure "Failed to reset working directory"

    # Pull latest changes if the branch exists on the remote
    remote_branch_exists "$branch"
    case $? in
        0)
            echo ">>> Pulling latest changes from origin/$branch..."
            git_retry git pull origin "$branch" || exit_with_failure "Failed to pull latest changes from origin/$branch"
            ;;
        1)
            echo ">>> Branch '$branch' exists locally only, no remote updates to pull"
            ;;
        2)
            exit_with_failure "Failed to verify remote branch '$branch' for pull (network unreachable)"
            ;;
    esac

    cd "$PROJECT_DIR"
    echo_success "Repository is ready on branch: $branch"
}

copy_files_to_repo() {
    local repo_dir=$1
    local syncignore_file="$SCRIPT_DIR/.syncignore.txt"

    echo_success ">>> Copying files to $PLATFORM repository..."

    # 检查 .syncignore.txt 是否存在
    if [ ! -f "$syncignore_file" ]; then
        exit_with_failure ".syncignore.txt not found at $syncignore_file"
    fi

    # 使用 rsync 同步文件，排除 .syncignore.txt 中的路径
    # --archive: 保留文件属性
    # --verbose: 显示详细信息
    # --delete: 删除目标目录中源目录不存在的文件
    # --exclude-from: 从文件读取排除模式
    rsync -av --delete \
        --exclude='.git/' \
        --exclude-from="$syncignore_file" \
        "$PROJECT_DIR/" "$repo_dir/" \
        || exit_with_failure "Failed to copy files to $PLATFORM repository"

    echo_success ">>> Files copied successfully"
}

sync_to_remote() {
    local repo_dir=$1

    cd "$repo_dir"

    # Check if there are any changes to commit
    echo_success ">>> Checking for file changes..."

    # Check for untracked files and modified files
    echo ">>> Checking for untracked files <<<"
    git status
    echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"

    # Check if there are local changes to commit
    if [ -n "$(git status --porcelain)" ]; then
        echo_warn "Changes detected, committing..."

        # Add all changes
        git add . || exit_with_failure "Failed to add changes to git"

        # Force add catkin tools config files
        git add -f .catkin_tools/profiles/default/config.yaml || echo_warn "Warning: config.yaml not found"
        git add -f .catkin_tools/profiles/default/build.yaml || echo_warn "Warning: build.yaml not found"

        # Commit with a timestamp
        local commit_message="sync $CI_PROJECT_URL/commit/$CI_COMMIT_SHA"

        git commit -m "$commit_message" || exit_with_failure "Failed to commit changes"
        echo_success "Changes committed successfully"
    else
        echo_success "No changes to commit"
    fi

    # Always push to remote
    git_retry git push -f origin "$BRANCH" || exit_with_failure "Git push failed"

    cd "$PROJECT_DIR"
}

###  main ###
# Check if parameters are non-empty
if [ -z "$REPO_URL" ]; then
    exit_with_failure "REPO_URL is empty"
fi
if [ -z "$REPO_DIR" ]; then
    exit_with_failure "REPO_DIR is empty"
fi
if [ -z "$BRANCH" ]; then
    exit_with_failure "BRANCH is empty"
fi

# Ensure SSH connections fail fast so network retries can actually trigger.
ensure_ssh_timeout

echo_success ">>> Syncing to $PLATFORM"
echo_success ">>> Syncing repo Url: $REPO_URL"
echo_success ">>> Syncing repo Dir: $REPO_DIR"
echo_success ">>> Syncing repo Branch: $BRANCH"

# clone 仓库
clone_check_repo "$REPO_URL" "$REPO_DIR" "$BRANCH"

# 复制文件到目标仓库
copy_files_to_repo "$REPO_DIR"

# 同步到远端
sync_to_remote "$REPO_DIR"

echo_success ">>> Sync to $PLATFORM completed successfully!"
