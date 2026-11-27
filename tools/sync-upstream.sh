#!/usr/bin/env bash
# Sync the fork's master with upstream and rebase the ipados branch on top.
# Usage: tools/sync-upstream.sh [branch]   (default: ipados)
set -euo pipefail

BRANCH="${1:-ipados}"

cd "$(git rev-parse --show-toplevel)"

echo "Fetching upstream and origin..."
git fetch upstream
git fetch origin

# Guard: never overwrite work that exists on origin but not locally.
if ! git merge-base --is-ancestor origin/"$BRANCH" "$BRANCH" 2>/dev/null; then
    echo "ERROR: origin/$BRANCH has commits not in local $BRANCH."
    echo "       Pull them first (git merge origin/$BRANCH) before syncing."
    exit 1
fi

echo "Updating master..."
git checkout master
git merge --ff-only upstream/master
git push origin master

echo "Rebasing $BRANCH..."
git checkout "$BRANCH"
git rebase upstream/master

echo "Pushing $BRANCH (force-with-lease)..."
git push --force-with-lease origin "$BRANCH"

echo "Done. Resolve conflicts above if any, then re-run."