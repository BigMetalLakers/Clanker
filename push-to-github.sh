#!/usr/bin/env bash
set -euo pipefail

# push-to-github.sh
# Safe helper to initialize a repo (if needed), create a first commit, add remote, and push.
# Tailored for: https://github.com/BigMetalLakers/Clanker.git

# Ensure script runs from repo root (its containing directory)
cd "$(dirname "$0")"

REMOTE_URL="https://github.com/BigMetalLakers/Clanker.git"

echo "Running push-to-github.sh in $(pwd)"

# Add README header if missing
if [ -f README.md ]; then
  if ! grep -q '^# Clanker' README.md; then
    echo "# Clanker" >> README.md
    echo "Appended header to README.md"
  else
    echo "README.md already contains header"
  fi
else
  echo "# Clanker" > README.md
  echo "Created README.md"
fi

# Initialize git if necessary
if [ -d .git ]; then
  echo ".git already exists — using existing repository"
else
  git init
  echo "Initialized empty git repository"
fi

# Stage all files
git add -A

# If repo has no commits yet, create initial commit, otherwise commit changes only if needed
if git rev-parse --verify HEAD >/dev/null 2>&1; then
  if [ -n "$(git status --porcelain)" ]; then
    git commit -m "chore: add project files"
    echo "Created commit with project changes"
  else
    echo "No changes to commit"
  fi
else
  git commit -m "first commit"
  echo "Created initial commit with project files"
fi

# Ensure branch is named main
git branch -M main || true

# Add or update origin remote
if git remote get-url origin >/dev/null 2>&1; then
  git remote set-url origin "$REMOTE_URL"
  echo "Updated origin remote to $REMOTE_URL"
else
  git remote add origin "$REMOTE_URL"
  echo "Added origin remote $REMOTE_URL"
fi

# Show remote and status, then push
git remote -v
git status --short --branch

echo "Pushing to origin main..."
git push -u origin main

echo "Push complete. Repository should now be on GitHub: $REMOTE_URL"
