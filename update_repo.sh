#!/bin/bash

# Fetch the latest changes from the remote repository
git fetch origin

# Reset the local branch to match the remote branch
git reset --hard origin/main

# Optional: Run your files after pulling the latest changes
# ./run_your_files.sh

