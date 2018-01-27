#!/bin/sh
echo "Adding ignore settings for Git..."
echo .classpath >> .git/info/exclude
echo .project >> .git/info/exclude
git update-index --assume-unchanged .classpath
git update-index --assume-unchanged .project
echo "Done."
