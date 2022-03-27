#!/bin/bash

if [ $# -ne 0 ]; then
	comment="$*"
else
	comment="Update"
fi

git add .
git commit -m "$comment"
git push origin master