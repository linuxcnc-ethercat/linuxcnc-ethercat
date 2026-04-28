#!/bin/sh
# One-time per-clone setup so .gitattributes' merge=keep-ours driver works.
# Without this, merging master -> master-pi will rewrite debian/control and
# debian/changelog with the -bb flavor and clobber the -pi identity.
set -e
git config merge.keep-ours.driver true
echo "merge.keep-ours.driver = true (configured for $(git rev-parse --show-toplevel))"
