# Release how-to

Releases are cut from `master` with a changelog-only commit, an annotated
tag, and a push. The release workflow (`.github/workflows/release.yml`)
builds all flavors and publishes the GitHub release; the apt repo ingests
from there automatically.

## Steps

1. Make sure `master` is synced and CI is green on the release candidate:

   ```sh
   git checkout master && git pull --ff-only
   gh run list --workflow ci.yml --branch master --limit 1
   ```

2. Add a `debian/changelog` entry at the top. Version scheme:
   `X.Y.Z-1` (Debian revision is always 1; the distro/codename suffix is
   added by CI at build time). Bump Y for features, Z for fixes only.
   One `*` bullet per user-visible change, prefixed with the conventional
   commit scope (`feat(hal):`, `fix(main):`, `docs:`, ...), wrapped at
   ~72 columns. Credit external contributors ("Thanks to <handle>.").
   Trailer format:

   ```
    -- Luca Toniolo <toniolo.luca@gmail.com>  Tue, 29 Sep 2026 19:56:03 +0800
   ```

   (`date '+%a, %d %b %Y %H:%M:%S %z'` gives the date string.)

3. Commit with only the changelog in it:

   ```sh
   git add debian/changelog && git commit -m "release: X.Y.Z-1"
   ```

4. Create an annotated tag; the message is `Release vX.Y.Z`:

   ```sh
   git tag -a vX.Y.Z -m "Release vX.Y.Z"
   ```

5. Push both:

   ```sh
   git push origin master vX.Y.Z
   ```

6. Watch the Release workflow:

   ```sh
   gh run list --workflow release.yml --limit 1
   ```

   It builds 10 debs (3 distros x amd64, 2 distros x arm64, each in plain
   and +getset flavor; bullseye is plain-only) and publishes the GitHub
   release with them attached. Publishing dispatches to the apt repo,
   which ingests the debs into the `main` (plain) and `getset` (+getset)
   components. Verify both components afterwards:

   ```sh
   curl -s https://linuxcnc-ethercat.github.io/apt/dists/trixie/main/binary-amd64/Packages | grep -A1 '^Package: linuxcnc-ethercat$'
   curl -s https://linuxcnc-ethercat.github.io/apt/dists/trixie/getset/binary-amd64/Packages | grep -A1 '^Package: linuxcnc-ethercat$'
   ```

## Notes

- The HAL API flavor matrix lives in `ci.yml`/`release.yml`
  (`hal_api: [legacy, next]`). The `next` (+getset) flavor builds against
  the linuxcnc ref pinned in
  `.github/actions/linuxcnc-master-debs/action.yml` (currently
  `v2.10.0-pre2`). Bumping that pin is a deliberate change with its own
  commit, never part of a release commit.
- Prereleases: name the tag `vX.Y.Z~preN` or `vX.Y.Z-preN` and the
  workflow marks the GitHub release as prerelease automatically.
- If the release workflow fails after the tag is pushed, fix on master,
  delete and re-create the tag (`git tag -d`, `git push origin :vX.Y.Z`),
  push again.
