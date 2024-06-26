# How to contribute

## Submitting changes

Please send a [GitHub Pull Request to
us](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/new/master)
with a clear list of what you've done (read more about [pull
requests](http://help.github.com/pull-requests/)). Please follow our
coding conventions (below) and make sure all of your commits are
atomic (one feature per commit).

Always write a clear log message for your commits.  This project uses
the [Conventional
Commit](https://www.conventionalcommits.org/en/v1.0.0/) standard for
commit messages.  Generally, your messages should look something like
this:

```
   fix(el1xxx): Add support for Beckhoff EL1111

   This adds support for Beckhoff's new EL1111 device.

   Issue: #999
```

The first part of the commit message is somewhat special, and a
pre-commit will reject it unless it meets the right format.  In
general, messages should look something like one of these:

- `build: Add missing dependency`
- `ci: fix Github workflow action`
- `docs: add instructions for installing on Raspberry Pi 3s`
- `style: fix C formatting in el2xxx.c`
- `fix(ep2316): add missing PDO`
- `feat(el7941): add new module`

Specifically, the message must start with `<type>[(<scope>)]:`; scope
is optional, but should generally be added if possible.  Type can be
one of a few options (see `.pre-commit-config.yaml` for the full
list), but `fix` and `feat` are special--any change that starts with
`fix` or `feat` will trigger the release of a new version of
LinuxCNC-Ethercat.  Changes that start with `fix` will update the last
digit of the version (v1.2.3 -> v1.2.4), while `feat` will update the
minor version (v1.2.4 -> v1.3.0).  See the Conventional Commits
website for more details.

In general, the first list of each commit will be included in the
changelog for each release, so make it readable in that context.

Please include enough detail on additional lines so that reviewers can
figure out what's going on.  See other PRs in this project for examples.

## Coding and Formatting Conventions

Right now, the code is somewhat inconsistent about formatting.  We've
[discussed](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/3)
adding (and possibly enforcing) a standard formatting/indentation
standard, but aren't ready to enforce it (or manage the multi-thousand
line PR that it'd cause, or the merge headaches that would fall out
from that).

At the moment, consider using
[`clang-format`](https://clang.llvm.org/docs/ClangFormat.html) on new
or heavily modified code, using the style included in `.clang-format`
in the root of the LinuxCNC-Ethercat directory.  This is a lightly
modified version of Google's standard format, and comes fairly close
to matching most of the existing code.

## Using `scripts/esi.yml`

To generate a list of all PIDs for devices that match a specific regex, you can try something like this:

```
$ yq '.[] | select(.Name | test("E[LPM][A-Z]*3[01][-0-9]+")) | [.ProductCode + "," + .Name]' scripts/esi.yml  | cut -d' ' -f2 | cut -c2- | cut -d- -f1| sort -u | awk -F,  '{print "#define LCEC_" $2 "_PID" ,  $1; }'
#define LCEC_EL3001_PID 0x0bb93052
#define LCEC_EL3002_PID 0x0bba3052
#define LCEC_EL3004_PID 0x0bbc3052
#define LCEC_EL3008_PID 0x0bc03052
...
```

Also, take a look at
[github.com/linuxcnc-ethercat/esi-data](http://github.com/linuxcnc-ethercat/esi-data),
which contains additional tools for working with ESI data.
