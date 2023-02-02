# Intermediate git

This document describes git commands that will give you more detailed
information about the state and structure of your repository, and the ability
to time-travel to older points in history.

* [Important git jargon](#important-git-jargon)
* [See what's changed with `git diff`](#see-whats-changed-with-git-diff)
* [Discarding uncommitted changes](#discarding-uncommitted-changes)
* [Temporarily stashing uncommitted changes](#temporarily-stashing-uncommitted-changes)
* [Reading git's log](#reading-gits-log)
* [Tagging commits](#tagging-commits)
* [Visit older points of history in the git timeline](#visit-older-points-of-history-in-the-git-timeline)


## Important git jargon

*Protip* You can view a manual page defining bits of git jargon with the command

```
$ git help glossary
```


#### Commit

*   **(n):** A single point in the Git history; the entire history of a project
    is represented as a set of interrelated commits.
    *   Everything you push to or pull from a git server is a commit.  In other
        words, you aren't sending files or directories to the server when you
        run `git push`.  You are sending commits.
*   **(v):** The action of storing a new snapshot of the project's state in the
    Git history by creating a new commit.
    *   A *commit* is performed with the `git commit` command.
    *   New commits are formed with `git add` by computing the differences in
        the contents of the working tree with the most recently recorded
        commit.  This new commit is then permanently recorded by running the
        `git commit` command.


#### Checkout

*   The action of updating all or some of your source code files to match what
    is recorded in the repository
*   Performed with the `git checkout` and `git switch --detach` commands.


#### HEAD

*   The currently checked-out commit is *always* named `HEAD`.
*   `HEAD` is analogous to the concept of "current working directory".
    *   Instead of a location on the file system, `HEAD` refers to a location in
        your repository's history.


#### Working tree

*   The tree of actual checked out files.
    *   The working tree consists of files and directories as of the HEAD
        commit, plus any new changes that you have not yet committed.
    *   When you "checkout" a commit, what Git actually does is edit the files in the *working tree* to make them appear as the files at that commit.
    *   You can think of Git as an automated text editor.


#### Object name or SHA-1

*   The "true name" of a git commit.
*   Takes the form of a 40 character-long string of hexadecimal digits (`0`
    through `9`, `A` through `F`)
    *   e.g. `f7e8295498512363f5cd0b12459e548ca80f329f`  
*   It is a cryptographic checksum of the author's name and email address,
    timestamp, commit message, as well as the checksum of the contents of the
    commit (in a round-about way).
    *   This cryptographically-strong digital fingerprint of the commit
    prevents tampering with the history of the repository.
*   A commit's SHA-1 object name also incorporates the SHA-1 object name of its
    parent commits, which, in turn, depend upon their parents' checksums, etc.
    *   Thus, git repositories were **blockchains** before blockchains were cool.

You can always refer to a commit with this universally unique identifier.
Because sequences of 40 arbitrary characters are hard for humans to remember,
git provides nicknames, such as `HEAD` or `master`.  A commit may have multiple
names at once, but it *always* has *one* SHA-1 object name.


### Remember which git commands operate on commit objects

Some git commands take filenames as arguments.  One such command is
`git add`.

Other git commands take names of objects as their arguments.  `git pull` is an
example.  For commands that operate on *OBJECTS* you may use any of the
following names as this argument:

0.  An SHA-1 object name, which may be abbreviated to the first 7 or 8
    characters
1.  A relative reference such as `HEAD`
2.  The name of a tag
3.  A branch name such as `master`, `main`, `devel`, etc.

Understand that for the rest of this document `OBJECT` refers *only* to git
commits.  A file or directory is *never* an `OBJECT`.



## See what's changed with `git diff`

`git status` tells you which files have been changed since the last commit, but
it cannot tell you *how* those files have been modified.  `git diff` displays
the differences in each changed file since the last commit.

**Important**: press `q` to quit the diff viewer.


### `git diff` with no other arguments
Displays the difference between your source code files in the working tree and
the most recent commit.  This provides a more detailed picture than `git
status`.  This shows you what will be recorded with `git add` followed by `git
commit`.


### `git diff -- FILE...`
Displays changes in specific files instead of *everything* in the repository.


### `git diff --cached`
Once you run `git add` your changes aren't visible with the ordinary `git diff` command.

The `--cached` argument displays what you just `git add`ed.  This form of the
command can be used between running `git add` and before `git commit`.


### `git diff OBJECT` 
Show the changes between `OBJECT` and `HEAD`.

*   In other words, what changes are needed to make to the code at `OBJECT`
    become the same as `HEAD`?
*   This lets you peer into the past and compare changes to files across
    time.


### `git diff OBJECT -- FILE ...` 
Show the changes that occurred between `OBJECT` and `HEAD` only for the listed files.

For example, to see how your user's manual changed between the latest commit
and the tag `designed`, you would run this command:

```
$ git diff designed -- doc/Manual.md
```


## Discarding uncommitted changes

If you don't like what you see in `git diff` you have the option of discarding
those changes instead of permanently committing them.

The `git status` command will remind you that the `git restore` command
restores one or more files to their state at the last commit.  To throw away
changes to your `README.md` since the last commit:

```
git restore README.md
```

To throw away all changes to all files under the current directory since the
last commit:

```
git restore .
```

`git restore` is a new command to git.  In case you are ever stranded on a
system with an old version of git you should know how to do this the
"old-fashioned way".  That was to use the `git checkout` command with the `--`
argument followed by filename(s):

```
git checkout -- README.md
```

The "old-fashioned way" still works in modern git.



## Temporarily stashing uncommitted changes

Use `git stash` when you want to record the current state of the working
directory and the index, but want to go back to a clean working directory. The
command saves your local modifications away and reverts the working directory
to match the `HEAD` commit.

The changes are saved in the same way as an ordinary commit, but are not stored
in the history with ordinary commits.  This command lets you discard changes
with the option of digging them out of the recycle bin later.

You can create, view, restore or discard stashed changes with the following
commands:

### `git stash`
Stash the changes in the working tree, restoring the working tree to its `HEAD`
state.  You can create as many stashes as you wish.  They are stored in a stack
in the order in which you created them.

### `git stash list`
List the stack of stashed changes.

### `git stash show`
Display the names of files changed and the number of lines added/removed by the
top-most stash in the stack.

### `git stash pop`
Apply to the working tree the changes stored in the stash entry at the top of
the stack, and remote that stash object.  According to `git status`, these
changes are 'not staged for commit'.

### `git stash apply`
Like `git stash pop`, but do not remove this stash from the stack.

### `git stash drop`
Discard the top-most stash.



## Reading git's log


The `git log` command displays the commit history from the current commit back
to the genesis of the repository.  What you see at the top of the listing is
the latest commit.  Scrolling down takes you back in time.

**Important**: press `q` to quit the log viewer.

#### Protip

If you don't see the extra branch/commit name information in the
output of `git log` as well as an abbreviated Git commit ID enable it by
running these commands:

```
$ git config --global log.decorate true
$ git config --global log.abbrevCommit true
```

### `git log --stat`
Displays a brief summary of affected files and their changes

### `git log --patch`
Displays a diff describing that commit

### `git log --stat --patch`
Show both the stat and the patch at once

### `git log OBJECT`

*   This form of `git log` displays the commit history beginning from the commit denoted by `OBJECT` back to the genesis of the repository.
*   Any commits which were added *after* `OBJECT` are not listed in this output.
    *   `OBJECT`s stored in Git's database are *singly-linked* to a "parent" object
        *   Thus, an object can always find its parent, but they cannot easily track down their children
        *   This is by design!  Once an object has been created, its SHA-1 ID cannot be changed.  To update it so it can point to a newly created child would require that a new SHA-1 ID be generated.  Then *its* parent would need a new SHA-1 ID, and so on.


## Tagging commits

A tag is a human-friendly name for a commit object.

A tag always refers to the same commit, even after new commits are added to the
repository.  This is in contrast to how the name `master` works.  `master`
always refers to the latest commit in the master branch, and moves as you make
new commits.  A tag, once created, "sticks" to its commit.

A tag exists only in your local repository until you push it.  By default, tags
are for your own personal use.  If you want others to be able to find your tags
you must push them.


### `git tag`

*   When run without arguments lists extant tags 
*   Remember that tags only exist in the local repository *until* you push them!


### `git tag TAGNAME`

* Gives the name `TAGNAME` to the current `HEAD` commit
* The tag sticks to this commit and does not follow the branch along as you add
  commits
* A commit may have any number of tags


### `git tag TAGNAME OBJECT`

* Apply the name `TAGNAME` to `OBJECT`
* `OBJECT` can refer to any commit object anywhere in your repository; it doesn't have to be the current `HEAD` object


### `git tag -d TAGNAME`

* Remove a tag from a commit locally
* This command does not modify or delete the commit


### `git push REMOTE TAGNAME`

* Send the name of a tag along with the commit to which it points to `REMOTE`
  repository
* Tags are not pushed unless you explicitly instruct git to push them.


### `git push --delete REMOTE TAGNAME`

* Remove the tag `TAGNAME` from `REMOTE` repository
* Use this if you tagged the wrong commit and want to push the same tag name on a different commit



## Visit older points of history in the git timeline

*Protip:* For best results, commit or stash unsaved changes before attempting
to time-travel.  Otherwise, be prepared to discard your changes.

The `git checkout` command is used to move `HEAD` to another commit in the
history.  This command causes git to make the working tree become identical to
the state captured by `OBJECT`.  This is how you travel back in time with git.

### `git checkout -`
*   Return to the previous location of `HEAD`

### `git checkout master`
*   Return to the latest commit on the `master` branch

When you checkout a commit in the past you may see a notice that you are in
'detached HEAD' state.  `git status` may also report this to you.  *Detached
HEAD* simply means that you are not currently on a branch.  Remember, `HEAD` is
the name of the currently checked-out commit.  Usually this is located at the
"tip" of the branch named `master`.  You'll learn about branches later on, but
for now you can just follow the on-screen instructions to get back.
