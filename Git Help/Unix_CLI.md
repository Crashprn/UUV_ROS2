# Unix command line basics

This document introduces important concepts and terminology, but using the command line is a skill.  Reading alone is not sufficient to learn a skill; you must **do**.  Using the command line effectively is simply a matter of practice.  The very best way to train you would be to sit beside you and talk you through it until you have mastered it.  Since that is not possible, I offer the next best thing, an [automated shell tutorial](https://gitlab.cs.usu.edu/erik.falor/shell-tutor/).

After installing Git on your computer, install and run through this tutorial.  You'll be hacking in no time!


## Table of Contents

*   [Jargon](#jargon)
*   [The Language of the Command Shell](#the-language-of-the-command-shell)
*   [Elementary Shell commands](#elementary-shell-commands)
*   [Changing directories](#changing-directories)
*   [Dotfiles](#dotfiles)


## Jargon

#### Terminal or Console

*   An interactive program which displays and reads textual information to and from a user.
    *   Typically a black screen with white text, or a white screen with black text.
*   A "virtual" terminal is a re-creation in software of a physical device called a "dumb terminal".

![DEC_VT100_terminal.jpg](assets/DEC_VT100_terminal.jpg "A classic DEC VT100 physical terminal")


#### Shell

*   A text-based user interface
*   Accepts commands from a user, checks for syntax errors, then passes them on to the Operating System kernel
    *   Thus, it serves as a layer of protection around the OS kernel, hence the name
*   A shell runs within the context of a terminal; the terminal reads the keyboard and sends the user's text to the shell.  The shell's output is displayed on the terminal's grid of characters.
*   shell **:** terminal **::** webpage **:** browser
*   Two popular shells are **Bash** and **Zsh**


#### Bash

*   `/usr/bin/bash`
*   The **B**ourne **A**gain **SH**ell
*   An evolution of Steve Bourne's original shell program `sh`
*   A flagship Open Source program of the GNU movement
*   The most popular and widespread shell on Linux systems


#### Zsh

*   `/usr/bin/zsh`
*   The Z Shell; no clever meaning, the name just looks cool
*   Inspired by David Korn's influential `ksh`
*   Created by Paul Falstad while studying at Princeton; also open source
*   A popular alternative to `bash`; is now the default shell on Mac OS X


### Which shell should I use?

*   Just use whatever is default on your computer.
*   Both shells are reasonably similar syntax-wise
    *   Everything you see me do in class will work identically in both shells
    *   You won't notice differences until you become a power-user
*   I personally prefer `zsh`, but I won't try to convert you ;)


#### Directory

A.K.A. "folder"; a container for files and other directories


#### Current working directory

The directory a program is currently running in.

Commands that use files and directories will look in the current directory first.


#### Subdirectory

A directory within another directory; a directory that is a "child" of another
directory


#### Parent directory

The directory that contains a subdirectory



#### Prompt

Text printed by the shell before the cursor to signal that the shell is ready
to accept your command.

The prompt displays useful information such as the current username, the name
of the computer running the shell, and the shell's current working
directory.


#### Command

An instruction entered by the user to be executed by the shell.

A command may be

0.  The name of an external program installed on the computer
1.  A function that is built-in to the shell and does not exist as a program
    outside of the shell itself
2.  An alias to another command
3.  A keyword that introduces a logical construct such as `if`, `else`, `for`,
    `while`, `until`, `case`, `select`, `function`, etc.

Not all of the commands that you run execute programs that are sitting out on
the hard drive.  Many commands are actually functionality built into the shell
itself.  The distinction between external and built-in commands doesn't usually
matter, but remembering that it is there will save you some confusion later.


#### Argument

Extra information given to a command from the command line.

Arguments may refer to objects on a computer system such as the names of files,
directories, user names, or host names.

While arguments may appear as integers, floating-point numbers, or other
structures, they are always passed by the OS to a program as an array of
strings.  It is the responsibility of the program accepting the arguments to
convert these strings into other data types as necessary.


#### Option (also: Switch, Flag)

A command-line argument that to modifies a program's behavior.

In some programs the mere presence or absence of an option is enough to signal
a change to the program's behavior.  These are often referred to as "boolean
options" or "flags", though the former term has long since been abused into
being a synonym for "option".

Some options must themselves be followed by further arguments in order to be
meaningful.



## The Language of the Command Shell

The command shell is a simple programming language.  Unlike the other languages
you are familiar with, this language is designed to be fast and easy to type
interactively.

*   Instructions tend to be brief
*   Very little punctuation is used
*   There is only one data type: String
*   Ease-of-use shortcuts such as tab completion are built-in to the interpreter

Shell commands follow this syntax:

    command [argument ...]


Commands, like functions in Python, may take arguments.  The square brackets
surrounding `argument ...` in the example above indicate an optional portion.
The ellipsis means that there may be more arguments beyond the first one.  All
together, this example means "_command_ takes zero or more arguments".

Unlike Python and Java, parentheses do not surround the argument list and
spaces are used to separate arguments from each other instead of commas.  When
an argument contains white space it must be surrounded by quote marks (either
single `'` or double-quotes `"` in a matching pair).

Arguments are passed from the shell into the command as a list of strings.
Each command verifies that the correct arguments were given.  It is up to the
command to decide how many arguments it needs as well as the meaning of each
argument.  For example, if an argument should be regarded as an integer, it is
the responsibility of the command to convert that string into its numeric form.


### Elementary Shell commands

*n.b. These commands are covered in much more depth in the Shell Tutor*

**TASK**                         |  **COMMAND**
---------------------------------|-------------------------------------------
Print arguments to screen        |  `echo [ARG0 ARG1...]`
List files                       |  `ls`
Print file to screen             |  `cat [FILE0 FILE1...]`
Clear the screen                 |  `clear`
Reset the terminal               |  `reset`
View the manual for a command    |  `man COMMAND`
Display the type of a command    |  `type COMMAND`
Copy target file to destination  |  `cp TARGET DEST`
Move target file to destination  |  `mv TARGET DEST`
Delete file(s)                   |  `rm FILE0 [FILE1...]`
Change Directory                 |  `cd [DIR]`
Make a new directory             |  `mkdir DIR0 [DIR1...]`
Remove an empty directory        |  `rmdir DIR0 [DIR1...]` (only works on empty dirs)


### A word on "standard" command-line arguments

Options can be preceded by zero, one or two hyphens.  Options for commands for
`cmd.exe` on Windows are even preceded by a front slash `/`.  There are many
standards governing the *right* way to present command-line arguments.  They
are all equally invalid.

![XKCD: Standards](./assets/standards.png)


### Common command-line options

Nevertheless, there are some options which mean the same thing to many
different programs.  Check the documentation before trusting this list;
before you leap

*   `--help` or `-h`
    Print a usage message
*   `--verbose` or `-v`
    Produce more output than usual
*   `--version`, `-version`, sometimes `-V`
    Report the version number of the program


## Changing directories
The `cd` command changes your working directory.  In the examples below the '$'
represents your prompt.  Don't type the '$'; type the text that follows.

Go to the "parent folder"; move up one level, leaving the directory you are
presently in

    $ cd ..


_Protip_: The tilde '~' character is shorthand for your home directory.

Return to your home directory regardless of your current location

    $ cd ~

Go to a directory under your home named 'workspace':

    $ cd ~/workspace

You can quickly return to your home by running `cd` with no arguments:

    $ cd



## Dotfiles

Files whose names begin with `.` are ordinarily hidden from file listings in
Unix programs such as `ls`.

    $ ls
    src/  data/  doc/  README.md


This isn't actually a super-13337 way to hide your secrets.  One can simply
give `ls` the `-a` flag to display *ALL* files, even hidden ones.

    $ ls -a
    .  ..  .git/  .gitignore  src/  data/  doc/  README.md


The fact that these files are hidden from view arises from a bug in the first
Unix system.  This bug proved to be a handy way to hide configuration files and
was promoted into a feature.

Dotfiles are configuration files used by many applications on Unix-like
systems.  Dotfiles contain plain text and can be edited with an ordinary text
editor.  Upon startup a Unix application will search for its configuration by
looking for a specific file under a handful of common directories on the
computer.  Reconfiguring a program is as simple as saving a file with the right
name in the right place.  This is in contrast to a centralized, system-wide
settings repository such as the Windows Registry which presents a unified
interface but requires special-purpose software to manage.

Git and SSH are two programs you will use this semester whose behavior is
governed by dotfiles.


### Creating a dotfile on Windows

Programs following the Unix tradition (e.g. git) still use dotfiles even when
they are modified to run on Windows.  In the Windows OS the concept of "file
extension" is very important, and filenames consisting only of an extension are
regarded as being invalid.  Many native Windows tools will "helpfully" refuse
to save a file without a proper name, rename the file for you, or add another
filename extension (such as `.txt`) to your dotfiles.  These filename changes
will cause Unix-style programs to fail to load their configuration.

The easiest thing to do is to not use Windows tools and instead use the `nano`
editor to manage your dotfiles.  For example, you can control which files `git`
will ignore by editing a dotfile called `.gitignore` using `nano`:

    $ nano .gitconfig
