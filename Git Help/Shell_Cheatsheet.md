# Unix Shell Cheat Sheet

## Shell shortcuts

The shortcuts listed here work out of the box in both Bash and Zsh.

There are a lot of shortcuts listed here; don't try to learn them all at once!
The best way is to pick one or two and then go out of your way to use them as
often as you can all day.  If you do something the old way, put the cursor back
and do it again the *new* way.  Do this until you are sure that

1. This shortcut is useful and you'll remember do to it from now on
2. This shortcut isn't that helpful and you can live without it

Don't feel bad about shortcuts which never *clicked* for you.  Expert shell
users don't use all of them, instead relying on their own personal subset.
Just move on to the next shortcut tomorrow.  After a week or two you'll have
worked through this list and will be a much more efficient shell user.

Shortcut   | Action
-----------|-----------------------------------------------
  `Up`     | Bring up older commands from history
  `Down`   | Bring up newer commands from history
  `Left`   | Move cursor BACKWARD one character
  `Right`  | Move cursor FORWARD one character
`Backspace`| Delete the character to the LEFT of the cursor (N/A on Macintosh)
  `Delete` | Delete the character on the RIGHT the cursor (Left of cursor on Macintosh)
  `^A`     | Move cursor to BEGINNING of line
  `^E`     | Move cursor to END of line
  `M-B`    | Move cursor BACKWARD one whole word
  `M-F`    | Move cursor FORWARD one whole word
  `^C`     | Cancel (terminate) the currently running process
  `TAB`    | Complete the command or filename at cursor
  `^W`     | Cut BACKWARD from cursor to beginning of word
  `^K`     | Cut FORWARD from cursor to end of line (kill)
  `^Y`     | Yank (paste) text to the RIGHT of the cursor
  `^L`     | Clear the screen while preserving command line
  `^U`     | Cut the entire command line



After you have gone through this list search online for a Bash/Zsh cheat sheet
and pick up some more tricks!  By dedicating just a little time each day to
intentional practice you will exceed what many "professional" programmers know
about the shell by the end of the semester.


## In case `Ctrl-S` freezes your shell

It could be the case on your computer that `ctrl-s` causes your terminal to
appear to freeze.  Nothing appears on the screen as you type.  However,
everything you do *is* still being sent to the shell where it is being run; you
just can't see the results.  

This is an old "feature" of terminals called *Scroll Lock*.
When this happens to you press `ctrl-q` to unfreeze the terminal.

You can add this code to your `~/.bash_profile` or `~/.zshrc` to prevent this
from happening:

    # Disable Ctrl-S scroll lock
    stty -ixon


## Basic Shell Commands

### Console manipulation

| Operation                        | Command
|----------------------------------|-------------
| Print arguments to console       | `echo`
| Clear the console                | `clear` 
| Reset a garbled console          | `reset`


### Directory manipulation

| Operation                        | Command
|----------------------------------|-------------
| Change directory                 | `cd`    
| Print working dir (where I am?)  | `pwd`   
| Create new directory             | `mkdir` 
| Remove empty directory           | `rmdir`  
| List files in directory          | `ls`    


### File manipulation

| Operation                        | Command
|----------------------------------|-------------
| Delete file(s)                   | `rm`     
| Copy a file                      | `cp`
| Move a file                      | `mv`
| Edit a file                      | `nano`


### Text manipulation

| Operation                        | Command
|----------------------------------|-------------
| Print entire file to screen      | `cat`   
| Print top 10 lines of file       | `head`  
| Print last 10 lines of file      | `tail`
| Search for text within a file    | `grep`
| Sort lines of a file             | `sort`
| Count chars, words, lines        | `wc`


### Getting help

| Operation                        | Command
|----------------------------------|-------------
| Show the manual for a command    | `man`
| Where is a command installed?    | `which`
| What type is a command?          | `type`
