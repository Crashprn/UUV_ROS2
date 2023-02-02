# Git repository setup...

...and how to use Git to submit your assignments to GitLab


* [Create an SSH key and load it on GitLab](#create-an-ssh-key-and-load-it-on-gitlab)
* [Create a new repository and put your code in it](#create-a-new-repository-and-put-your-code-in-it)
* [Send the repository on your computer to the GitLab server](#send-the-repository-on-your-computer-to-the-gitlab-server)
* [Double-check that everything worked as expected](#double-check-that-everything-worked-as-expected)
* [Help! `git remote` printed something unexpected](#help-git-remote-printed-something-unexpected)
* [Help! I'm still having some troubles with Git](#help-im-still-having-some-troubles-with-git)



## Create an SSH key and load it on GitLab

You can avoid the tedious entry of username and password by creating an SSH key and importing it into your GitLab profile.  There is a tutorial video on Canvas that explains how to set this up.

Doing this now avoid problems later.  If you encounter errors at this stage, follow the instructions in the Git [Troubleshooting Guide](./Troubleshooting.md)



## Create a new repository and put your code in it

*In the code examples below a dollar sign `$` represents the shell prompt.  This is to distinguish commands that you will input from their output. Do not type the `$` when you run these commands yourself.*

0.  **[Windows Users Only]** Install and configure Git for Windows.  Watch the tutorial video on Canvas for instructions.


1.  **[First-time only]** Launch a command shell and use the `git config` command to set up your name and email address.  Git needs to know who you are so that when you make commits it can record who was responsible.
    ```
    $ git config --global user.name  "Danny Boy"
    $ git config --global user.email "danny.boy@houseofpain.com"
    ```

2.  Ensure that you're not presently within a git repository by running `git status`.

    *Protip:* You *don't* want to create a git repository within a git repository.  *Always* run `git status` before using any command which creates a new git repository to ensure that you are not already within a git repository.  You want to see `git status` present an error message in this situation as it will confirm that you are *not* already inside a git repository.

    ```
    $ git status
    fatal: not a git repository (or any parent up to mount point /)
    Stopping at filesystem boundary (GIT_DISCOVERY_ACROSS_FILESYSTEM not set)
    ```

3.  Clone the git repository containing the starter code from GitLab onto your computer using the `git clone` command.  The *local* repository's name on your own computer isn't terribly important as long as you can keep things straight.

    ```
    $ git clone git@gitlab.cs.usu.edu:erik.falor/cs1440-falor-erik-assn0 cs1440-LAST-FIRST-assn0
    ```


4.  Enter the directory containing the newly cloned repository.
    ```
    $ cd cs1440-LAST-FIRST-assn0
    ```

5.  Open `README.md` in an editor and change the file in some way. You might say something nice to your grader (hey, you never know...)

6.  Return to your command shell ask `git` about the status of your repository.
    ```
    $ git status
    ```

7.  Run `git add` to add `README.md` to your repository.
    ```
    $ git add README.md
    ```


8.  Check the status of your repository again; it should tell you that you have changes ready to be committed.
    ```
    $ git status
    ```


9. You're ready to permanently commit to these changes.  Use the `-m` option to add a brief message (between double quotes) about this change.
    ```
    $ git commit -m "say something flattering to the graders"
    ```


10. Get the status of your repository once more; the directory should be "clean".
    ```
    $ git status
    ```


11. Review the commit history of your repository.
    ```
    $ git log
    ```



## Send the repository on your computer to the GitLab server

Now you're ready to put your code up on the web, but you don't need to open your browser.  The `git` command can do it all from your command shell.

0.  Run the `git remote` command in *verbose* mode to see if there are already remote servers connected to your repository.  If you're following this tutorial exactly there will be one named `origin`.
    ```
    $ git remote -v
    origin	git@gitlab.cs.usu.edu:erik.falor/cs1440-falor-erik-assn0 (fetch)
    origin	git@gitlab.cs.usu.edu:erik.falor/cs1440-falor-erik-assn0 (push)
    ```

1.  Rename `origin` to `old-origin`
    ```
    $ git remote rename origin old-origin
    ```

2.  Define the name that your repository will have on the GitLab server.  The name you choose in this step becomes part of the URL (a.k.a. internet address) for your repository.  *This is the name that matters!*  This name must follow the course guidelines.  If it doesn't match the expected pattern I will not be able to find your repository on GitLab.  It will be as though you did not turn it in.
    *   The pattern has four parts, in this order, separated with hyphens `-`:
        *   Course number.  In this class use `cs1440`.
        *   Your last name (if you have two last names, separate them with hyphens).
        *   First name (if you have two first names, separate them with hyphens).
        *   `assn` plus the assignment number.
    *   For example, `cs1440-LAST-FIRST-assn0` is appropriate for my Assignment #0 submission.
    *   *Note* this name does *not* need to match the name of the repository's directory on your own computer.
    *   Do not take any liberties with this pattern.  I wrote a program to go out and find your submissions.  It is a dumb program that is easily confused.

3.  Create a remote repository called `origin` that points to the URL for your repository on the GitLab server.  This will be an SSH URL; it looks a little different than the web addresses you are familiar with.  An SSH URL has four parts:
    *   The username `git`, followed by `@`
    *   The address to the GitLab server `gitlab.cs.usu.edu`, followed by `:`
    *   Your username on GitLab (in the example below I use `USERNAME` as a placeholder), followed by `/`
    *   The repository name you defined in the previous step

    For example: `git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0`

    Use the `git remote add` command to associate the nickname `origin` to the complete URL.  Pay attention to the `@`, `:` and `/` in this URL:

    ```
    $ git remote add origin git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0
    ```

4.  Run `git remote -v` to make sure everything looks okay:
    ```
    $ git remote -v
    origin	git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0 (fetch)
    origin	git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0 (push)
    old-origin	git@gitlab.cs.usu.edu:erik.falor/cs1440-falor-erik-assn0 (fetch)
    old-origin	git@gitlab.cs.usu.edu:erik.falor/cs1440-falor-erik-assn0 (push)
    ```

    *Protip:* If you spelled `origin` wrong, you can fix it with `git remote rename` (see below)

    *Protip:* If you made a typo in the URL, you can fix it with the `git remote set-url` command (see below)

5.  Use `git push` to upload all of your code to the GitLab server under the nickname `origin`.  You will see an ASCII art receipt if this works.  If you do not see this image, contact the instructor or a TA for help.
    ```
    $ git push -u origin --all
    Enumerating objects: 3, done.
    Counting objects: 100% (3/3), done.
    Delta compression using up to 4 threads
    Compressing objects: 100% (2/2), done.
    Writing objects: 100% (3/3), 270 bytes | 270.00 KiB/s, done.
    Total 3 (delta 0), reused 0 (delta 0), pack-reused 0
    remote: ***********************************************************************
    remote: *           __  ________  __  _____                ____    _          *
    remote: *          / / / / __/ / / / / ___/__  __ _  ___  / __/___(_)         *
    remote: *         / /_/ /\ \/ /_/ / / /__/ _ \/  ' \/ _ \_\ \/ __/ /          *
    remote: *         \____/___/\____/  \___/\___/_/_/_/ .__/___/\__/_/           *
    remote: *                                         /_/                         *
    remote: *  ,/         \,                                                      *
    remote: * ((__,-"""-,__))                                                     *
    remote: *  `--)~   ~(--`                                                      *
    remote: * .-'(       )'-,                                                     *
    remote: * `--`d\   /b`--`  Big Blue says:                                     *
    remote: *     |     |                                                         *
    remote: *     (6___6)  Your submission arrived Thu 19 Aug 2021 01:07:26 MDT   *
    remote: *      `---`                                                          *
    remote: *                                                                     *
    remote: ***********************************************************************
    To gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0
     * [new branch]      master -> master
    Branch 'master' set up to track remote branch 'master' from 'origin'.
    ```


## Double-check that everything worked as expected

0.  Visit your repository online using your web browser.  Do this by converting the SSH URL into a familiar HTTPS address.
    *   Replace `git@` with `https://`
    *   Replace the `:` separating your username from the repository's name with `/`.
    *   **Before** `git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0`
    *   **After** `https://gitlab.cs.usu.edu/USERNAME/cs1440-LAST-FIRST-assn0`

1.  You should not see the message **The repository for this project is empty**; instead, you should see a listing of files in your repo and the contents of `README.md`.

2.  Double-check that *all* of the files that you expect to see are present, including your `.gitignore`.  Make sure that *no* files that should *not* be part of your project are present
    *   What you see on this page is what we will see when we grade your work.

3.  Triple-check that everything works by cloning your project back to your computer.  This step replicates what we'll do when we grade your work.  What you see here is what we will see when we grade your submission.
    *   Use the `cd` command to go to a new location on your computer, one that is not associated with your project.
    *   Run `git status` to make sure you're not already in a git repository (you want to see a **fatal** error here).
    *   Run `git clone` along with the SSH URL to your repository to re-download it here.
    *   The `git clone` command creates a new directory with the full name of your repo on GitLab (e.g. `cs1440-LAST-FIRST-assn0`).
    *   `cd` into this directory to look around, execute your program, run your tests, etc.  Make sure everything looks good.



## Help! `git remote` printed something unexpected

This happens when your repository already knows about a remote server.  This may or may not be a problem:

*   If the remote's name is `origin` and the URL is correct, you might just try pushing your code.  Just continue following the steps above.  If that doesn't work...
*   If you spelled `origin` wrong, you may change it with `git remote rename`:
    ```
    $ git remote rename origin old-origin
    ```
    I should point out that there is nothing special about the name `origin`; it's just a git tradition.  You can submit your work to my server under any name you please.
*   If the name of the remote server is `origin`, but the URL is wrong, fix it with `git remote set-url`:
    ```
    $ git remote set-url origin git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn0
    ```
*   If the remote repository already exists on GitLab but contains the wrong stuff, you can [delete it](./Troubleshooting.md#deleting-a-repository-from-gitlab).



## Help! I'm still having some troubles with Git

See the Git [Troubleshooting Guide](./Troubleshooting.md) for solutions to the most common problems.
