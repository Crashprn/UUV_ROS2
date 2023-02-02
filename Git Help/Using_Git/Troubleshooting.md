# Git Troubleshooting Guide

*Note: In this document I use the terms **project** and **repository** interchangeably.  GitLab prefers the term project, while the Git command-line program uses repository.*

*Note: In the code examples below a dollar sign `$` represents the shell prompt. This is to distinguish commands that you will input from their output. Do not type the `$` when you run these commands yourself.*

## Table of Contents

* [Changing a project's URL on GitLab](#changing-a-projects-url-on-gitlab)
* [Renaming a project in GitLab](#renaming-a-project-in-gitlab)
* [Deleting a project from GitLab](#deleting-a-project-from-gitlab)
* [Git asks for my username and password, but freezes when i enter the password](#git-asks-for-my-username-and-password-but-freezes-when-i-enter-the-password)
* [I've created an ssh key but gitlab still asks for a password](#ive-created-an-ssh-key-but-gitlab-still-asks-for-a-password)
* [On MacOS git gives an 'xcrun error'](#on-macos-git-gives-an-xcrun-error)
* ["The project you were looking for could not be found."](#the-project-you-were-looking-for-could-not-be-found)
* ["Permission denied" when I use 'git init' or 'git clone'](#permission-denied-when-i-use-git-init-or-git-clone)
* [SSL certificate problem: unable to get local issuer certificate](#ssl-certificate-problem-unable-to-get-local-issuer-certificate)
* [git fails with an SSH usage message](#git-fails-with-an-ssh-usage-message)
* [git push says "No such file or directory" and 'Anaconda3' appears in the error message](#git-push-says-no-such-file-or-directory-and-anaconda3-appears-in-the-error-message)

------------------------------------------------------------

## Changing a project's URL on GitLab

If you gave your project the wrong URL you can fix it in GitLab before the due date and receive no penalty.

*Note that the project's **URL** is different from its **name**; see the next tip for details about renaming a project*

0.  Navigate to the project on the web and click the gear icon at the bottom of the left sidebar (the Settings menu), and click *General*
    *   _Do not change the **Project name** that you see at the top of this page!_
1.  Scroll all the way to the bottom until you find the **Advanced** section.  Click `Expand`
2.  Scroll to the middle of the **Advanced** section until you see a box titled **Change path**.  Put the correct project name into this box and click the `Change path` button.
3.  Back on your PC update your project's remote URL.  Assuming your GitLab remote is nicknamed `origin`, this command will update the URL (substitute your own details in this command):
    ```
    $ git remote set-url origin https://gitlab.cs.usu.edu/USERNAME/cs1440-LAST-FIRST-assn#
    ```
4.  After changing the path you *must* make another push before my submission collection program can notice the change.
    *   You may need to edit a file and create a new commit so that you can do a push.
    *   If you don't know what to change, just make a small, cosmetic change in one of the README.md files.


## Renaming a project in GitLab

For grading purposes, the name of your project on GitLab doesn't matter.  Your project's **URL** must follow a strict naming convention so that your grader can find it.

However, if you do wish to change its name, follow these steps

0.  Navigate to the project on the web and click the gear icon at the bottom of the left sidebar (the Settings menu), and click *General*
1.  The very first setting you will see is **Project name**
2.  After changing this string click the `Save Changes` button


## Deleting a project from GitLab

This is done through the GitLab web interface.

0.  Navigate to the project on the web and click the gear icon at the bottom of the left sidebar (the Settings menu).  Click *General*
1.  Scroll all the way to the bottom until you find the **Advanced** section.  Click `Expand`
2.  Scroll all the way to the bottom one more time to find the **Delete project** section.  Click the red `Delete project` button.
3.  Enter the name of the project into the text box and click the `Yes` button to confirm.

You may now create a new project with the same URL.





## Git asks for my username and password, but freezes when I enter the password

Git isn't frozen; your keystrokes are merely hidden from the screen to prevent a passerby from reading your password.  Type your password as usual and press Enter.



## I've created an SSH key but GitLab still asks for a password

GitLab won't ask for a password if it has a copy of the SSH public key that matches a key on your local machine.

0.  Ensure that you have actually generated a public key on your system.
    *   Look for a subdirectory named `.ssh` under your home folder.
    *   It should contain one or more files with the `.pub` extension.
1.  SSH prefers to use a public key called `id_rsa.pub` before one called `id_ed25519.pub`.
    *   If you have both public keys on your local machine, try importing the both to GitLab.
2.  Make sure that you're not logged in to **gitlab.com** instead of **gitlab.cs.usu.edu**
    *   An SSH key imported to **gitlab.com** does not grant access to **gitlab.cs.usu.edu**, and vice-versa.
3.  Verify that the contents of the public key file on your computer matches what is in your GitLab account.
4.  Check that you aren't trying to push your code to the wrong website.
    *   The URL shown by the command `git remote -v` should contain **gitlab.cs.usu.edu**



## On MacOS git gives an 'xcrun error'

As of MacOS Catalina you may run into a few errors when trying to use the Terminal application or any command line tools that are built-in.  The first error you may run into is encountered when you run the `git` command, and it looks like this:

```
xcrun: error: invalid active developer path (/Library/Developer/CommandLineTools), missing xcrun at: /Library/Developer/CommandLineTools/usr/bin/xcrun
```

This is because by default command line tools are installed with Xcode.  Beginning with MacOS Catalina there is a new terms and conditions agreement that has to be accepted. To install/update the built-in command line tools and accept the new agreement run:

```
$ xcode-select --install
```

This will open a window with the new terms and agreements for developer tools.  When you accept it will open up the install/update window and it will download and install/update the command line tools from Xcode.



## "The project you were looking for could not be found."

This error is often accompanied by this message from the remote server:

```
Please make sure you have the correct access rights and the repository exists
```

While it *could* be the case that you are trying to access a private repository for which you lack access privileges, this error message is also given when you access an incorrect URL.

Check that the URL displayed by `git remote -v` matches *exactly* the URL to your repo on GitLab.  You can find this URL by clicking the `Clone` button in your repository's webpage.

Alternatively, it is straightforward to transform the repo's `https://` URL used by a browser into an SSH URL like this:

0. Replace `https://` with `git@`
1. Replace the 1st `/` after the hostname with `:`

```
Change this: vvvvvvvv                 v
             https://gitlab.cs.usu.edu/erik.falor/sp20-cs1440-lecturenotes.git
                 git@gitlab.cs.usu.edu:erik.falor/sp20-cs1440-lecturenotes.git
       into: ^^^^^^^^                 ^
```

Most often people forget to change that first `/` into a `:`.  Make sure the remainder of the URL is spelled correctly.

The `set-url` subcommand of `git remote` changes a URL.  For example, if I want to set `origin` to point to the SSH URL, I run this command:

```
$ git remote set-url origin git@gitlab.cs.usu.edu:erik.falor/sp20-cs1440-lecturenotes.git
```



## "Permission denied" when I use 'git init' or 'git clone'

When running `git clone` you may see this error:

```
fatal: could not create work tree dir: Permission denied
```

`git init` may also tell you `Permission denied`.  This error means that git
cannot create files and directories in the current directory because your user
account does not have write privileges.  For instance, you will see this
message if you try to clone a repository directly into `C:\`.

Make sure that you're in your home directory or a subdirectory of your home and
try the command again.  Running the `cd` command with no arguments will always
return you to your home directory.



## SSL certificate problem: unable to get local issuer certificate

Pushes to and clones from the GitLab server via an HTTPS-style URL are failing with an error message mentioning an SSL certificate.

This is happening because the SSL library included with your installation of Git does not trust the certificate authority which issued the certificate for my GitLab server.  Most often it is students using Git+Bash for Windows, but this could happen on other platforms as well.

The best work-around is to use the SSH interface instead of HTTPS.  This involves creating an SSH key and changing the remote repository's URL over to an SSH-style address with the `git remote set-url` command.  You will find a page on Canvas called "Creating your account on GitLab and adding an SSH key".  It contains a video that walks you through the process.

After you've imported your SSH key into GitLab, you will be given the option to use an SSH-style URL.

0.  You will find this URL on your repository's GitLab page by clicking the blue "Clone" drop-down button at the upper-right of the page.
1.  Copy the URL listed in the "Clone with SSH" box
2.  In your command shell you will use another form of the `git remote` command to change the URL (substitute your own details in this command):
    ```
    $ git remote set-url origin git@gitlab.cs.usu.edu:USERNAME/cs1440-LAST-FIRST-assn#
    ```
3.  Run `git push` to verify that git can talk to the remote GitLab server.


## git fails with an SSH usage message

Pushes to and clones from the GitLab server via an SSH-style URL are failing
with this error message:

```
$ git push -u origin master
usage: ssh [-1246AaCfGgKkMNnqsTtVvXxYy] [-b bind_address] [-c cipher_spec]
           [-D [bind_address:]port] [-E log_file] [-e escape_char]
           [-F configfile] [-I pkcs11] [-i identity_file]
           [-L address] [-l login_name] [-m mac_spec]
           [-O ctl_cmd] [-o option] [-p port]
           [-Q cipher | cipher-auth | mac | kex | key]
           [-R address] [-S ctl_path] [-W host:port]
           [-w local_tun[:remote_tun]] [user@]hostname [command]
fatal: Could not read from remote repository.
```

Most often it is students who installed and used Anaconda when taking a previous course from me.  The most common cause is an incorrect piece of configuration in your global `.gitconfig` file.  To remove it, run this command:

```
$ git config --global --unset core.sshCommand
```

Afterward, re-try your `git push` or `git clone` command.



## git push says "No such file or directory" and 'Anaconda3' appears in the error message

You're getting an error message that looks something like this:

```
$ git push origin master

C:\\Users\\Jarvis\\Anaconda3\\Library\\usr\\bin\\ssh -i C:\\Users\\Jarvis\\.ssh\\id_rsa -F C:\\Users\\Jarvis\\.ssh\\config: C:\Users\Jarvis\Anaconda3\Library\usr\bin\ssh: No such file or directory

fatal: Could not read from remote repository.
Please make sure you have the correct access rights and the repository exists.
```


This problem is essentially the same as the previous issue: git is still trying
to run Anaconda's `ssh` command but you have uninstalled Anaconda from your
computer.  The remedy is the same as for the previous problem:

```
$ git config --global --unset core.sshCommand
```



