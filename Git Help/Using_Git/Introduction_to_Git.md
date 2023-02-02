# Introduction to Git

*   [What is Git?](#what-is-git)
*   [How to use GitLab](#how to use gitlab)
*   [How to learn Git](#how-to-learn-git)
*   [Pushing content to GitLab](#pushing-content-to-gitlab)
*   [Extra Git Tutorials](#extra-git-tutorials)


## What is Git?

Git is a version control system (VCS).  Think of it as the ultimate "undo" button for an entire project (a.k.a. repository). Git can remember all of the changes you've made to any files, no matter which program you've used to change them.

#### repository = a directory which has been initialized for use with git

Instead of manually keeping track of lots of backup directories for your code (BlackJack.1, BlackJack.2, BlackJack.3, BlackJack.4, etc.), git will remember what you were doing at every step of the way, and gives you a convenient way to undo your work, and compare your current files to what they were previously.


### Is Git the same thing as GitHub?

Git is not made by GitHub; GitHub just made the hub.

There are several "hubs" Git users may use to share their code online; GitHub is just the most popular right now.  Others include GitLab, Bitbucket, Sourcehut, Beanstalk, SourceForge, among others.   While we use GitLab in this course, the skills you learn here are easily transferred to other services.


## How to use GitLab

GitLab is an online service that combines a remote location for git repositories with a web-based UI to explore code.  Similar services are GitHub, Bitbucket and sourcehut.

Because it is based upon Git, you have at your fingertips not only the latest version of a file but *all* versions.  Understanding this helps you to navigate the site.

The most important parts of the site are in the blue toolbar at the top:

*   Look for the Old Main logo instead of the orange GitLab Fox logo
    *   This is how you know you're on the right site!
*   The search button (magnifying glass icon)
    *   Find words in source code/documentation


## How to learn Git

My experience as a software engineer has taught me that using a version control system is vital.  Discussions with professionals and former students confirms that knowing git makes a big difference in your job prospects.  I am also convinced that learning git will make your time as a student more effective and enjoyable.

But if you know anything about git, you know that it does not have a reputation of being user-friendly.  While you are new to git you face a steep learning curve.  Don't give up, and don't panic over your mistakes!  Continuous practice is the only way to master this tool; this is why I make you submit all assignments through git.  The power of git is that it allows you to bravely forge ahead, confident that you can recover from the mistakes you make along the way.  Git is your safety net.  You will get tangled up in it at first, but in time it will become your favorite tool.

Your journey starts with ten fundamental commands.

### Ten fundamental Git commands

|**Command**                                        | **Purpose**
|---------------------------------------------------|------------
|[git config](https://git-scm.com/docs/git-config)  | Set and view git's configuration parameters
|[git status](https://git-scm.com/docs/git-status)  | Status of local files (which files are changed, which files haven't been added to the repository, etc.)
|[git clone](https://git-scm.com/docs/git-clone)    | Download a remote repository onto your computer
|[git restore](https://git-scm.com/docs/git-restore)| Restore files to their previous state
|[git add](https://git-scm.com/docs/git-add)        | Select files for inclusion into the repository
|[git commit](https://git-scm.com/docs/git-commit)  | Permanently record changes into the repository
|[git log](https://git-scm.com/docs/git-log)        | Review the history of commits
|[git remote](https://git-scm.com/docs/git-remote)  | Link a remote repository on another computer with this local repository
|[git push](https://git-scm.com/docs/git-push)      | Send committed changes to a remote repository
|[git pull](https://git-scm.com/docs/git-pull)      | Download and integrate changes from a remote repository

As you explore git's capabilities [this reference](https://git-scm.com/docs) will be useful.

There is an eleventh command that we won't use in this class...

|**Command**                                        | **Purpose**
|---------------------------------------------------|------------
|[git init](https://git-scm.com/docs/git-init)      | Create a brand-new local repository

In CS 1440 I will always provide you with starter code, so you don't need to use `git init`.  However, you may want to use Git to track assignments for other classes or your own projects.

*   `git init` converts the current directory into a git repository.
    *   Be careful with this command!  Carelessness leads to the creation of Git repositories **inside** other repositories, which is a confusing situation.
    *   Get in the habit of first running `git status` to make sure that you are **NOT** already inside a Git repository.
*   You can give `git init` the name of a new directory to create as an argument.
    *   For example, to create a new repository in the current directory named "English1010", run `git init English1010`


## Pushing content to GitLab

While you could use the web interface to edit files in GitLab, it is more convenient to do your work in a local repository on your computer and use the `git` command to push your work to GitLab.

Before you can do this on your own, you must watch the videos in Canvas that instruct you on installing and setting up Git, and setting up an SSH key for use with GitLab.

In the next document titled [Repository Setup](./Repository_Setup.md) you'll learn how to

*   Create a git repo on your computer
*   Edit code and commit it to the repository
*	Create a new repo on my GitLab server to hold your code
*   Push your changes to the new repo on GitLab
*   Verify that your submission is correct


## Extra Git Tutorials

There are many great resources for learning git in more depth on the web. This is a selection that previous students have recommended. Please use these to deepen your understanding of this important tool.


*   Our GitLab server comes with a bunch of documentation built in.  Check out the
    [Basics Guides](https://gitlab.cs.usu.edu/help/gitlab-basics/README.md#git-basics)
*   This is an in-depth, detailed [Git Tutorial](https://git-scm.com/book/en/v1/Getting-Started) from the official Git site.
*   [Git Documentation and Videos](https://git-scm.com/doc)
*   [Git Command Cheatsheet](https://services.github.com/on-demand/downloads/github-git-cheat-sheet.pdf)
*   [Git Tutorial on Codecademy](https://www.codecademy.com/learn/learn-git)
*   [Getting Git Right](https://www.atlassian.com/git)
*   [Learn Git With Bitbucket Cloud](https://www.atlassian.com/git/tutorials/learn-git-with-bitbucket-cloud)
