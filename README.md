# ros-template

This repository is a template for a containerized [ROS 2](https://docs.ros.org/) project. It uses `make` to implement the following utility commands:

| Command                  | Description                                                                                                                   |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------- |
| `make build`             | Builds the colcon workspace.                                                                                                  |
| `make test`              | Tests the colcon workspace.                                                                                                   |
| `make dev`               | Enters a bash shell inside the container containing the project's dependencies and build environment.                         |
| `make final`             | Builds the colcon workspace and copies the resulting artifacts into a minimal `ros-core` image tagged `{name}:latest-final`.  |
| `make refresh`           | Causes the previous set of commands to rebuild the image when run.                                                            |
| `make clean`             | Untags and removes any installed images.                                                                                      |
| `make update-submodules` | Recursively updates git submodules.                                                                                           |

## Why?

Working in ROS is often unnecessarily challenging due to the ecosystem relying on [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) to map the dependency keys listed in projects' `package.xml` to those in linux distributions' repositories. This causes ROS distributions to be pinned to specific releases of specific linux distributions. Because individual ROS installations are often very complex and coupled to specific versions of ubuntu, it is often desirable to run the entire environment inside a docker container.

However, developing inside a containerized environment presents a very large pit of potential mistakes. Features like proper caching, x11 forwarding, a properly permissioned workspace volume, and even smaller things like an ergonomic development environment are all easy to mess up and cumbersome to implement.

## Structure

This template project acts as the ros/colcon workspace, with all packages to be built going in `src`.

`Containerfile` contains build instructions for several stages of a container build, which can be used independently. `Makefile` contains the afformentioned scripts for using the container image. The images produced by the makefile's commands are:

- `{name}:latest-base` (for `make build`), extends `ros-{distro}-desktop-full`
- `{name}:latest-dev` (for `make dev`), extends `base`
- `{name}:latest-final` (for `make final`), extends `{distro}-ros-core`

Where `{name}` is to be replaced with your project's name.

Make commands that rely on a specific image existing (`make build`, `make test`, `make dev`) will build the image first if it does not already exist. Afterwards, they will only rebuild the image if changes have been made to the `Containerfile`, or if they are forced to rebuild (`make refresh`, `make clean`). Since the images share most of the same layers, only your first command should take long, and the rest should be relatively quick. `make build` will also mount your entire project into `/workspace`, meaning build artifacts (`build`, `install`, `log`) are cached by default.

`make dev` will, by default, bridge the following between the host and container:

- `/workspace` volume with rw access
- host user namespace
- host network
- X11

All the commands and syntax used is intended to be compatible with [podman](https://podman.io/), [docker](https://www.docker.com/), or any other OCI complaint container engine.

## Usage

To set up the template, replace `ROS_DISTRO` and `REPO_NAME` in `Makefile` with your desired distro and project name. If you're using devcontainers, replace `ROS_DISTRO` in `./container/devcontainer.json` as well.

### Installing Packages

This template, by default, does not support ros-specific tooling for dependency or repository management. This is for several reasons, but the most important one is that we can only guarantee ros-specific tooling exists inside the container's ros environment. This means tools such as `rosdep` and `vcstool` must be ran on the workspace *inside* the container, necessitating the workspace be copied into the container as part of the `base` image. This would destroy our caching because any changes to the workspace would cause lengthy rebuilds.

The consequence of not supporting tools like `rosdep` inside the container is that, to install packages, you must manually find their `apt` keys and add them to the `apt-get install` block inside the `base` layer of the containerfile. Additionally, if you want to use the `make final` feature, you must add runtime dependencies to the `apt-get install` block inside the `final` layer.

To add source dependencies, use git submodules inside src. For example:

```sh
git submodule add https://github.com/ros2/examples.git src/examples
```

You can view submodules in `.gitmodules` and pin them to specific commits or branches by running `git checkout <commit/branch>` from the specific module's directory.

## Questions and Limitations

> You only explain how to install packages from `apt`. What if I need dependencies that only exist in pypi?

To add additonal dependencies via pip, for example, make a new `RUN` command below that looks something like:

```containerfile
RUN pip install antigravity
```

> Great, but what if I have multiple packages with conflicting versions of dependencies?

A constraint of this setup is that we are completely tied to one specific version of every ros and ubuntu package. Anything that doesn't fit those versions needs to be ported or can't be used.

If the messiness of dependency management bothers you as much as it bothered me, you may want to check out the real solution to these problems, [nix](http://nix.dev/). I will refrain from shilling too hard, but there exists [nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay) to enable working with ros packages from nix. There is also [robostack](https://robostack.github.io/index.html), which is a possibly more mature solution that seeks to address similar problems for ros specifically.

Back to the question: Use conda inside a container at your own risk... I heard the last person who did it was found years later in a shack in the woods.

> I don't use VSCode with devcontainers. How can I edit files with language server support from within the container with \<insert my editor here\>?

Ha

This specific issue is very annoying. So annoying that I wrote a [blog post](https://anglesideangle.dev/blog/container-hell/) about it. In short, a fundamental flaw of container based development is that you, and any other contributors, will need to add their personal tooling to the `dev` stage in the containerfile, and not commit it or hope it doesn't conflict with other contributors'.

If this again bothers you, both nix and pixi (the tool that robostack is based on) fix this problem by not installing your dependencies inside a full isolated system from the host.

> It sounds like you really dislike containers and container based development. Why did you make this?

This template only depends on a container engine, git, and make. These, unlike nix or pixi/conda, are tools almost everyone, regardless of their os, has installed on their system. Perhaps one day things may be different, but until then, I am but a servant to network effects.
