# PiranhaBot
### Directories
- Packages for controller device go in `/SurfaceDevice` folder.
- Config files for all packages go in the `/config` or `/SurfaceDevice/config` folder.
---
### Commits
When making commits, please use this style for descriptions ([more info](http://wiki.ros.org/RecommendedRepositoryUsage)):
```
package_name: Fixed tearing when zooming in on moving images.
```
When a commit is made, the file `/CMakeLists.txt` will be used to try to build the files. On the first commit for the branch, **change this build file to add your main C++ file!** If the build does not succeed, don't sweat it. It's just there for convienience, and only needs to be ok when creating pull requests.
