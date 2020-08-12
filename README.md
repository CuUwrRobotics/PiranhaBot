# PiranhaBot
### Directories
- Packages for controller device go in `/SurfaceDevice` folder.
- Config files for any packages go in the `/config` or `/SurfaceDevice/config` folder.
---
### Issues

Issues color-coded tags, where certain colors for tags denote what the tag is for. This is the color code for common tags:
- ![#7fe3e1](https://via.placeholder.com/15/7fe3e1/000000?text=+) `Teal`: How advanced the programming work is.
- ![#1d76db](https://via.placeholder.com/15/1d76db/000000?text=+) `Blue`: The type of issue.
- ![#eeeeee](https://via.placeholder.com/15/eeeeee/000000?text=+) `Grey`: What topic the issue mainly covers.

Other colors are used for some tags, like the red `vital` tag.

### Commits
When making commits, please name the package in the title. ([More info](http://wiki.ros.org/RecommendedRepositoryUsage#Commits_and_Pull_Requests)):
```
package_name
Fixed tearing when zooming in on moving images.
```
When a commit is made, the file `/CMakeLists.txt` will be used to try to build the files. On the first commit for the branch, **change this build file to add your main C++ files.** If the build does not succeed, don't sweat it. It's just there for convienience, and only needs to be ok when creating pull requests.
