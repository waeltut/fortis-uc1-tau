# Rebuilding with the tested TF fix

Replace your project's .devcontainer directory with this directory, then use
VS Code: **Dev Containers: Rebuild Container Without Cache**.
Preserve any maps or other files stored only inside the old container first;
the existing workspace mount persists src, not the entire home/workspace.

The Dockerfile builds geometry2 0.25.24 from the verified release commit
404b7224d623d614f18fa9738dbf1716403d857e into /opt/tf2_fix.
It checks the commit, package version and selected package prefix during build.
This replaces the manual ~/tf2_fix_ws overlay with an image-owned installation.
The apt TF packages remain installed for dependency resolution. Seeing 0.25.23
in dpkg-query is therefore possible; ROS must select /opt/tf2_fix instead.
Other existing image dependencies are unchanged and are not all version pinned.

The shared /etc/ros/ros2_setup.bash loads Humble, the application workspace,
then the fixed TF overlay. Interactive Bash, the entrypoint and the devcontainer
workspace build use it. Custom noninteractive launch scripts should source it
as well. The entrypoint no longer appends duplicate lines to .bashrc.
Nav2 and nav2_bringup are now explicitly installed. No robot launch files,
maps, initial poses, footprints or navigation parameters were changed.

## Verify after rebuilding

In a new container terminal:

```bash
ros2 pkg prefix tf2
ros2 pkg prefix tf2_ros
grep '<version>' /opt/tf2_fix/share/tf2_ros/package.xml
cat /opt/tf2_fix/geometry2-commit.txt
```

Both prefixes must be /opt/tf2_fix and the version must be 0.25.24.
Start your usual MiR/localization/Nav2 launch and leave it running for several
minutes. Confirm /local_costmap/published_footprint and
/global_costmap/published_footprint timestamps continue advancing.

After this succeeds, remove any manually added ~/tf2_fix_ws sourcing lines
from persistent shell or launch scripts. The old ~/tf2_fix_ws directory can
then be deleted if it contains only the temporary geometry2 build. Remove
manual FASTRTPS_DEFAULT_PROFILES_FILE / FASTDDS_DEFAULT_PROFILES_FILE exports
pointing to /tmp/mur_udp_only.xml if you kept that diagnostic experiment.
None of these temporary overrides were present in the supplied configuration.
Do not revert your corrected map, footprint or localization settings.

Validated here: Bash syntax, JSONC parsing, release tag commit resolution.
A full Docker/ARM64 image build was not possible in the editing environment.





VERY IMPORTANT NOTE: REVERT BACK TO OLD SETUP ONCE TF2-ROS RELEASES VERSION 0.25.24 OR NEWER!!!