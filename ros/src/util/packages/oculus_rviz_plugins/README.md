oculus_rviz_plugins
===========

This plugin is based on [OgreOculus](https://bitbucket.org/rajetic/ogreoculus) (C++)
and [OsgOculusViewer](https://github.com/bjornblissing/osgoculusviewer) (GLSL shaders).

Usage Instructions
==================

Download the catkin_package [ros_ovr_sdk](https://github.com/OSUrobotics/ros_ovr_sdk.git) into your workspace and run catkin_make.

Before using the OculusDisplay, run rosrun OculusSDK/build/bin/ovrd in a sourced terminal.

In RViz, add the "OculusDisplay". This will create an additional window with a stereo rendering
of the contents of the main RViz rendering area. Check "Render to Oculus" to 
render in full screen mode on your Oculus headset. It must be set up as secondary screen
for this to work.

By default, the Oculus view will be rendered from the same position as the main RViz camera while following
your head's orientation. Alternatively, you can attach the camera to a tf frame.

This is how the Display should look like in windowed mode:

![ScreenShot](doc/screenshot.png)

See [the Wiki](https://github.com/OSUrobotics/oculus_rviz_plugins/wiki) for more details.
