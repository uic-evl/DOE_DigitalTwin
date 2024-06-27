# Unity Samples

There are three Unity sample projects that demonstrate the digital twin capabilities of the Unity game engine. 

The first, `Unity_Sample_Win10/DigitalTwin` uses the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) [ROS TCP Connection](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md) to publish and subscribe to ROS Topics from the Create3 roomba. 

`Unity_ZMQ_to_Arm\DigitalTwin_Unity` demonstrates connecting a Unity application to Omniverse Isaac Sim for robot simulation with the Elepahnt Robotics arm. 

`Unity_ZMQ_VR\VR` adapts the former project to include VR interactions using Unity's [XR Interaction Toolkit](https://github.com/Unity-Technologies/XR-Interaction-Toolkit-Examples/tree/classic/2.2) package. 

All of these projects require some level of outside system to work properly. Please navigate to the README of each project for more details on how to operate. 

## Using Unity

### File Structure
To use these projects, clone or download this repository. Then, you can add each project to the Unity Hub. On the Unity Hub Projects tab,  click Add > Add project from disk. Then, navigate to wherever you have cloned this repository and locate the base directory of the Unity project you with to open. The base directory of a Unity project is the folder that contains, among other folders, the "Assets" folder. 

Unity projects have several file structure conventions. 
* Projects will always have an `Assets` folder, and it is within this folder than most of the important user-developed files will go. 
* Folders at the same level as `Assets` typically pertain to the added Packages, Plugins, Libraries, and things of the like. 
* Within the `Assets` folder, there are several standard folders that—if not present when a new project is created—will be created by the user. Some of the more common of these folders include:
  * `Scripts` - This is where all of your C# scripts will go
  * `Scenes` - This is where you save all of the scenes in the project. In Unity, a "scene" refers to the 3D or 2D stage where you build your application. Complicated games are often many scenes, where each scene is a different level. 
  * `Shaders` - If you are programming custom shaders, the code or ShaderGraphs are saved here. 
  * `Resources` - Files (such as text files, CSV files, or audio files) that need to be loaded by the application can be placed in the Resources folder. This allows you to use Unity's built-in [Resource loading](https://docs.unity3d.com/ScriptReference/Resources.Load.html) functions, and is often more reliable than using C# file management functions. 
  * `Materials` - Simple materials (standard shaders) go here. 
  * `Meshes` or `Models` - Any imported meshes are saved here
  * `Textures` - Any imported textures are saved here. 

### Unity Versions

Once you have added your project, Unity will detect which editor version is required. You will want to make sure you have the exact editor version installed before you open a project, as many Unity features and packages vary between supported versions. 

For our projects, we are using Unity version 2020.3.48f1, as this is the version compatible with both the Unity Robotics Hub packages and the Vicon package. 

### Editing scripts in Unity 

To edits scripts used in your Unity project, you can use any text editor of your choice. By default, Unity will use Visual Studio, which is typically installed alongside Unity when you download an editor. Visual Studio is fairly heavy IDE, so it is often easier to switch to Visual Studio Code. 

To switch the default editor used by your Unity editor, go to "Edit > Preferences > External Tools" and update the "External Script Editor" to be your text editor of choice. 

To open a script, you can double-click on it in the Unity editor, or access it through the file explorer of your computer and find it in the "Assets/Scripts" folder. 

### Safe Mode

Sometimes when opening a new Unity project, you are promopted to open in "Safe Mode". This means that the project contains compiler errors. Often, these errors are due to missings packages that will be loaded once the project fully initiallizes, or that can be easily added from the editor once it has launched. For this reason, it is usually best to "Ignore" the request to open in safe mode, and then check the console to resolve compiler errors yourself. 

### Some Debugging Tips

The Unity Console will show you errors and warnings. Some errors are compiler errors that prevent the code from compiling. Other errors are encountered while the application is running and may (but not always) halt the execution of the app. Depending on the error, and the function of your code, a given error may or may not be important. Read error carefully when they appear, and clear to console often. Many packages will clutter the console with irrelevant errors. 

