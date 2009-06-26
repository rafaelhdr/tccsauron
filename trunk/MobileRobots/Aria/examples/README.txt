This directory has various example pieces of code and Microsoft VC++
project files.  

The two most useful example programs are teleopActionsExample and 
wander.  TeleopActionsExample does guarded teleoperation, so you 
can drive the robot around without running into walls.  
Wander just has the robot wander, ie drive forward until there is 
an obstacle, avoid it, then keep driving. 

The smallest, simplest example program is simpleConnect. It just
connects to the robot, prints some information, then disconnects
and exits.


Partial list of examples:
-------------------------------------------------------------------------------


actionExample - Program defines a couple of actions and uses them.

actionGroup - Program that uses action groups to switch back and forth 
between wander and teleop mode

actsColorFolowingExample - A simple program that uses ACTS and a VC-C4 camera to move the
robot toward a color blob.

demo - Uses "Modes" defined in ARIA to provide keyboard control of many
different robot features. Use to experiment and test the robot's 
hardware.

directMotionDemo - Program that drives the robot around using the direct
motion commands (no Actions or obstacle avoidance). it also runs the
robot in its own thread, and has examples of connection handler callbacks.

dpptuDemo - A program to control the Directed Perceptions PTU with the keyboard.

functor - An example program on the basic use of functors.

getAuxExample - An example program that uses the getAux command and
talks about how to use the getAux to do actual work

gotoActionExample - An example of how to use ArActionGoto to go to many different
points and not just one

gpsExample - An example showing how to get data from a GPS

gripperDemo - Program that moves the robot and controls the gripper with the 
joystick, note this doesn't do obstacle avoidance

joydriveActionExample - Uses an action that reads the robot to drive the joystick,
does not do obstacle avoidance

joydriveThreaded - Program to drive the robot with a joystick. This one
uses its own ArASyncTask to drive the joystick handler.  This is a good
example to look at to see how threading works.  This does not do obstacle
avoidance, this also has a connection handler

joydriveUserTask - Program to drive the robot with a joystick. This one
uses a user task to drive the joystick handler, does not do obstacle avoidance

armExample - demonstrates how to connect with the p2 controller and
set up the arm class

moduleExample - Tests the loadable modules in a simple way

robotSyncTaskExample - simple example of ArRobot synchronized task callbacks

socketClientExample - This program works with socketServerExample to
demonstrate ArSockets

socketServerExample - A program to demonstrate ArSocket, works with
socketClientExample

sickRobotExample - Connects to the sick laser and the robot and prints
out some readings

sickTeleop - Uses the sick laser to do a simple teleoperated drive

sickWander - Uses the sick laser to do a simple wander program

sickLineFinderSimple - Just does an example of using the sick line finding
class and lets you save the lines

sickLineFindingComplex - Finds the lines with the sick then tries to
find a 90 degree (outward) corner to drive to

simpleConnect - The smallest example program. It just connects to the robot.

simpleUserTask - Demonstrates how to make a simple user task

sonyPTZDemo - Program that moves the sony camera and drives the robot with the
joystick, note this doesn't do obstacle avoidance

soundsQueueExamples - Demonstrates use of the ArSoundsQueue sound/speech queue

teleopActionsExample - This uses ARIA's powerful Actions system to drive 
the robot around using input from the keyboard or a joystick, but does 
obstacle avoidance so the robot won't run into things (if it can sense 
them with sonar or laser). 

threadExample - Demonstrates ARIA's threading tools

vcc4Demo - A program to control the Canon VC-C4 PTZ camera with the keyboard.

wander - Makes the robot wander around

wanderAndLogData - similar to wander, but prints out all kinds of runtime
information about the robot

