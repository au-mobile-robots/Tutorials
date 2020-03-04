
%% Connect to the TurtleBot
% Make sure you have a TurtleBot running either in simulation through
% Gazebo(R) or on real hardware.
%% /home/mads
clear all
%rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit

%% 
global BumperMsg
global BumperEvent
BumperEvent = false;

odom = rossubscriber('/odom');
rs_bumper = rossubscriber('/mobile_base/events/bumper', @BumperCallback);

disp('Running...')

tic
while toc < 20
  odomdata = receive(odom);
  pose = odomdata.Pose.Pose;
  x = pose.Position.X;
  y = pose.Position.Y;
  z = pose.Position.Z;

  %[x,y,z]
      
  if BumperEvent
      Str = sprintf('Got event: Bumper %i, State %i',BumperMsg.Bumper,BumperMsg.State);
      disp(Str)
      BumperEvent = false;
  end
end

disp('Stopped...')

%% Disconnect from the Robot
% * It is good practice to clear the workspace of publishers,
% subscribers, and other ROS related objects when you are finished with
% them.
%%
  clear
%%
% * It is recommended to use |rosshutdown| once you are done working with 
% the ROS network. Shut down the global node and disconnect from the
% TurtleBot.
%% 
  rosshutdown
