clc
clear all
close all
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.55')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');

imgMsg = rosmessage('sensor_msgs/Image');
imagepub = rospublisher('/imagefrommatlab','sensor_msgs/Image');
imagesub = rossubscriber('/imagefrompython','sensor_msgs/Image');

while(1)
    I = imread('mandril1.jpg');
    
    imgMsg.Encoding = 'rgb8'; % Specifies Image Encoding Type
    writeImage(imgMsg,I); % Convert image to ROS message
    send(imagepub,imgMsg); % send the image

    try
        I2 = receive(imagesub,5); % Receive an image on the /imagefrompython topic
        I2 = readImage(I2); % Convert message to a matlab array
        figure(1), imshow(I2) % Show the image
    catch
        disp('not receiving anything')
    end
end