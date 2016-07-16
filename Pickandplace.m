% defining length of the  each link of the arm ----------------------------
l1=9;
l2=8;
l3=16;

% Interfacing with Arduino and setting up default pose--------------------- 
 a = arduino('COM7');                      % connecting arduino on com17 
 servoAttach(a,5);                          % initailizing Servo 
 servoWrite(a,5,90);                       
 pause(.5)
 servoAttach(a,6);
 servoWrite(a,6,90);
 pause(.5)
 servoAttach(a,9);
 servoWrite(a,9,90);
 pause(.5)
 servoAttach(a,10);
 servoWrite(a,10,90);
 pause(.5)
 servoAttach(a,11);                          % Servo for gripper
 servoWrite(a,11,30);
 pause(.5)
 servoAttach(a,3);                           % Servo for base
 servoWrite(a,3,90);
 pause(.5)
 %-------------------------------------------------------------------------
 
 clear thetai;
 clear s;
 clear d;
 c=0;
 r=0;
 
 imaqreset          % Disconnect and delete all image acquisition object 


L(1) = Link([0 0 l1 0]);                             %---------------------------------------------
L(2) = Link([0 0 l2 0]);                             %
L(3) = Link([0 0 l3 0]);                             % Initializing the arm links with RVC tool box
ThreeLink = SerialLink(L);                           %--------------------------------------------- 
ThreeLink.name = 'Planar3R';

vid = videoinput('winvideo', 2, 'YUY2_640x480');     %-----------------------------------------
src = getselectedsource(vid);                        %
vid.ReturnedColorspace = 'rgb';                      %  Settings for Video and frame aquisition
triggerconfig(vid, 'manual');                        %
vid.FramesPerTrigger = 1;                            %-----------------------------------------


vid.ROIPosition = [113 34 430 430];                  % Defining Region of Interest.(experimental, varies with the setup) 

p1 =[2   430    13   411                             
     8     6   420   426                             % Source matrix for 2D homographic transform (experimental, pixels from current view)
     1     1     1     1];

 p2 =[1   430     1   430
     1     1   430   430                             % Destination matrix for 2D homographic transform (required pixel locations)
     1     1     1     1];
 
 H = homography2d(p1, p2);                           % Calculating transformation matrix, This helps to remove any kind of distortions in the current view.

preview(vid)

while(1)                                             % Main loop for image processing and inverse kinematics 
 start(vid);
 
    in = input('>> Press "0" for exit ; "1" for capute : >>  ');           % Taking input from user
    if (in == 0)
    break;
    else 
    trigger(vid);
    frame = getdata(vid);
    corr = imTransD(frame,H,[430,430],'lh');                               % Transforming the image using homgraphic transformation matrix
    figure()
    imshow(corr);
    
    [i,j,k] = size(corr);
    corr1 = zeros([i,j]);
    corr2 = zeros([i,j]);
    corr3 = zeros([i,j]);
    
    % for finding red color of ball----------------------------------------
    for m = 1:i
        for n = 1:j
            if( (corr(m,n,1) >= 220) && (corr(m,n,2) <= 150) && (corr(m,n,3) <= 200))           % Thresholding for red pixels 
                corr1(m,n) = 255;
            else
                corr1(m,n) = 0;
            end
        end
    end
  
     BW = edge(corr1,'canny');                                             % Edge detection
    [c,r] = imfindcircles(BW,[10,25]);                                     % for ball
     
   
     viscircles(c, r,'EdgeColor','b');
     x1 = (215-c(2))*0.209;                                                % here multiplying value .209 depends on your cameras height from the base
     y1 = (215-c(1))*0.209;
   
    %for finding green color of the destination bowl ----------------------
    for m = 1:i
        for n = 1:j
            if( (corr(m,n,1) <= 50) && (corr(m,n,2) >= 100) && (corr(m,n,3) <100 ))            % Thresholding for green pixels
                corr2(m,n) = 255;
            else
                corr2(m,n) = 0;
            end
        end
    end

     BW = edge(corr2,'canny');
                                                     
%     [c2,r2] = imfindcircles(BW,[10,25]);                                   % for the green bowl
%     
%       viscircles(c2, r2,'EdgeColor','b');
%      
%     x2 = (215-c2(2))*0.209;                                                % green base coordinates
%     y2= (215-c2(1))*0.209;
    
   
    x = sqrt(x1^2 + y1^2);                                                 % Diagonal distance from basr of the arm to center of the ball (horizontal distance)
    y = -.5;                                                                 % y=0 since ball is on the ground (vertical distance)
    
    thetai= 180 - atan2d(y1,x1);                              %-------------------------------------------------------------------------------------------
    if x>=13&&x<=24                                          %
        h=pi/4;                                             %
    elseif x>=25&&x<=29                                    %
        h=pi/6;                                           %
    elseif x>=30&&x<=32                                  % selecting end effector orientaion with x, orienations are obtained using trial and error method
        h=pi/12;                                          %
    elseif x>32&&x<=33                                     %
       h=pi/24;                                             %
    elseif x>33&&x<=34                                       %
        h=0;                                                  %------------------------------------------------------------------------------------------
    end                                                                                                   
         if x>34||x<13
             disp('Arm cannot reach')                                      % Max distance arm can reach is 33cm
         end
         
   b = ikine3r([l1 l2 l3],[x y -h],1);                                     % applying inverse kinematics for given ball coordinates

   d = toDegrees('radians',b); 
   s = d + [0 90 90];                                                      % adding servo offsets to servo angles in degrees
   if ((min(s)<0)||(max(s)>180))
    disp('Not possible');
   end
% view(0,90);                    
% ThreeLink.plot(b);               

  % Writing the values to each servo motors of the arm --------------------
    
    servoWrite(a,5,90);
    servoWrite(a,11,30);
    pause(1)

    servoWrite(a,3,ceil(0.96*thetai));
    servoWrite(a,5,round(s(1)));
    pause(.5)
    servoWrite(a,6,round((s(2))));
    pause(.5)
    servoWrite(a,9,180-ceil((s(3))));
    pause(1)
    servoWrite(a,11,90);               % closing the gripper/ ball grabbed
    pause(1)
    servoWrite(a,5,90);
    servoWrite(a,6,90);
    servoWrite(a,9,90);
    servoWrite(a,3,90);
   % ---------------------------------------------------------------------
   
   % perfoming the same operations for moving towards the destination bowl
   
%       x = sqrt(x2^2 + y2^2);
%       y = 0;
%     thetai= 180 - atan2d(y2,x2);
%     if x>=13&&x<=24
%         h=pi/4;
%     elseif x>=25&&x<=29
%         h=pi/6;
%     elseif x>=30&&x<=32
%         h=pi/12;
%     elseif x>32&&x<=33
%        h=pi/24;
%     elseif x>33&&x<=34
%         h=0;
%     end
%          if x>34||x<13
%              disp('Arm cannot reach')
%          end
%          
%    b = ikine3r([l1 l2 l3],[x y -h],1);                                     % applying inverse kinematics for given bowl coordinates
%    d = toDegrees('radians',b);
%    s = d + [0 90 90];
%    if ((min(s)<0)||(max(s)>180))
%     disp('Not possible');
%    end
% %view(0,90);
% %ThreeLink.plot(b);
% 
%   % Writing the values to each servo motors of the arm --------------------
%     
%     servoWrite(a,3,ceil(0.96*thetai));
%     servoWrite(a,5,round((s(1))));
%     pause(.5)
%     servoWrite(a,6,round((s(2))));
%     pause(.5)
%     servoWrite(a,9,90+ceil((s(3))));
%     pause(1)
%     servoWrite(a,11,30);        % opening the gripper/ ball placed in bowl            
%     pause(1)
%     
%     servoWrite(a,5,90);
%     servoWrite(a,6,90);
%     servoWrite(a,9,90);
%     servoWrite(a,3,90);
     
   % ----------------------------------------------------------------------
     end
end
 
stoppreview(vid)
imaqreset