% ?Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

map=im2bw(imread('map5.bmp')); % input map read from a bmp file. for new maps write the file name here
%im2bw()使用阈值 threshold 变换法把灰度图像转换成二值图像
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000; 
display=true; % display of RRT

%%%%% parameters end here %%%%%

tic;%tic与toc用来记录matlab指令执行的时间，tic用来保存当前时间，而后使用toc来记录程序完成的时间
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
% rectangle()函数详解
% rectangle('position',[1,1,5,5],'curvature',[1,1],'edgecolor','r','facecolor','g');
% 'position',[1,1,5,5]表示从（1,1）点开始高为5，宽为5；
% 'curvature',[1,1]表示x,y方向上的曲率都为1，即是圆弧；
% 'edgecolor','r'表示边框颜色是红色；
% 'facecolor','g'表示面内填充颜色为绿色。
RRTree=double([source -1]); % RRT rooted at the source, representation node and parent index
failedAttempts=0;
counter=0;
pathFound=false;
while failedAttempts<=maxFailedAttempts  % loop to grow RRTs
    if rand < 0.5, 
        sample = rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    %RRTree(;,1;2)取出RRTree的第一列和第二列,另(:,1:2:3)则取第1第3列，(:,1:3)则取1,2,3列
    %min函数见help
    closestNode = RRTree(I(1),1:2);
    theta = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end
    if distanceCost(newPoint,goal)<disTh, pathFound=true;break; end % goal reached
    [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh, failedAttempts=failedAttempts+1;continue; end 
    RRTree = [RRTree;newPoint I(1)]; % add node
    failedAttempts=0;
    if display, 
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);
        counter=counter+1;M(counter)=getframe;
    end
end
if display && pathFound 
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;M(counter)=getframe;
end
if display 
    disp('click/press any key');
    waitforbuttonpress; 
end
if ~pathFound, error('no path found. maximum attempts reached'); end
path=[goal];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));