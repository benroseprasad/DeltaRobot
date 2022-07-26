clear;clc;

%Geometry of the robot
r = 26; %Moving plate radius
R = 56; %Base radius
D = 95; %Length of arm
h = 299; %Length of leg
alpha = [0,120,240];

%Coordinates
P = [0;-200;200];  
x = P(1); y = P(2); z = P(3); 

for i = 1:3
    % Note:angles are in degrees
    U(i) = -2*D*(x*cosd(alpha(1))+y*sind(alpha(1))+r-R);
    V(i) = -2*D*z;
    W(i) = x^2 + y^2 + z^2 + D^2 - h^2 + (r-R)^2 + 2*(r-R)*(x*cosd(alpha(1))+y*sind(alpha(1)));
endfor

for i = 1:3
    th_a(i) = rad2deg(2*atan2(-V(i)+sqrt(U(i)^2+V(i)^2-W(i)^2),(W(i)-U(i))));
    th_b(i) = rad2deg(2*atan2(-V(i)-sqrt(U(i)^2+V(i)^2-W(i)^2),(W(i)-U(i))));
endfor

th_b
% Compute points for plotting 
for i = 1:3
    Q_alpha(:,:,i) = [cosd(alpha(i)) -sind(alpha(i)) 0; sind(alpha(i)) cosd(alpha(i)) 0; 0 0 1];
    a(:,i) = Q_alpha(:,:,i)*[R;0;0];
    b(:,i) = P+Q_alpha(:,:,i)*[r;0;0];
    d(:,i) = a(:,i)+Q_alpha(:,:,i)*[D*cosd(th_b(i));0;D*sind(th_b(i))];
endfor

%Plotting
clf;
TopPlate = [a,a(:,1)];
plot3(TopPlate(1,:),TopPlate(2,:),TopPlate(3,:))
hold on
BottomPlate = [b,b(:,1)];
plot3(BottomPlate(1,:),BottomPlate(2,:),BottomPlate(3,:))

% Plotting three chains
for i = 1:3
    Chain = [a(:,i),d(:,i),b(:,i)];
    plot3(Chain(1,:),Chain(2,:),Chain(3,:))
endfor
axis equal


% Plot a straight line trajectory
P1 = [0;-100;200];
P2 = [0;100;200];
line1 = [P1,P2];
plot3(line1(1,:),line1(2,:),line1(3,:),'r')

% Trajectory generation
distance = sqrt((P1-P2)'*(P1-P2)); %distance btwn points
travel_time = 2; % seconds
no_points = 10 % points
velocity_vector = (P2-P1)/travel_time

for i = 0:no_points
    Positions(:,i+1)=P1 +  i*travel_time/no_points*velocity_vector;
end


