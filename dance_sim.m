close all
global l_shoulder l_hip l_elbow l_arm l_thigh l_leg l_body p_actual l_head
l_shoulder=0.35;
l_hip=0.35;
l_elbow=0.35;
l_arm=0.5;
l_thigh=0.35;
l_leg=0.35;
l_body=0.3;
l_head=0.25;  % Head length parameter
name = 'Fanie_Zumba-01';
[skeleton,time] = loadbvh(name);
frames=skeleton(1).Nframes;

thz = zeros(26, frames);  % Changed to store parameters as columns

%Neck
p0=skeleton(17).Dxyz;
%Head
p_head=skeleton(18).Dxyz;  % Adding head position from skeleton
%Right Shoulder
p3=skeleton(30).Dxyz;
%Right elbow
p4=skeleton(31).Dxyz;
%Right wrist
p5=skeleton(33).Dxyz;
%Left shoulder
p6=skeleton(21).Dxyz;
%Left elbow
p7=skeleton(22).Dxyz;
%Left wrist
p8=skeleton(24).Dxyz;
%Right hip
p9=skeleton(8).Dxyz;
%Right leg
p10=skeleton(10).Dxyz;
%Right foot
p11=skeleton(11).Dxyz;
%Left hip
p12=skeleton(2).Dxyz;
%Left leg
p13=skeleton(4).Dxyz;
%left foot
p14=skeleton(5).Dxyz;



for i=1:15:frames
    fprintf('Processing frame %d of %d\n', i, frames);
    l_shoulder=norm(p3(:,i)-p6(:,i))/2;
    l_hip=norm(p9(:,i)-p12(:,i))/2;
    l_elbow=norm(p3(:,i)-p4(:,i));
    l_arm=norm(p4(:,i)-p5(:,i));
    l_thigh=norm(p9(:,i)-p10(:,i));
    l_leg=norm(p10(:,i)-p11(:,i));
    l_body=norm(p0(2:3,i)-p12(2:3,i));
    p_actual = [p0(:,i) p_head(:,i) p3(:,i) p4(:,i) p5(:,i) p6(:,i) p7(:,i) p8(:,i) p9(:,i) p10(:,i) p11(:,i) p12(:,i) p13(:,i) p14(:,i)]; 
    x0=[0 0 0,0,0 0 0,0 ,0 0 0,0, 0 0 0,0,0 0 0,0,0 0 0,p0(:,i)']';  % Added 3 angles for head
    if i>1
        x0=thz(:, i-1);
    end
    [thz(:,i), ~] = lsqnonlin(@human2, x0);  % Store only the optimized parameters
end


for i=1:15:frames
    cla;
    human1(thz(:,i));
    drawnow;
    pause(0.035);
end








function out1=human2(thz)
global l_shoulder l_hip l_elbow l_arm l_thigh l_leg l_body p_actual l_head
p0=thz(24:26);  % Updated index for p0
p3=[0;-l_shoulder;0];
p4=[l_elbow;0;0];
p5=[l_arm;0;0];

p6=[0;l_shoulder;0];
p7=[l_elbow;0;0];
p8=[l_arm;0;0];

p9 =[0;-l_hip;-l_body];
p10=[l_thigh;0;0];
p11=[l_leg;0;0];

p12 =[0;l_hip;-l_body];
p13=[l_thigh;0;0];
p14=[l_leg;0;0];


R_th1=rot_Z(thz(1))*rot_Y(thz(2))*rot_X(thz(3));
p3_i = R_th1*p3 +p0;
p6_i=R_th1*p6 +p0;
p9_i=R_th1*p9 +p0;
p12_i=R_th1*p12 +p0;

R_th3=rot_Z(thz(5))*rot_Y(thz(6))*rot_X(thz(7));
R_th4=rot_Y(thz(8));
p4_i=R_th1*R_th3*p4 + p3_i;
p5_i=R_th1*R_th3*R_th4*p5+p4_i;
R_th5=rot_Z(thz(9))*rot_Y(thz(10))*rot_X(thz(11));
R_th6=rot_Y(thz(12));
p7_i=R_th1*R_th5*p7 + p6_i;
p8_i=R_th1*R_th5*R_th6*p8+p7_i;

R_th7=rot_Z(thz(13))*rot_Y(thz(14))*rot_X(thz(15));
R_th8=rot_Y(thz(16));
p10_i=R_th1*R_th7*p10 + p9_i;
p11_i=R_th1*R_th7*R_th8*p11+p10_i;

R_th9=rot_Z(thz(17))*rot_Y(thz(18))*rot_X(thz(19));
R_th10=rot_Y(thz(20));
p13_i=R_th1*R_th9*p13 + p12_i;
p14_i=R_th1*R_th9*R_th10*p14+p13_i;

p_head = [0;0;l_head];  % Head position relative to neck
R_head = rot_Z(thz(21))*rot_Y(thz(22))*rot_X(thz(23));  % Head rotation
p_head_i = R_th1*R_head*p_head + p0;  % Final head position

z0=[p0 p_head_i p3_i p4_i p5_i p6_i p7_i p8_i p9_i p10_i p11_i p12_i p13_i p14_i];
 out1 = reshape(z0 - p_actual, [], 1);
end


function human1(thz)
global l_shoulder l_hip l_elbow l_arm l_thigh l_leg l_body l_head
p0=thz(24:26);  % Updated index for p0
p3=[0;-l_shoulder;0];
p4=[l_elbow;0;0];
p5=[l_arm;0;0];

p6=[0;l_shoulder;0];
p7=[l_elbow;0;0];
p8=[l_arm;0;0];

p9 =[0;-l_hip;-l_body];
p10=[l_thigh;0;0];
p11=[l_leg;0;0];

p12 =[0;l_hip;-l_body];
p13=[l_thigh;0;0];
p14=[l_leg;0;0];


R_th1=rot_Z(thz(1))*rot_Y(thz(2))*rot_X(thz(3));
p3_i = R_th1*p3 +p0;
p6_i=R_th1*p6 +p0;
p9_i=R_th1*p9 +p0;

p12_i=R_th1*p12 +p0;

R_th3=rot_Z(thz(5))*rot_Y(thz(6))*rot_X(thz(7));
R_th4=rot_Y(thz(8));
p4_i=R_th1*R_th3*p4 + p3_i;
p5_i=R_th1*R_th3*R_th4*p5+p4_i;
R_th5=rot_Z(thz(9))*rot_Y(thz(10))*rot_X(thz(11));
R_th6=rot_Y(thz(12));
p7_i=R_th1*R_th5*p7 + p6_i;
p8_i=R_th1*R_th5*R_th6*p8+p7_i;

R_th7=rot_Z(thz(13))*rot_Y(thz(14))*rot_X(thz(15));
R_th8=rot_Y(thz(16));
p10_i=R_th1*R_th7*p10 + p9_i;
p11_i=R_th1*R_th7*R_th8*p11+p10_i;

R_th9=rot_Z(thz(17))*rot_Y(thz(18))*rot_X(thz(19));
R_th10=rot_Y(thz(20));
p13_i=R_th1*R_th9*p13 + p12_i;
p14_i=R_th1*R_th9*R_th10*p14+p13_i;

p_head = [0;0;l_head];  % Head position relative to neck
R_head = rot_Z(thz(21))*rot_Y(thz(22))*rot_X(thz(23));  % Head rotation
p_head_i = R_th1*R_head*p_head + p0;  % Final head position

%Plot
line_w=6;

% Draw head
[x,y,z]=point4plot(p0,p_head_i);
plot3(x,y,z,'b','LineWidth',line_w);

% Draw head sphere
[X,Y,Z] = sphere(20);
r = l_head/3;  % Head radius
X = X*r + p_head_i(1);
Y = Y*r + p_head_i(2);
Z = Z*r + p_head_i(3);
surf(X,Y,Z,'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');
alpha(0.8);  % Make sphere slightly transparent

vert1=[p3_i p6_i p9_i p12_i p3_i];
x=vert1(1,:);
y=vert1(2,:);
z=vert1(3,:);
patch(x,y,z,'b'); axis equal; hold on;

[x,y,z]=point4plot(p3_i,p4_i);
plot3(x,y,z,'r','LineWidth',line_w);
[x,y,z]=point4plot(p5_i,p4_i);
plot3(x,y,z,'g','LineWidth',line_w);
[x,y,z]=point4plot(p6_i,p7_i);
plot3(x,y,z,'r','LineWidth',line_w);
[x,y,z]=point4plot(p8_i,p7_i);
plot3(x,y,z,'g','LineWidth',line_w);
[x,y,z]=point4plot(p6_i,p7_i);
plot3(x,y,z,'r','LineWidth',line_w);

[x,y,z]=point4plot(p9_i,p10_i);
plot3(x,y,z,'r','LineWidth',line_w);
[x,y,z]=point4plot(p11_i,p10_i);
plot3(x,y,z,'y','LineWidth',line_w);
[x,y,z]=point4plot(p12_i,p13_i);
plot3(x,y,z,'r','LineWidth',line_w);
[x,y,z]=point4plot(p14_i,p13_i);
plot3(x,y,z,'y','LineWidth',line_w);





end

function [x,y,z]=point4plot(x1,x2)
    x=[x1(1);x2(1)];
    y=[x1(2);x2(2)];
    z=[x1(3);x2(3)];
end

function y = rot_X(alpha)
    y= [1 0 0; 0 cos(alpha) -sin(alpha) ;0 sin(alpha) cos(alpha)];
end
function y = rot_Z(alpha)
    y= [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0;0 0 1];
end
function y = rot_Y(alpha)
    y= [cos(alpha) 0 sin(alpha);0 1 0; -sin(alpha) 0 cos(alpha)];
end