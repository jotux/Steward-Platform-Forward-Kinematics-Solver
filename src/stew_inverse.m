% Filename: stew_inverse.m
% Author: Joe Brown, California State University, Sacramento
% Description: This file contains a function to find the inverse kinematics
% solution for a Stewart platform.
% Definitions of frames A, B, G, M, and N used in this program are given in the report: Kumagai, A and Brown, J.P “Development of a Closed Form Forward Kinematics Analysis Program for the 6-DOF CDSL Stewart Platform,” Aug. 2006.
% Inputs: 
%         xsi - x coordinates of base connection points
%         ysi - y coordinates of base connection points
%         xmi - x coordinates of platform connection points
%         ymi - y coordinates of platform connection points
% Note: See the FORTRAN program ‘hard-init.f’ for full-definitions of xsi, ysi, xmi, ymi.
%         roll - x rotation angle in degrees
%         pitch - y rotation angle in degrees
%         yaw - z rotation angle in degrees
% Note: In this program, x(roll), y(pitch), and z(yaw) rotation angles are Euler angels THE6(1), THE6(2), and THE6(3), respectively. The THE6 vectors are described in the FORTRAN simulation programs for the 6-DOF Stewart platform.
% For definitions of parameters px, py, pz, and vectors ai (i=1,..,6), and bi (i=1,..,6), see the following technical paper: Lee, T.Y, and Shim J.K., “Algebraic Elimination-Based Real-Time Forward Kinematics of the 6-6 Stewart Platform with Planar Base and Platform,” Proceedings of the IEEE International Conference on Robotics and Automation, Seoul, Korea, May 21-26 2001, pp. 1301-1306.
% Outputs: 
%         Legs[6] - Leg lengths for legs 1 through 6
%         platcoords[18] - x,y, and z coordinates for the platform with respect to Frame N
%         animcoords[18] - x,y, and z coordinates for the platform with a
%           different z translation to make it look accurate when
%           animating or drawing.

function inv_return=stew_inverse(xsi,ysi,xmi,ymi,roll,pitch,yaw,px,py,pz)
    digits(15)                           %digit accuracy specification
    a1=[xsi(1);ysi(1);0;1];              
    a2=[xsi(2);ysi(2);0;1];              
    a3=[xsi(3);ysi(3);0;1];              
    a4=[xsi(4);ysi(4);0;1];              
    a5=[xsi(5);ysi(5);0;1];
    a6=[xsi(6);ysi(6);0;1]; 

    TXrad=roll*pi/180;                  %convert the roll angle to radians
    TYrad=pitch*pi/180;                 %convert the pitch angle to radians
    TZrad=yaw*pi/180;                   %convert the yaw angle to radians

        %T is the transformation (translation + rotation) matrix from Frame B to Frame N.
    T =[cos(TYrad)*cos(TZrad),                                 -cos(TYrad)*sin(TZrad),                                  sin(TYrad),           px;
        sin(TXrad)*sin(TYrad)*cos(TZrad)+cos(TXrad)*sin(TZrad),-sin(TXrad)*sin(TYrad)*sin(TZrad)+cos(TXrad)*cos(TZrad),-sin(TXrad)*cos(TYrad),py;
       -cos(TXrad)*sin(TYrad)*cos(TZrad)+sin(TXrad)*sin(TZrad), cos(TXrad)*sin(TYrad)*sin(TZrad)+sin(TXrad)*cos(TZrad), cos(TXrad)*cos(TYrad),pz-111.31;
        0,                                                      0,                                                      0,                    1 ];
        
        %Ta is the transformation matrix that rotates and translates the
        %platform to the correct orientation for animation.
    Ta =[cos(TYrad)*cos(TZrad),                                 -cos(TYrad)*sin(TZrad),                                  sin(TYrad),           px;
         sin(TXrad)*sin(TYrad)*cos(TZrad)+cos(TXrad)*sin(TZrad),-sin(TXrad)*sin(TYrad)*sin(TZrad)+cos(TXrad)*cos(TZrad),-sin(TXrad)*cos(TYrad),py;
        -cos(TXrad)*sin(TYrad)*cos(TZrad)+sin(TXrad)*sin(TZrad), cos(TXrad)*sin(TYrad)*sin(TZrad)+sin(TXrad)*cos(TZrad), cos(TXrad)*cos(TYrad),111.31-pz;
         0,                                                      0,                                                      0,                    1 ];
    
    b1=T*[xmi(1);ymi(1);0;1];   %b# is the coordinates that will be used for calculation
    b2=T*[xmi(2);ymi(2);0;1];
    b3=T*[xmi(3);ymi(3);0;1];
    b4=T*[xmi(4);ymi(4);0;1];
    b5=T*[xmi(5);ymi(5);0;1];
    b6=T*[xmi(6);ymi(6);0;1];

    b1t=Ta*[xmi(1);ymi(1);0;1]; %b#t are the coordinates that will be animated
    b2t=Ta*[xmi(2);ymi(2);0;1];
    b3t=Ta*[xmi(3);ymi(3);0;1];
    b4t=Ta*[xmi(4);ymi(4);0;1];
    b5t=Ta*[xmi(5);ymi(5);0;1];
    b6t=Ta*[xmi(6);ymi(6);0;1];

    L1=sqrt((abs(a1(1)-b1(1)))^2+(abs(a1(2)-b1(2)))^2+(abs(a1(3)-b1(3)))^2); %L=sqrt((ax-bx)^2+(ay-by)^2+(az-bz)^2)
    L2=sqrt((abs(a2(1)-b2(1)))^2+(abs(a2(2)-b2(2)))^2+(abs(a2(3)-b2(3)))^2);
    L3=sqrt((abs(a3(1)-b3(1)))^2+(abs(a3(2)-b3(2)))^2+(abs(a3(3)-b3(3)))^2);
    L4=sqrt((abs(a4(1)-b4(1)))^2+(abs(a4(2)-b4(2)))^2+(abs(a4(3)-b4(3)))^2);
    L5=sqrt((abs(a5(1)-b5(1)))^2+(abs(a5(2)-b5(2)))^2+(abs(a5(3)-b5(3)))^2);
    L6=sqrt((abs(a6(1)-b6(1)))^2+(abs(a6(2)-b6(2)))^2+(abs(a6(3)-b6(3)))^2);

    Legs=[L1,L2,L3,L4,L5,L6];   %leg value return
    platcoords=[b1(1),b1(2),b1(3),b2(1),b2(2),b2(3),b3(1),b3(2),b3(3),b4(1),b4(2),b4(3),b5(1),b5(2),b5(3),b6(1),b6(2),b6(3)]; %plat position return
    animcoords=[b1t(1),b1t(2),b1t(3),b2t(1),b2t(2),b2t(3),b3t(1),b3t(2),b3t(3),b4t(1),b4t(2),b4t(3),b5t(1),b5t(2),b5t(3),b6t(1),b6t(2),b6t(3)]; %plat position return for animation
    inv_return=[Legs,platcoords,animcoords]; %return

