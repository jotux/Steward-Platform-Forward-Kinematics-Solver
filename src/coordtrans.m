% Filename: coordtrans.m
% Author: Akihiko Kumagai
% Co-Author: Joe Brown, California State University, Sacramento  8/04/2006
% Description: This file contains a function to transform coordinate systems used in the forward kinematics analysis based on the Lee’s paper to those used in the CDSL Stewart platform system.
% Definitions of frames A, B, G, M, and N used in this program are given in the report: Kumagai, A and Brown, J.P “Development of a Closed Form Forward Kinematics Analysis Program for the 6-DOF CDSL Stewart Platform,” Aug. 2006.
% Descriptions of transformation matrices:
% 	TGA: translation matrix from Frame G to Frame A
% 	TMB: translation matrix from Frame M to Frame B
% 	TBA: transformation (translation + rotation) matrix from Frame B to Frame A.
% 	TGA: translation matrix from Frame G to Frame N
% Descriptions of vectors:
% rm: the vector from the origin of Frame N to that of Frame M
% pm: position vector with respect to Frame M
% TXrad, TYrad, and TZrad rotation angles are Euler angels THE6(1), THE6(2), and THE6(3), respectively. The THE6 vectors are described in the FORTRAN simulation programs for the 6-DOF Stewart platform.

function coordtrans_return=coordtrans(TXdeg,TYdeg,TZdeg,px,py,pz,XSI,YSI,XMI,YMI)
    TXrad=TXdeg*pi/180;
    TYrad=TYdeg*pi/180;
    TZrad=TZdeg*pi/180;
 
    TGA=[1,0,0,-XSI(1);
         0,1,0,-YSI(1);
         0,0,1,0;
         0,0,0,1];

    TMB=[1,0,0,-XMI(1);
         0,1,0,-YMI(1);
         0,0,1,0;
         0,0,0,1];

    TBA =[cos(TYrad)*cos(TZrad),                                 -cos(TYrad)*sin(TZrad),                                  sin(TYrad),           px;
          sin(TXrad)*sin(TYrad)*cos(TZrad)+cos(TXrad)*sin(TZrad),-sin(TXrad)*sin(TYrad)*sin(TZrad)+cos(TXrad)*cos(TZrad),-sin(TXrad)*cos(TYrad),py;
         -cos(TXrad)*sin(TYrad)*cos(TZrad)+sin(TXrad)*sin(TZrad), cos(TXrad)*sin(TYrad)*sin(TZrad)+sin(TXrad)*cos(TZrad), cos(TXrad)*cos(TYrad),pz;
          0,                                                      0,                                                      0,                    1 ];

    TGN=[1 0 0 0; 
         0 1 0 0; 
         0 0 1 111.31];
     
    TAG=inv(TGA);
    
    pm=[0;0;0;1];
    
    rm=vpa(TGN*TAG*TBA*TMB*pm);
    
    coordtrans_return=rm;

