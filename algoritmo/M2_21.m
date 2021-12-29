function m2_21 = M2_21(alpha123,alpha312,alpha231,Dalpha231,Dalpha123,Dalpha312)
%return 2x2 matrix
I2=[1 0;
    0 1];
rotT=rot(alpha231).';
rotT2=rot(alpha231+pi/2).';
m2_21=Dalpha123*cos(alpha123)*I2 - Dalpha312*cos(alpha312)*rotT - Dalpha231*sin(alpha312)*rotT2;
end