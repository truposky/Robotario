function m2_12 = M2_12(alpha123,alpha312,Dalpha123,Dalpha312)
%return 2x2 matrix
% need alpha123,alpha312 and derivate of them
rotT=rot(alpha312).';
rotT2=rot(alpha312+pi/2).';
m2_12=-Dalpha123.*cos(alpha123).*rotT-Dalpha312.*sin(alpha123).*rotT2;
end