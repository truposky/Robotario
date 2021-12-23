function R = rot(alpha)
%rotation matrix
%   return 2x2 matrix
ca=cos(alpha);
sa=sin(alpha);
R=[ca -sa;
    sa ca];
end