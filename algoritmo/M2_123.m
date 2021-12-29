function m2_123=M2_123(alpha123,alpha231,Dalpha123,Dalpha231,m2_12,m2_21)
I2=[1 0;
    0 1];
m11=-Dalpha231*cos(alpha231)*I2;
m12=m2_12;
m21=m2_21;
m22=Dalpha123*cos(alpha123)*I2;

m2_123=[m11 m12;m21 m22];
end