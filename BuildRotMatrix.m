function Rn = BuildRotMatrix(tx,ty,tz,n)
RX=[1,0,0;0,cosd(tx(n)),-sind(tx(n));0,sind(tx(n)),cosd(tx(n))];
RY=[cosd(ty(n)),0,sind(ty(n));0,1,0;-sind(ty(n)),0,cosd(ty(n))];
RZ=[cosd(tz(n)),-sind(tz(n)),0;sind(tz(n)),cosd(tz(n)),0;0,0,1];
Rn=RX*RY*RZ;
end

