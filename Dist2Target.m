function Dist = Dist2Target(Displacement_total,targetpos)
%Finds distance between point A and point B
Dist=sqrt((Displacement_total(1)-targetpos(1))^2+(Displacement_total(2)-targetpos(2))^2+(Displacement_total(3)-targetpos(3))^2);
end