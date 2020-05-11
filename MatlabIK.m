%% Import arm dimensions and constraints from .xlsx file
dataname=convertCharsToStrings(input('Input arm data sheet from folder containing this program:\n','s'));
count=0;
err_count=0;
warning('off','MATLAB:table:ModifiedAndSavedVarnames')
%Makes sure you input data sheet name correctly
while count==err_count
    try
        armdata=readtable(dataname)
    catch
        err_count=err_count+1;
        dataname=convertCharsToStrings(input('Make sure your input matches the name of the file in the folder and try again:\n','s'));
    end
    count=count+1;
end
armdata=armdata{:,:};
%% Convert data from .xlsx file to desired variables
SegmentLength=armdata(:,1)
segmentnumber=numel(SegmentLength);
RotLock=armdata(:,2);
%% Establish user-defined target location
targetx=input('input target x coordinate:\n');
targety=input('input target y coordinate:\n');
targetz=input('input target z coordinate:\n');
targetpos=[targetx;targety;targetz];
%Plot target position
scatter3(targetx,targety,targetz,'k');
hold on
%% Establish variables for base configuration
%Roll
tx=zeros(1,segmentnumber);
%Pitch
ty=zeros(1,segmentnumber);
%Yaw
tz=zeros(1,segmentnumber);
%Current Location of end effector
Displacement_total=0;
%Identity matrix to be built upon
R=[1 0 0; 0 1 0; 0 0 1];
%Base of segment
P0=[0;0;0];
%Deltas used for finding gradients
deltax=0.01;
deltay=0.01;
deltaz=0.01;
%% Build Arm in base config
for n=1:segmentnumber
    Rn=BuildRotMatrix(tx,ty,tz,n);
    if n==1
        R=Rn;
    else
        R(:,:,n)=R(:,:,n-1)*Rn;
    end
    Segment(:,:,n)=SegmentLength(n,:).*[0;0;1];
    Displacementn=R(:,:,n)*Segment(:,:,n);
    Displacement_total=Displacement_total+Displacementn
    %graph segment
    plot3([P0(1),Displacement_total(1)],[P0(2),Displacement_total(2)],[P0(3),Displacement_total(3)],'b');
    P0=Displacement_total;
end
%Calculate distance to target
f=Dist2Target(Displacement_total,targetpos);
%initial distance to target, for debug purposes only
f0=f;
%% IK
thresh=0.15
L=0.1
while  f>thresh
    for m=1:segmentnumber
        %find f(tx(m)+deltax)
        txg=tx;
        txg(m)=tx(m)+deltax;
        Displacement_totalGX=0;
        for n=1:segmentnumber
            RGXn=BuildRotMatrix(txg,ty,tz,n);
            if n==1
                RGX=RGXn;
            else
                RGX(:,:,n)=RGX(:,:,n-1)*RGXn;
            end
            DisplacementGXn=RGX(:,:,n)*Segment(:,:,n);
            Displacement_totalGX=Displacement_totalGX+DisplacementGXn;
        end
        fGX(m)=Dist2Target(Displacement_totalGX,targetpos);
        %find f(ty(m)+deltay)
        tyg=ty;
        tyg(m)=ty(m)+deltay;
        Displacement_totalGY=0;
        for n=1:segmentnumber
            RGYn=BuildRotMatrix(tx,tyg,tz,n);
            if n==1
                RGY=RGYn;
            else
                RGY(:,:,n)=RGY(:,:,n-1)*RGYn;
            end
            DisplacementGYn=RGY(:,:,n)*Segment(:,:,n);
            Displacement_totalGY=Displacement_totalGY+DisplacementGYn;
        end
        fGY(m)=Dist2Target(Displacement_totalGY,targetpos);
        %find z gradients
        tzg=tz;
        tzg(m)=tz(m)+deltaz;
        Displacement_totalGZ=0;
        for n=1:segmentnumber
            RGZn=BuildRotMatrix(tx,ty,tzg,n);
            if n==1
                RGZ=RGZn;
            else
                RGZ(:,:,n)=RGZ(:,:,n-1)*RGZn;
            end
            DisplacementGZn=RGZ(:,:,n)*Segment(:,:,n);
            Displacement_totalGZ=Displacement_totalGZ+DisplacementGZn;
        end
        fGZ(m)=Dist2Target(Displacement_totalGZ,targetpos);
    end
    %Estimate gradients from distances calculated previously
    gradx=(fGX-f)/deltax;
    grady=(fGY-f)/deltay;
    gradz=(fGZ-f)/deltaz;
    %Update angles
    for n=1:segmentnumber
        LRX(n)=L*gradx(n)/norm(gradx(n))
        LRY(n)=L*grady(n)/norm(grady(n))
        LRZ(n)=L*gradz(n)/norm(gradz(n))
        %actually update angles
        if norm(tx(n)-LRX)<=RotLock(n)
            tx(n)=tx(n)-L*gradx(n)
        end
        if norm(ty(n)-LRY)<=RotLock(n)
            ty(n)=ty(n)-L*grady(n)
        end
        if norm(tz(n)-LRZ)<=RotLock(n)
            tz(n)=tz(n)-L*gradz(n)
        end
    end
    %Build arm with updated angles
    Displacement_total=0
    for n=1:segmentnumber
        if n==1
            P0=[0;0;0];
        end
        Rn=BuildRotMatrix(tx,ty,tz,n);
        if n==1
            R=Rn;
        else
            R(:,:,n)=R(:,:,n-1)*Rn;
        end
        Segment(:,:,n)=SegmentLength(n,:).*[0;0;1];
        Displacementn=R(:,:,n)*Segment(:,:,n);
        Displacement_total=Displacement_total+Displacementn
    end
    %Calculate new distance to target
    f=Dist2Target(Displacement_total,targetpos);
    %If new distance to target is greater than threshold, loop repeats
end
%final config of angles has been found
%% Build and plot final config
Displacement_total=0
for n=1:segmentnumber
    if n==1
        P0=[0;0;0];
    end
    Rn=BuildRotMatrix(tx,ty,tz,n);
    if n==1
        R=Rn;
    else
        R(:,:,n)=R(:,:,n-1)*Rn;
    end
    Segment(:,:,n)=SegmentLength(n,:).*[0;0;1];
    Displacementn=R(:,:,n)*Segment(:,:,n);
    Displacement_total=Displacement_total+Displacementn
    %graph arm
    plot3([P0(1),Displacement_total(1)],[P0(2),Displacement_total(2)],[P0(3),Displacement_total(3)],'r');
    P0=Displacement_total;
end

