function refPoses = RefSample(zdata, udata)
% refPoses: Every row is a pose dimension, Every col is a time step
    delta_s = 1; % The sampling size

    %% RefPath is the optimized path
    RefPath.PathSegments = zdata;
    RefPath.Radius = udata(1,:);
    RefPath.Angle = udata(2,:);
    for j = 1:size(udata,2)
        if udata(1,j)==0
            % According to |path_planner|, this means the start and 
            % end point have same heading angle
            % The section length is recorded in u(2)
            RefPath.Length(j) = udata(2,j);
        else
            % According to |path_planner|, this means there is difference 
            % between the start and end pose
            % The arc length is calculated by Angle * Raduis
            RefPath.Length(j) = abs(udata(1,j)*udata(2,j));
        end
    end

    total_length = sum(RefPath.Length);
    k = 1;
    t = 1;
    refPoses(:,t) = RefPath.PathSegments(:,1);
    Path_s = [0];
    for s = delta_s:delta_s:total_length
        Path_s = [Path_s;s];
        t = t+1;
        if s>sum(RefPath.Length(1:k))
            k = k+1;
            if any(RefPath.PathSegments(:,k)~=refPoses(:,t-1))
                diff = s-sum(RefPath.Length(1:k-1));
                refPoses(:,t) = RefPath.PathSegments(:,k);
                Path_s(end) = [s-diff];       
                Path_s = [Path_s;s];
                t = t+1;
            end
        end
        delta_s_mod = Path_s(end)-Path_s(end-1);
        if RefPath.Radius(k)~=0
            gamma = delta_s_mod/(RefPath.Angle(k)*RefPath.Radius(k));
            refPoses(1,t) = refPoses(1,t-1) - RefPath.Radius(k)*sin(refPoses(3,t-1))+RefPath.Radius(k)*sin(refPoses(3,t-1)+gamma*RefPath.Angle(k));
            refPoses(2,t) = refPoses(2,t-1) + RefPath.Radius(k)*cos(refPoses(3,t-1))-RefPath.Radius(k)*cos(refPoses(3,t-1)+gamma*RefPath.Angle(k));
            refPoses(3,t) = refPoses(3,t-1) + gamma*RefPath.Angle(k);
        else
            refPoses(1,t) = refPoses(1,t-1) + delta_s_mod*cos(refPoses(3,t-1));
            refPoses(2,t) = refPoses(2,t-1) + delta_s_mod*sin(refPoses(3,t-1));
            refPoses(3,t) = refPoses(3,t-1);
        end
    end
    if any(RefPath.PathSegments(:,end)~=refPoses(:,t))
        refPoses(:,t+1) = RefPath.PathSegments(:,end);
        Path_s = [Path_s;total_length];
    end

end