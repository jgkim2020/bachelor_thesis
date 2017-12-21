function [phi_z, phi_xy] = quaternSplit(q)
    [rowSize colSize] = size(q);
    phi_z = zeros(rowSize,1);
    phi_xy = zeros(rowSize,1);
    for i = 1:rowSize
      if (q(i,1) < 0)
        q(i,:) = -q(i,:);
      end
      % get phi_z (range: -pi ~ pi)
      phi_z(i) = 2*atan2(q(i,4), q(i,1));
      % get phi_xy (range: -pi ~ pi)
      if(abs(cos(phi_z(i)/2)) > abs(sin(phi_z(i)/2)))
        phi_xy(i) = 2*acos(q(i,1)/cos(phi_z(i)/2));
      else
        phi_xy(i) = 2*acos(q(i,4)/sin(phi_z(i)/2));
      end
      if(phi_xy(i) > pi)
        phi_xy(i) = 2*pi - phi_xy(i);q(1)
      end
    end
end
