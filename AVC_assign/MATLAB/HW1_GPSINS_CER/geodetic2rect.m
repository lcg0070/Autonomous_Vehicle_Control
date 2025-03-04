function [X, Y, Z] = geodetic2rect(lat, lon, lat0, lon0, h0)
    % Reference
    % center of soccerfield
    
    ecef_ref = geod2ecef(lat0, lon0, h0);

    ecef = geod2ecef(lat, lon, h0);
    
    d = ecef - ecef_ref;

    sin_lat = sind(lat0);
    cos_lat = cosd(lat0);
    sin_lon = sind(lon0);
    cos_lon = cosd(lon0);

    R = [-sin_lat * cos_lon,    -sin_lat * sin_lon,     cos_lat;
         -sin_lon,              cos_lon,                0;
         -cos_lat * cos_lon,    -cos_lat * sin_lon,     -sin_lat
    ];
    delta_ned = R * d';
    
    X = delta_ned(1);
    Y = delta_ned(2);
    Z = -delta_ned(3);

end


function [ecef] = geod2ecef(lat, lon, alt)
    a = 6378137.0;            % length of semi-major axis of the ellipsoid (radius at equator)
    b = 6356752.314245;            % length of semi-minor axis of the ellipsoid (radius at the poles)
    f = (a - b) / a; 
    e2 = f * (2-f);
    

    N = a / sqrt(1 - f * (2 - f) * sind(lat)^2);
    X = (N + alt) * cosd(lat) * cosd(lon);
    Y = (N + alt) * cosd(lat) * sind(lon);
    Z = (N * (1 - f)^2 + alt) * sind(lat);
    
    ecef = [X, Y, Z];

end