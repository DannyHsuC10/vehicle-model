function track = loadFSAETrackCSV(csvFile, trackWidth, resampleDistance)
%LOADFSAETRACKCSV Read an FSAE ideal racing line CSV for MPCC tracking.
%
% The CSV is expected to contain Point, X, and Y columns. X/Y are used
% directly as the target path, matching python/track/track_geo.py:
%   Track(csv_file=..., track_width=3.0, resample_distance=1)
%
% The returned struct matches the existing MPCC track format:
%   track.center, track.inner, track.outer are all 2 x N arrays.
% Here track.center is the ideal racing line target, not a path that needs
% additional racing-line generation.

if nargin < 2 || isempty(trackWidth)
    trackWidth = 3.0;
end

if nargin < 3
    resampleDistance = 1.0;
end

if ~isfile(csvFile)
    error('loadFSAETrackCSV:fileNotFound', 'Track CSV not found: %s', csvFile);
end

data = readtable(csvFile);
if all(ismember({'X','Y'}, data.Properties.VariableNames))
    x = data.X;
    y = data.Y;
else
    numericData = table2array(data(:, vartype('numeric')));
    if size(numericData, 2) < 3
        error('loadFSAETrackCSV:invalidFormat', ...
            'Expected Point, X, Y numeric columns in %s.', csvFile);
    end
    x = numericData(:, 2);
    y = numericData(:, 3);
end

x = x(:);
y = y(:);
valid = isfinite(x) & isfinite(y);
x = x(valid);
y = y(valid);

if numel(x) < 3
    error('loadFSAETrackCSV:notEnoughPoints', ...
        'The target path requires at least three valid points.');
end

stepLength = hypot(diff(x), diff(y));
keep = [true; stepLength > eps];
x = x(keep);
y = y(keep);

if numel(x) < 3
    error('loadFSAETrackCSV:notEnoughUniquePoints', ...
        'The target path requires at least three unique points.');
end

if ~isempty(resampleDistance) && resampleDistance > 0
    dx = gradient(x);
    dy = gradient(y);
    ds = hypot(dx, dy);
    s = [0; cumsum(ds(1:end-1))];
    [s, uniqueIdx] = unique(s, 'stable');
    x = x(uniqueIdx);
    y = y(uniqueIdx);

    totalLength = s(end);
    if totalLength <= 0
        error('loadFSAETrackCSV:badGeometry', ...
            'Unable to compute arc length for target path.');
    end

    sNew = (0:resampleDistance:(totalLength - eps(totalLength)))';
    x = pchip(s, x, sNew);
    y = pchip(s, y, sNew);
    s = sNew;
else
    ds = hypot(diff(x), diff(y));
    s = [0; cumsum(ds)];
end

dx = gradient(x);
dy = gradient(y);
tangentLength = hypot(dx, dy);

if any(tangentLength <= eps)
    error('loadFSAETrackCSV:badGeometry', ...
        'Unable to compute target path normals from repeated points.');
end

tx = dx ./ tangentLength;
ty = dy ./ tangentLength;
normal = [-ty'; tx'];
halfWidth = trackWidth / 2;
targetPath = [x(:)'; y(:)'];

arcStep = hypot(dx, dy);
s = [0; cumsum(arcStep(1:end-1))];

ddx = gradient(dx);
ddy = gradient(dy);
curvatureDenominator = (dx.^2 + dy.^2).^1.5;
curvature = (dx .* ddy - dy .* ddx) ./ curvatureDenominator;

track.center = targetPath;
track.inner = targetPath + halfWidth * normal;
track.outer = targetPath - halfWidth * normal;
track.width = trackWidth;
track.s = s(:)';
track.length = s(end);
track.psi = atan2(dy, dx)';
track.curvature = curvature(:)';
track.start_x = targetPath(1, 1);
track.start_y = targetPath(2, 1);
track.start_heading = track.psi(1);
track.start_pose = [track.start_x; track.start_y; track.start_heading];
track.source = csvFile;
track.borderSplineMode = 'direct';
track.pathType = 'ideal_racing_line';
end
