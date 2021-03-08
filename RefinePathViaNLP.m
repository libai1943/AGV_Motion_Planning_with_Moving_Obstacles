function [x, y, ready_flag] = RefinePathViaNLP(x, y)
WriteObsFile();
WriteNLPInfo();
WriteCoarsePath(x, y);
WriteIG(x, y);

ready_flag = 0;
!ampl r0.run
load x.txt;
load y.txt;
load flag.txt
if (flag == 1)
    ready_flag = 1;
end
end

function WriteObsFile()
global params_
obs_x = [];
obs_y = [];
for ii = 1 : params_.Nobs
    obs_x = [obs_x; linspace(params_.obs(ii,1), params_.obs(ii,3), params_.Nfe)];
    obs_y = [obs_y; linspace(params_.obs(ii,2), params_.obs(ii,4), params_.Nfe)];
end

delete('obs');
fid = fopen('obs', 'w');
for ii = 1 : params_.Nobs
    for jj = 1 : params_.Nfe
        fprintf(fid, '%g  %g  %g  %f \r\n', ii, jj, 1, obs_x(ii, jj));
        fprintf(fid, '%g  %g  %g  %f \r\n', ii, jj, 2, obs_y(ii, jj));
        fprintf(fid, '%g  %g  %g  %f \r\n', ii, jj, 3, params_.obs(ii, 5));
    end
end
fclose(fid);
end

function WriteNLPInfo()
global params_
delete('NLPinfo');
fid = fopen('NLPinfo', 'w');
fprintf(fid, '%g %f \r\n', 1, params_.Nfe);
fprintf(fid, '%g %f \r\n', 2, params_.Nobs);
fprintf(fid, '%g %f \r\n', 3, params_.x0);
fprintf(fid, '%g %f \r\n', 4, params_.y0);
fprintf(fid, '%g %f \r\n', 5, params_.xf);
fprintf(fid, '%g %f \r\n', 6, params_.yf);
fprintf(fid, '%g %f \r\n', 7, params_.radius);
fprintf(fid, '%g %f \r\n', 8, params_.tf_max);
fprintf(fid, '%g %f \r\n', 9, params_.x_min);
fprintf(fid, '%g %f \r\n', 10, params_.x_max);
fprintf(fid, '%g %f \r\n', 11, params_.y_min);
fprintf(fid, '%g %f \r\n', 12, params_.y_max);
fclose(fid);
end

function WriteIG(x, y)
global params_
hi = params_.tf_max / (params_.Nfe - 1);

x_full = [];
y_full = [];
for ii = 1 : (length(x) - 1)
    temp = linspace(x(ii), x(ii + 1), 4);
    x_full = [x_full, temp(1 : 3)];
    temp = linspace(y(ii), y(ii + 1), 4);
    y_full = [y_full, temp(1 : 3)];
end
x_full = [x_full, x_full(end), x_full(end), x_full(end)];
y_full = [y_full, y_full(end), y_full(end), y_full(end)];
x = x_full;
y = y_full;

dx = 0;
dy = 0;
for ii = 2 : length(x)
    dx = [dx, (x(ii) - x(ii - 1)) / hi];
    dy = [dy, (y(ii) - y(ii - 1)) / hi];
end

ddx = 0;
ddy = 0;
for ii = 2 : length(x)
    ddx = [ddx, (dx(ii) - dx(ii - 1)) / hi];
    ddy = [ddy, (dy(ii) - dy(ii - 1)) / hi];
end

delete('ig.INIVAL');
fid = fopen('ig.INIVAL', 'w');
for ii = 1 : params_.Nfe
    fprintf(fid,'let x[%g] := %f;\r\n', ii, x(ii));
    fprintf(fid,'let y[%g] := %f;\r\n', ii, y(ii));
    fprintf(fid,'let dx[%g] := %f;\r\n', ii, dx(ii));
    fprintf(fid,'let dy[%g] := %f;\r\n', ii, dy(ii));
    fprintf(fid,'let ddx[%g] := %f;\r\n', ii, ddx(ii));
    fprintf(fid,'let ddy[%g] := %f;\r\n', ii, ddy(ii));
end
fclose(fid);
end

function WriteCoarsePath(x, y)
global params_
delete('path');
fid = fopen('path', 'w');
for ii = 1 : params_.NT
    fprintf(fid, '%g  1  %f \r\n', ii, x(ii));
    fprintf(fid, '%g  2  %f \r\n', ii, y(ii));
end
fclose(fid);
end

