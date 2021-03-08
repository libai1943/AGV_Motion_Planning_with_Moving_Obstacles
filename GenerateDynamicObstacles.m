function mat = GenerateDynamicObstacles()
global params_
mat = zeros(params_.Nobs, 5);
index = 0;
while (index <= params_.Nobs)
    r = 1 + rand;
    ox0 = params_.x_min + params_.x_scale * rand;
    oy0 = params_.y_min + params_.y_scale * rand;
    oxf = params_.x_min + params_.x_scale * rand;
    oyf = params_.y_min + params_.y_scale * rand;
    dis1 = hypot(ox0 - params_.x0, oy0 - params_.y0);
    dis2 = hypot(oxf - params_.xf, oyf - params_.yf);
    if ((dis1 < r + params_.radius)||(dis2 < r + params_.radius))
        continue;
    end
    index = index + 1;
    mat(index, 1 : 5) = [ox0 oy0 oxf oyf r];
end
end