% BresenhamPlotLine_Conservative - plot line to a binary image with arbitrary endpoints
% On input:
%     I (nxm array): Input binary image
%     x1 (float): x-coordinate of first point
%     y1 (float): y-coordinate of first point
%     x2 (float): x-coordinate of second point
%     y2 (float): y-coordinate of second point
% On output:
%     L (nxm array): Copy of I with the line added in
% Example:
%     L = BresenhamPlotLineInt(zeros(5), 1.1, -2.1, 4.3, 5.0);
% Author:
%     Michael Bentley
%     2020 September 29
%
function L = BresenhamPlotLine_Conservative(I, x1, y1, x2, y2)
  [N, M] = size(I);
  L = I;
  dx = abs(x2 - x1);
  dy = abs(y2 - y1);
  sx = sign(x2 - x1);
  sy = sign(y2 - y1);
  x = round(x1);
  y = round(y1);
  x2 = round(x2);
  y2 = round(y2);
  err = (y - y1) * dx - (x - x1) * dy;
  cutoff = (dy - dx) / 2.0;
  while true
    if x > 0 && x <= N && y > 0 && y <= M
      L(x, y) = 1;  % set this pixel
    end
    if [x, y] == [x2, y2]
      break
    end
    if err <= cutoff
      err = err + dx;
      y = y + sy;
    else
      err = err - dy;
      x = x + sx;
    end
  end
end
