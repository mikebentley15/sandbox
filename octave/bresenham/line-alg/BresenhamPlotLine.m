% BresenhamPlotLine - plot line to a binary image with arbitrary endpoints
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
function L = BresenhamPlotLine(I, x1, y1, x2, y2)
  [N, M] = size(I);
  L = I;
  dx = abs(x2 - x1);
  dy = -abs(y2 - y1);
  sx = sign(x2 - x1);
  sy = sign(y2 - y1);
  x = round(x1);
  y = round(y1);
  x2 = round(x2);
  y2 = round(y2);
  err = (1 + y - y1) * dx + (1 + x - x1) * dy;
  while true
    if x > 0 && x <= N && y > 0 && y <= M
      L(x, y) = 1;  % set this pixel
    end
    if [x, y] == [x2, y2]
      break
    end
    e2 = 2 * err;
    if e2 >= dy
      err = err + dy;
      x = x + sx;
    end
    if e2 <= dx
      err = err + dx;
      y = y + sy;
    end
  end
end
