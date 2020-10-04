% BresenhamPlotLineInt - plot line to a binary image with integer endpoints
% On input:
%     I (nxm array): Input binary image
%     x1 (int): x-coordinate of first point
%     y1 (int): y-coordinate of first point
%     x2 (int): x-coordinate of second point
%     y2 (int): y-coordinate of second point
% On output:
%     L (nxm array): Copy of I with the line added in
% Example:
%     L = BresenhamPlotLineInt(zeros(5), 1, 1, 4, 5);
% Author:
%     Michael Bentley
%     2020 September 29
%
function L = BresenhamPlotLineInt(I, x1, y1, x2, y2)
  L = I;
  dx = abs(x2 - x1);
  dy = -abs(y2 - y1);
  sx = sign(x2 - x1);
  sy = sign(y2 - y1);
  err = dx + dy;
  while true
    L(x1, y1) = 1;  % set this pixel
    if [x1, y1] == [x2, y2]
      break
    end
    e2 = 2 * err;
    if e2 >= dy
      err = err + dy;
      x1 = x1 + sx;
    end
    if e2 <= dx
      err = err + dx;
      y1 = y1 + sy;
    end
  end
end
