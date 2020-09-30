% BresenhamPlot3LineInt - plot line to a binary image with integer endpoints
% On input:
%     I (NxMxK array): Input binary image
%     a (3x1 ints): first point of line
%     b (3x1 ints): second point of line
% On output:
%     L (NxMxK array): Copy of I with the line added in
% Example:
%     L = BresenhamPlot3LineInt(zeros(5,5,5), [1, 1, 1], [4, 5, 3]);
% Author:
%     Michael Bentley
%     2020 September 29
%
function L = BresenhamPlot3LineInt(I, a, b)
  L   = I;
  s   = sign(b - a);
  dp  = abs(b - a);
  dm  = max(dp);
  i   = dm;
  err = (dm / 2) * b;

  while true
    L(a(1), a(2), a(3)) = 1;
    i = i - 1;
    if i == 0
      break
    end
    for j = [1:3]
      err(j) = err(j) - dp(j);
      if err(j) < 0
        err(j) = err(j) + dm;
        a(j) = a(j) + s(j);
      end
    end
  end
end
