% function I = show_line_approx(N, mult, p1, p2)
%
% Input:
%   N (int): dimension of the smaller image
%   mult (int): multiple of N for the larger image (to show the approximation)
%   p1 (2x1 ints): index of pixel center in smaller image for line start
%   p2 (2x1 ints): index of pixel center in larger image for line end
%
% Output:
%   I ((N*mult)x(N*mult)x3): A larger image of size N*mult^2 with three
%     channels.  The smaller image will be upscaled to the larger image after
%     the line is drawn and will be drawn in red.  The larger image will have
%     its line be in green.  Where they overlap, the colors will be added.
%
function I = show_line_approx(N, mult, p1, p2)
  % populate the smaller one
  smaller = BresenhamPlotLineInt(zeros(N), p1(1), p1(2), p2(1), p2(2));
  smaller2 = BresenhamPlotLineInt_Conservative(zeros(N), p1(1), p1(2), p2(1), p2(2));

  % populate the larger one
  q1 = floor((mult+1)/2 + (p1-1)*mult);
  q2 = floor((mult+1)/2 + (p2-1)*mult);
  larger = BresenhamPlotLineInt(zeros(N*mult), q1(1), q1(2), q2(1), q2(2));

  I = zeros(N*mult, N*mult, 3);
  I(:, :, 1) = upsample(smaller, mult);  % red
  I(:, :, 2) = larger;                   % green
  I(:, :, 3) = upsample(smaller2, mult); % blue
end
