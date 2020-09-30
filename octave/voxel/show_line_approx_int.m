% function I = show_line_approx_int(N, mult, a, b)
%
% Input:
%   N (int): dimension of the smaller image
%   mult (int): multiple of N for the larger image (to show the approximation)
%   a (3x1 ints): index of pixel center in smaller image for line start
%   b (3x1 ints): index of pixel center in larger image for line end
%
% Output:
%   FV Faces and vertices of the combined ones
%
function FV = show_line_approx_int(N, mult, a, b)
  % populate the smaller one
  smaller = BresenhamPlot3LineInt(zeros(N, N, N), a, b);
  %smaller2 = BresenhamPlot3LineInt_Conservative(zeros(N, N, N), a, b);

  % populate the larger one
  q = floor((mult+1)/2 + (a-1)*mult);
  p = floor((mult+1)/2 + (b-1)*mult);
  M = N*mult;
  larger = BresenhamPlot3LineInt(zeros(M, M, M), q, p);

  a'
  b'
  smaller

  FV1 = Voxel2mesh(smaller, 3);
  FV2 = Voxel2mesh(larger, 3);
  FV2.vertices = (FV2.vertices/mult - (mult+1)/2);

  patch (FV1, 'FaceColor', 'red', 'FaceAlpha', 0.1);
  hold on;
  patch (FV2, 'FaceColor', 'green', 'FaceAlpha', 0.1);
  hold off;

  %I = zeros(N*mult, N*mult, 3);
  %I(:, :, 1) = upsample(smaller, mult);  % red
  %I(:, :, 2) = larger;                   % green
  %%I(:, :, 3) = upsample(smaller2, mult); % blue

end
