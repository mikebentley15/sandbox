function U = upsample(I, mult)
  U = zeros(size(I)*mult);
  for i = [1:mult]
    for j = [1:mult]
      U(i:mult:end, j:mult:end) = I;
    end
  end
end
