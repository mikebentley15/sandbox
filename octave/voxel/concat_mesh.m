function FV = concat_mesh(FV1, FV2)
  FV = struct('vertices', [FV1.vertices; FV2.vertices]);
  FV.triangles = [FV1.triangles; size(FV1.vertices, 1) + FV2.triangles];
end
