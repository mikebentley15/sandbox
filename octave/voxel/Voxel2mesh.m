function [FV, vals]=Voxel2mesh(bw, polytype)
% Voxel2mesh - creates triangulated or quad mesh from array; the
% mesh represents the surface faces of boundary voxels in the input array
% (i.e. each voxel face, facing the outside of the object);
% mesh triangles follow the right hand rule with face normals pointing
% outwards
% 
% if the input array is not binary (int or float) optionally a color vector
% can be can be saved. 
%
% the method for cleaning up duplicate vertices was taken from:
%       https://www.mathworks.com/matlabcentral/fileexchange/49691-patch-remesher
% 
%
% Syntax:  
%     FV=Voxel2mesh(bw)
%     FV=Voxel2mesh(bw, polytype)
%     [FV, vals]=Voxel2mesh(labelMatrix)
%     [FV, vals]=Voxel2mesh(labelMatrix, polytype)
% 
% Inputs:
%    bw - R3 array defining objects. Can be binary, int, float. 
%       Background/Outside Voxels must be 0/false
%       All Voxels with values > 0 will be regarded as "Object"
%       (the mesh is generated at the boundary of voxels with value>0 (object)
%       and value==0 (outside))
%    polytype - selection of type of mesh polygon
%       3 -> DEFAULT; triangulated mesh (each square voxel face is made up of 2
%           triangles; 
%       4 -> quad-mesh
%       any other input will default to 3
% 
% Outputs:
%    FV - FV struct; FV.faces/FV.vertices defining the mesh
%    vals - Optional: color vector Nx1 array where N=number of mesh
%          vertices. Values correspond to the value of the voxel which
%          defined the corresponding vertex. In the current version there
%          is no interpolation if a vertex is adjacent to multiple
%          voxels and the vertex is assigned the value of the first voxel
%          that defined it (in clean up step using unique(vv)). 
% 
% Example 1 (binary input): 
%     % take any binary array as input 
%     % in this case I am using the function starship_voxel.m from :
%           https://www.mathworks.com/matlabcentral/fileexchange/75241-build-mesh-or-voxel-spheres-ellipsoid-and-test-objects
%     BW=starship_voxel;
% 
%     % generate triangulated surface mesh
%     FV=Voxel2mesh(BW); 
% 
%     % display
%     figure
%     patch(FV, 'FaceColor', 'b'); 
%     axis equal
% 
% 
% Example 2 (color data):
%     % take any R3 array as input 
%     % in this case I am using the function voxelSphereCreator.m
%     % (https://www.mathworks.com/matlabcentral/fileexchange/75241-build-mesh-or-voxel-spheres-ellipsoid-and-test-objects)
%     % to build a matrix with two overlapping spheres with two different
%     % values
%     
%     bw=zeros(50, 50, 50); 
%     bw=voxelSphereCreator(bw, 10, 15, 25, 25).*2.73425; 
%     bw=voxelSphereCreator(bw, 10, 30, 25, 25); 
%     % generate quad surface mesh and colour vector
%     [FV, c]=T(bw, 4); 
%
%     % display
%     p=patch(FV, 'FaceColor', 'interp', 'facevertexcdata', c); 
%     colormap cool
%     axis equal
% 
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
% 
% Author: J. Benjamin Kacerovsky
% Centre for Research in Neuroscience, McGill University
% email: johannes.kacerovsky@mail.mcgill.ca
% Created: 27-Apr-2020 ; Last revision: 29-May-2020 
% ------------- BEGIN CODE --------------
% set polytype = 4 to get a quad mesh
if nargin<2 
    polytype=3; % default to triangulated mesh
end
bwLogic=bw>0; 
% define cube points relative to voxel coordinates
Vbase=  [-0.5 -0.5 0.5;...
         0.5 -0.5 0.5;...
         0.5 0.5 0.5;...
        -0.5 0.5 0.5;...
         0.5 -0.5 -0.5;...
        -0.5 -0.5 -0.5;...
        -0.5 0.5 -0.5;...
         0.5 0.5 -0.5];
% figure 
% hold on
% t=cellstr(num2str([1:8]'));
% textscatter3(Vbase(:, 1)+0.1, Vbase(:, 2), Vbase(:, 3), t);
% scatter3(Vbase(:, 1), Vbase(:, 2), Vbase(:, 3), 'filled');
% axis equal
% define possible cube faces
Fbase=  [1 2 3 4;...  % front
        2 5 8 3;... % right
        5 6 7 8;... % back
        6 1 4 7;... % left
        6 5 2 1;... % bottom
        8 7 4 3];   % top
% define search neighbourhood
% for each face of a cube around every perimeter voxel we want to assess if
% it's neighbour is outside the bw object. Faces that face the outside of
% the object will be included in the output FV
searcher=[0 0 -1;... % front (by mshifting the outside "backwards" we find voxels with a "free" frontfacing face
          -1 0 0;... % right
          0 0 1;...  % back
          1 0 0;...  % left
          0 1 0;...  % bottom
          0 -1 0];   % top
% Initialize FV
FV.vertices=[];
FV.faces=[];
Outside=~logical(bwLogic);               % inverse of the input array.
perim=logical(bwperim(bwLogic));     % we only have to consider voxels on the perimeter of the object
if nargout==2
    getColor=true;
    vals=[];
else
    getColor=false;  
end
for j=1:6
    % shift Outside array in the direction of searcher(j, :)
    temp=circshift(Outside, searcher(j, 1), 1); 
    temp=circshift(temp, searcher(j, 2), 2); 
    temp=circshift(temp, searcher(j, 3), 3); 
    % find perimeter voxels that have a neighbour outside voxel in this direction 
    temp=temp&perim;
    pts=find(temp); 
    [x, y, z]=ind2sub(size(temp), pts); 
    pts=[x, y, z];
    % base vertices for cube face in direction j
    v=Vbase(Fbase(j, :), :);
    % define new vertices
    vv=zeros(size(pts, 1)*4, 3);
        for i=1:4
            vv(i:4:end, :)=v(i, :)+pts;
        end
    % define new faces
    if polytype==4    % quad-faces if specified
        ff=zeros(4, size(pts, 1));
        ff(:)=1:size(vv, 1); 
        ff=ff';
    else              % triangular faces by default
        ff=zeros(size(pts, 1)*2, 3);
        ff(1:2:end, 1)=1:4:size(vv, 1);
        ff(1:2:end, 2)=2:4:size(vv, 1);
        ff(1:2:end, 3)=3:4:size(vv, 1);
        ff(2:2:end, 1)=1:4:size(vv, 1);
        ff(2:2:end, 2)=3:4:size(vv, 1);
        ff(2:2:end, 3)=4:4:size(vv, 1);
    end
   
    if getColor==true
        valsTemp=bw(temp);
        valsTemp=repmat(valsTemp', 4, 1); 
        valsTemp=valsTemp(:);
%         fprintf('before removal isequal(length(vals), length(vv))=%d\n', isequal(length(vals), length(vv)));
        % remove duplicate vertices 
        [vv, ia, indexn] =  unique(vv, 'rows');
        ff = indexn(ff);
        valsTemp=valsTemp(ia);
        
        % add new vertices and faces to FV
        FV.faces=[FV.faces; ff+size(FV.vertices, 1)];
        FV.vertices=[FV.vertices; vv];
        
        vals=[vals;valsTemp];
       
    else
 
        % remove duplicate vertices 
        [vv, ~, indexn] =  unique(vv, 'rows');
        ff = indexn(ff);
        % add new vertices and faces to FV
        FV.faces=[FV.faces; ff+size(FV.vertices, 1)];
        FV.vertices=[FV.vertices; vv];
        
        % add new vertices and faces to FV
        FV.faces=[FV.faces; ff+size(FV.vertices, 1)];
        FV.vertices=[FV.vertices; vv];
    end
    
    
    
end
% final cleanup
[FV.vertices, ia, indexn] =  unique(FV.vertices, 'rows');
FV.faces = indexn(FV.faces);
    if getColor==true
        vals=vals(ia);
    end
% ------------- END OF CODE --------------
