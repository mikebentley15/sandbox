function [p, FV, colourVector]=plotVoxelArray(bw, polytype, colour)
% plotVoxelArray - displays the surface of a voxel array as a patch
% object
%
% optionally the mesh colour can be defined manually or by the values of
% the voxel array
% 
% Syntax:  
%     p=plotVoxelArray(bw)
%     p=plotVoxelArray(bw, polytype)
%     [p, FV]=plotVoxelArray(bw, polytype, colour)
%     [p, FV, colourVector]=plotVoxelArray(bw, polytype, colour)
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
%   colour - Optionally define mesh colour using 
%       - string argument (as used by built in patch function. e.g. 'r',
%           'red',etc) -> DEFAULT ='b'
%       - RGB triplet (e.g. [1 0 1] for magenta
%       - by voxel values; if colour is set to true/1 vertex colours will
%           be defiend by the numeric values of the input array
% 
% Outputs:
%    p  - patch object
%    FV - FV struct; FV.faces/FV.vertices defining the mesh
%    colourVector – optionally save colourVector (setting the colourVector
%       output will overeride any "colour" input variable and define vertex
%       colours by voxel values
% 
% Example 1 (binary input and default): 
%     % take any binary array as input 
%     % in this case I am using the function starship_voxel.m from 
%           https://www.mathworks.com/matlabcentral/fileexchange/75241-build-mesh-or-voxel-spheres-ellipsoid-and-test-objects
%     BW=starship_voxel;
% 
%     % generate triangulated surface mesh
%     figure
%     p=plotVoxelArray(BW); 
%     p.FaceColor='b'; 
%     axis equal
%
% Example 2 (color data):
%     % take any binary array as input 
%     % in this case I am using the function voxelSphereCreator.m
%     % (https://www.mathworks.com/matlabcentral/fileexchange/75241-build-mesh-or-voxel-spheres-ellipsoid-and-test-objects)
%     % to build a matrix with two overlapping spheres with two different
%     % values
%     bw=zeros(50, 50, 50); 
%     bw=voxelSphereCreator(bw, 10, 15, 25, 25).*2.73425; 
%     bw=voxelSphereCreator(bw, 10, 30, 25, 25); 
% 
%     % plot as red triangle mesh
%     figure
%     [p, FV]=plotVoxelArray(bw, 3, 'red');
%     axis equal
% 
%     % plot as quad mesh using voxel values to define mesh colours
%     plotVoxelArray(bw, 4, 1); 
%     colormap cool
%     axis equal
% 
%     % plot as quad mesh using voxel values to define mesh colours (by calling
%     % the outputvariable "colourVector" without deefining the "colour" input
%     [p, FV, colourVector]=plotVoxelArray(bw, 4); 
%     colormap cool
%     axis equal
% 
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
% 
% See also: Voxel2mesh.m
% Author: J. Benjamin Kacerovsky
% Centre for Research in Neuroscience, McGill University
% email: johannes.kacerovsky@mail.mcgill.ca
% Created: 27-Apr-2020 ; Last revision: 27-Apr-2020 
% ------------- BEGIN CODE --------------
% set polytype = 3 to plot voxel Array as triangulated mesh
if nargin < 2
    polytype =4;    % default to quad-mesh
end
if nargin < 3||isequal(colour, false)
    colour='b';    % default do not extract color vector
end
if isequal(colour, true)||nargout==3
    if polytype==4 
        [FV, colourVector]=Voxel2mesh(bw, 4);
    else
        [FV, colourVector]=Voxel2mesh(bw, 3); 
    end
    p=patch(FV, 'FaceColor', 'interp', 'facevertexcdata', colourVector); 
else
    if polytype==4 
        FV=Voxel2mesh(bw, 4);
    else
        FV=Voxel2mesh(bw, 3); 
    end
    p=patch(FV, 'FaceColor', colour); 
end
% ------------- END OF CODE --------------
