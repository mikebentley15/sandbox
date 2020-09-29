function sphereImage = voxelSphereCreator(r, varargin)
% VoxelSphereCreator - creates a voxelated sphere 
% Creates a voxelated sphere of defined radius at defined centre point
% coordinates of the input image/matrix (matrix element=voxel).
% Optional inputs can be added to create deformed spheres. 
% Sphere-voxels will be set to 1 in the output matrix
% 
% Syntax:  
%     sphereImage = voxelSphereCreator(r)
%     sphereImage = voxelSphereCreator(r, centrePoint)
%     sphereImage = voxelSphereCreator(r, centrePoint, sphereImage)
%     sphereImage = voxelSphereCreator(r, centrePoint, sphereImage, ..., 'Name', value)
%     sphereImage = voxelSphereCreator(r, ..., 'Name', value)
% 
% Inputs:
%    r - scalar; radius of the sphere
%    centrePoint – OPTIONAL: 1x3 vector defining the center point of the sphere;
%           default=[r+1; r+1, r+1]
%    sphereImage - OPTIONAL: target matrix NxNxN; sphere will be added to the input
%           matrix as voxels value 1 (allows multiple spheres or objects to
%           be added to the same input matrix, by repeatedly calling 
%           voxelSphereCreator and using the output as next input) 
%               (to create many spheres more easily see:
%               multiVoxelElipsoidCreator.m)
%           if no input matrix is provided an empty matrix of
%           [centrePoint]+r+1 will be created
%    
% Optional Inputs as Name-Value pairs
%    
%   'Ysquash' - scalar; sphere is "squashed" N-fold along the y axis
%   'Xsquash' - scalar; sphere is "squashed" N-fold along the x axis
%   'Zsquash' - scalar; sphere is "squashed" N-fold along the z axis
%   'resample' - scalar; sphere is resampled to a grid with N-fold higher 
%       resolution for more precise boundaries (using built-in interp3 with
%       on a meshgird with stepsize 1/N); usually not necessary; high N can
%       increase computation time dramatically; 
%    
% Outputs:
%    sphereImage - same as input sphereImage with voxels in the newly created
%    sphere set to 1
% 
% Example: 
%     % for visualizing the result these examples use the plotVoxelArray function
%     % Voxel2mesh - plotVoxelArray (https://www.mathworks.com/matlabcentral/fileexchange/75240-voxel2mesh-plotvoxelarray), MATLAB Central File Exchange. Retrieved June 6, 2020.%
%     % build a sphere r=10 at the center of a 51x51x51 matrix
% 
%     % simple voxel sphere with r=10
%     sp=voxelSphereCreator(10);
%     plotVoxelArray(sp);
%     axis equal
%     %%
%     % simple voxel sphere with r=10 centred at [20, 30, 40]
%     figure
%     sp=voxelSphereCreator(10, [20, 30, 40]);
%     plotVoxelArray(sp);
%     axis equal
% 
%     %%
% 
%     % use 'Ysquash' to create a disc instead
%     figure
%     sp=voxelSphereCreator(10, [20, 30, 40], 'Ysquash', 2);
%     plotVoxelArray(sp);
%     axis equal
% 
%     % sphereImages can be added and subtracted to create testObjects
%     sp=zeros(51, 51, 51); 
%     S=voxelSphereCreator(15, [25, 25, 25], sp);
%     S=voxelSphereCreator(10, [25, 35, 25], S, 'Xsquash', 3, 'Zsquash', 3);
%     S=S-voxelSphereCreator(20, [10, 25, 30], sp, 'Ysquash', 2);
%     plotVoxelArray(S);
%     axis equal
% 
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
% 
% Author: J. Benjamin Kacerovsky
% Centre for Research in Neuroscience, McGill University
% email: johannes.kacerovsky@mail.mcgill.ca
% Oct-2019 ; Last revision: 05-Jun-2020 
% ------------- BEGIN CODE --------------
p=inputParser;
addRequired(p, 'r'); 
addOptional(p, 'centrePoint', [r+1, r+1, r+1], @(x) isnumeric(x)&&(numel(x)==3)); 
addOptional(p, 'sphereImage', NaN, @isnumeric); 
addParameter(p, 'Xsquash', 1, @isnumeric);
addParameter(p, 'Ysquash', 1, @isnumeric);
addParameter(p, 'Zsquash', 1, @isnumeric);
addParameter(p, 'resample', 1, @isnumeric);

parse(p, r, varargin{:});

centrePoint=p.Results.centrePoint;
sphereImage=p.Results.sphereImage;
Ys=p.Results.Xsquash; 
Xs=p.Results.Ysquash;
Zs=p.Results.Zsquash;
RR=p.Results.resample;

x=centrePoint(1); 
y=centrePoint(2); 
z=centrePoint(3); 
if isnan(sphereImage)
    sphereImage=zeros(x+r+1, y+r+1, z+r+1); 
end

rr=round(100/RR) / 100;
    [X, Y, Z]=meshgrid(1:rr:size(sphereImage, 2), 1:rr:size(sphereImage, 1), 1:rr:size(sphereImage, 3));
    A=sqrt(((X-y)*Xs).^2+((Y-x)*Ys).^2+((Z-z)*Zs).^2);
    sp=A<=r;
    
    if rr~=1
        disp('resample');
        sp=single(sp);
        [Xb, Yb, Zb]=meshgrid(1:1:size(sphereImage, 1), 1:1:size(sphereImage, 2), 1:1:size(sphereImage, 3));
        sp = interp3(X,Y,Z,sp,Xb,Yb,Zb);
        sp = sp>0.5;
    end
    
   sphereImage(sp)=1;
   
end
% ------------- END OF CODE --------------
