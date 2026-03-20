% Hungarian deals with distmMat with negative elements
% Hungarian's algorithm finds out the minimumassignment.
%IN: distMat is an  nMeas*nTarg cost matrix.
%OUT: assgmt is an nMeas*1 matrix giving the ith minimum assignment ,cost is the cost of this assignment.

function [assgmt, cost] = Hungarian(distMat)

minEmt = min(distMat(:)); % minimum element in distMat

if minEmt >= 0
    [assgmt, cost] = assignmentoptimal(distMat);
else
    distMat = distMat - minEmt;
    [assgmt, cost] = assignmentoptimal(distMat);
    cost = cost + minEmt*size(distMat, 1);
end

% if ismember(0, assgmt)
%     error('All possible assignments leads to infinite cost.');
% end