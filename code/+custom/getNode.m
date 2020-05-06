% Create a new Node (or copy an existing one).
function node = getNode(varargin)
    if nargin == 0
        % create an empty node
        node.node = [];
        node.cost = 0;
        node.height = 0;
        node.rotation = [1,0,0;0,1,0;0,0,1];
        node.cumulative = [1,0,0;0,1,0;0,0,1];        
        node.parent = 0;
    elseif nargin == 1
        % just return a copy
        node = varargin{1};
    else
        error("Only 0 or 1 arguments supported.");
    end
end %function