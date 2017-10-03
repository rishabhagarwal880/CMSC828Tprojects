function varargout = printKeySet(varargin)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeySet')
        gtsam_wrapper(2356, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'gtsam.KeySet') && isa(varargin{2},'char')
        gtsam_wrapper(2357, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.printKeySet');
      end
