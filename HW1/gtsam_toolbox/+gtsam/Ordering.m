%class Ordering, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Ordering()
%Ordering(Ordering other)
%
%-------Methods-------
%at(size_t key) : returns size_t
%equals(Ordering ord, double tol) : returns bool
%print(string s) : returns void
%push_back(size_t key) : returns void
%size() : returns size_t
%
classdef Ordering < handle
  properties
    ptr_gtsamOrdering = 0
  end
  methods
    function obj = Ordering(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(931, my_ptr);
      elseif nargin == 0
        my_ptr = gtsam_wrapper(932);
      elseif nargin == 1 && isa(varargin{1},'gtsam.Ordering')
        my_ptr = gtsam_wrapper(933, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.Ordering constructor');
      end
      obj.ptr_gtsamOrdering = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(934, obj.ptr_gtsamOrdering);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = at(this, varargin)
      % AT usage: at(size_t key) : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(935, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Ordering ord, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.Ordering') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(936, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Ordering.equals');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(937, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Ordering.print');
      end
    end

    function varargout = push_back(this, varargin)
      % PUSH_BACK usage: push_back(size_t key) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(938, this, varargin{:});
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(939, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
