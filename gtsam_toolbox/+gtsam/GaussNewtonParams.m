%class GaussNewtonParams, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussNewtonParams()
%
%-------Methods-------
%getAbsoluteErrorTol() : returns double
%getErrorTol() : returns double
%getLinearSolverType() : returns string
%getMaxIterations() : returns int
%getRelativeErrorTol() : returns double
%getVerbosity() : returns string
%isCholmod() : returns bool
%isIterative() : returns bool
%isMultifrontal() : returns bool
%isSequential() : returns bool
%print(string s) : returns void
%setAbsoluteErrorTol(double value) : returns void
%setErrorTol(double value) : returns void
%setIterativeParams(IterativeOptimizationParameters params) : returns void
%setLinearSolverType(string solver) : returns void
%setMaxIterations(int value) : returns void
%setOrdering(Ordering ordering) : returns void
%setRelativeErrorTol(double value) : returns void
%setVerbosity(string s) : returns void
%
classdef GaussNewtonParams < gtsam.NonlinearOptimizerParams
  properties
    ptr_gtsamGaussNewtonParams = 0
  end
  methods
    function obj = GaussNewtonParams(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(1122, varargin{2});
        end
        base_ptr = gtsam_wrapper(1121, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = gtsam_wrapper(1123);
      else
        error('Arguments do not match any overload of gtsam.GaussNewtonParams constructor');
      end
      obj = obj@gtsam.NonlinearOptimizerParams(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamGaussNewtonParams = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(1124, obj.ptr_gtsamGaussNewtonParams);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = getAbsoluteErrorTol(this, varargin)
      % GETABSOLUTEERRORTOL usage: getAbsoluteErrorTol() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1125, this, varargin{:});
    end

    function varargout = getErrorTol(this, varargin)
      % GETERRORTOL usage: getErrorTol() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1126, this, varargin{:});
    end

    function varargout = getLinearSolverType(this, varargin)
      % GETLINEARSOLVERTYPE usage: getLinearSolverType() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1127, this, varargin{:});
    end

    function varargout = getMaxIterations(this, varargin)
      % GETMAXITERATIONS usage: getMaxIterations() : returns int
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1128, this, varargin{:});
    end

    function varargout = getRelativeErrorTol(this, varargin)
      % GETRELATIVEERRORTOL usage: getRelativeErrorTol() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1129, this, varargin{:});
    end

    function varargout = getVerbosity(this, varargin)
      % GETVERBOSITY usage: getVerbosity() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1130, this, varargin{:});
    end

    function varargout = isCholmod(this, varargin)
      % ISCHOLMOD usage: isCholmod() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1131, this, varargin{:});
    end

    function varargout = isIterative(this, varargin)
      % ISITERATIVE usage: isIterative() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1132, this, varargin{:});
    end

    function varargout = isMultifrontal(this, varargin)
      % ISMULTIFRONTAL usage: isMultifrontal() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1133, this, varargin{:});
    end

    function varargout = isSequential(this, varargin)
      % ISSEQUENTIAL usage: isSequential() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1134, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1135, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussNewtonParams.print');
      end
    end

    function varargout = setAbsoluteErrorTol(this, varargin)
      % SETABSOLUTEERRORTOL usage: setAbsoluteErrorTol(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1136, this, varargin{:});
    end

    function varargout = setErrorTol(this, varargin)
      % SETERRORTOL usage: setErrorTol(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1137, this, varargin{:});
    end

    function varargout = setIterativeParams(this, varargin)
      % SETITERATIVEPARAMS usage: setIterativeParams(IterativeOptimizationParameters params) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.IterativeOptimizationParameters')
        gtsam_wrapper(1138, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussNewtonParams.setIterativeParams');
      end
    end

    function varargout = setLinearSolverType(this, varargin)
      % SETLINEARSOLVERTYPE usage: setLinearSolverType(string solver) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1139, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussNewtonParams.setLinearSolverType');
      end
    end

    function varargout = setMaxIterations(this, varargin)
      % SETMAXITERATIONS usage: setMaxIterations(int value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1140, this, varargin{:});
    end

    function varargout = setOrdering(this, varargin)
      % SETORDERING usage: setOrdering(Ordering ordering) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        gtsam_wrapper(1141, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussNewtonParams.setOrdering');
      end
    end

    function varargout = setRelativeErrorTol(this, varargin)
      % SETRELATIVEERRORTOL usage: setRelativeErrorTol(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1142, this, varargin{:});
    end

    function varargout = setVerbosity(this, varargin)
      % SETVERBOSITY usage: setVerbosity(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1143, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussNewtonParams.setVerbosity');
      end
    end

  end

  methods(Static = true)
  end
end
