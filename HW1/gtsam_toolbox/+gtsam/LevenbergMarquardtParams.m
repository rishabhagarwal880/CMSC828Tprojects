%class LevenbergMarquardtParams, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%LevenbergMarquardtParams()
%
%-------Methods-------
%getAbsoluteErrorTol() : returns double
%getErrorTol() : returns double
%getLinearSolverType() : returns string
%getMaxIterations() : returns int
%getRelativeErrorTol() : returns double
%getVerbosity() : returns string
%getVerbosityLM() : returns string
%getlambdaFactor() : returns double
%getlambdaInitial() : returns double
%getlambdaUpperBound() : returns double
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
%setVerbosityLM(string s) : returns void
%setlambdaFactor(double value) : returns void
%setlambdaInitial(double value) : returns void
%setlambdaUpperBound(double value) : returns void
%
classdef LevenbergMarquardtParams < gtsam.NonlinearOptimizerParams
  properties
    ptr_gtsamLevenbergMarquardtParams = 0
  end
  methods
    function obj = LevenbergMarquardtParams(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(1145, varargin{2});
        end
        base_ptr = gtsam_wrapper(1144, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = gtsam_wrapper(1146);
      else
        error('Arguments do not match any overload of gtsam.LevenbergMarquardtParams constructor');
      end
      obj = obj@gtsam.NonlinearOptimizerParams(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamLevenbergMarquardtParams = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(1147, obj.ptr_gtsamLevenbergMarquardtParams);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = getAbsoluteErrorTol(this, varargin)
      % GETABSOLUTEERRORTOL usage: getAbsoluteErrorTol() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1148, this, varargin{:});
    end

    function varargout = getErrorTol(this, varargin)
      % GETERRORTOL usage: getErrorTol() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1149, this, varargin{:});
    end

    function varargout = getLinearSolverType(this, varargin)
      % GETLINEARSOLVERTYPE usage: getLinearSolverType() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1150, this, varargin{:});
    end

    function varargout = getMaxIterations(this, varargin)
      % GETMAXITERATIONS usage: getMaxIterations() : returns int
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1151, this, varargin{:});
    end

    function varargout = getRelativeErrorTol(this, varargin)
      % GETRELATIVEERRORTOL usage: getRelativeErrorTol() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1152, this, varargin{:});
    end

    function varargout = getVerbosity(this, varargin)
      % GETVERBOSITY usage: getVerbosity() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1153, this, varargin{:});
    end

    function varargout = getVerbosityLM(this, varargin)
      % GETVERBOSITYLM usage: getVerbosityLM() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1154, this, varargin{:});
    end

    function varargout = getlambdaFactor(this, varargin)
      % GETLAMBDAFACTOR usage: getlambdaFactor() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1155, this, varargin{:});
    end

    function varargout = getlambdaInitial(this, varargin)
      % GETLAMBDAINITIAL usage: getlambdaInitial() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1156, this, varargin{:});
    end

    function varargout = getlambdaUpperBound(this, varargin)
      % GETLAMBDAUPPERBOUND usage: getlambdaUpperBound() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1157, this, varargin{:});
    end

    function varargout = isCholmod(this, varargin)
      % ISCHOLMOD usage: isCholmod() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1158, this, varargin{:});
    end

    function varargout = isIterative(this, varargin)
      % ISITERATIVE usage: isIterative() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1159, this, varargin{:});
    end

    function varargout = isMultifrontal(this, varargin)
      % ISMULTIFRONTAL usage: isMultifrontal() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1160, this, varargin{:});
    end

    function varargout = isSequential(this, varargin)
      % ISSEQUENTIAL usage: isSequential() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1161, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1162, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.LevenbergMarquardtParams.print');
      end
    end

    function varargout = setAbsoluteErrorTol(this, varargin)
      % SETABSOLUTEERRORTOL usage: setAbsoluteErrorTol(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1163, this, varargin{:});
    end

    function varargout = setErrorTol(this, varargin)
      % SETERRORTOL usage: setErrorTol(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1164, this, varargin{:});
    end

    function varargout = setIterativeParams(this, varargin)
      % SETITERATIVEPARAMS usage: setIterativeParams(IterativeOptimizationParameters params) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.IterativeOptimizationParameters')
        gtsam_wrapper(1165, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.LevenbergMarquardtParams.setIterativeParams');
      end
    end

    function varargout = setLinearSolverType(this, varargin)
      % SETLINEARSOLVERTYPE usage: setLinearSolverType(string solver) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1166, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.LevenbergMarquardtParams.setLinearSolverType');
      end
    end

    function varargout = setMaxIterations(this, varargin)
      % SETMAXITERATIONS usage: setMaxIterations(int value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1167, this, varargin{:});
    end

    function varargout = setOrdering(this, varargin)
      % SETORDERING usage: setOrdering(Ordering ordering) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        gtsam_wrapper(1168, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.LevenbergMarquardtParams.setOrdering');
      end
    end

    function varargout = setRelativeErrorTol(this, varargin)
      % SETRELATIVEERRORTOL usage: setRelativeErrorTol(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1169, this, varargin{:});
    end

    function varargout = setVerbosity(this, varargin)
      % SETVERBOSITY usage: setVerbosity(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1170, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.LevenbergMarquardtParams.setVerbosity');
      end
    end

    function varargout = setVerbosityLM(this, varargin)
      % SETVERBOSITYLM usage: setVerbosityLM(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1171, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.LevenbergMarquardtParams.setVerbosityLM');
      end
    end

    function varargout = setlambdaFactor(this, varargin)
      % SETLAMBDAFACTOR usage: setlambdaFactor(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1172, this, varargin{:});
    end

    function varargout = setlambdaInitial(this, varargin)
      % SETLAMBDAINITIAL usage: setlambdaInitial(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1173, this, varargin{:});
    end

    function varargout = setlambdaUpperBound(this, varargin)
      % SETLAMBDAUPPERBOUND usage: setlambdaUpperBound(double value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1174, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
