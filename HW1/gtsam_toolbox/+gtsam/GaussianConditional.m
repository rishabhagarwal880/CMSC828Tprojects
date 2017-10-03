%class GaussianConditional, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianConditional(size_t key, Vector d, Matrix R, Diagonal sigmas)
%GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S, Diagonal sigmas)
%GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S, size_t name2, Matrix T, Diagonal sigmas)
%GaussianConditional(size_t key, Vector d, Matrix R)
%GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S)
%GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S, size_t name2, Matrix T)
%
%-------Methods-------
%augmentedInformation() : returns Matrix
%augmentedJacobian() : returns Matrix
%clone() : returns gtsam::GaussianFactor
%empty() : returns bool
%equals(GaussianConditional cg, double tol) : returns bool
%error(VectorValues c) : returns double
%information() : returns Matrix
%jacobian() : returns pair< Matrix, Vector >
%negate() : returns gtsam::GaussianFactor
%print(string s) : returns void
%scaleFrontalsBySigma(VectorValues gy) : returns void
%size() : returns size_t
%solve(VectorValues parents) : returns gtsam::VectorValues
%solveOtherRHS(VectorValues parents, VectorValues rhs) : returns gtsam::VectorValues
%solveTransposeInPlace(VectorValues gy) : returns void
%
classdef GaussianConditional < gtsam.GaussianFactor
  properties
    ptr_gtsamGaussianConditional = 0
  end
  methods
    function obj = GaussianConditional(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(770, varargin{2});
        end
        base_ptr = gtsam_wrapper(769, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(771, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(772, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      elseif nargin == 8 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'numeric') && isa(varargin{7},'double') && isa(varargin{8},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(773, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7}, varargin{8});
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(774, varargin{1}, varargin{2}, varargin{3});
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(775, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      elseif nargin == 7 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'numeric') && isa(varargin{7},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(776, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7});
      else
        error('Arguments do not match any overload of gtsam.GaussianConditional constructor');
      end
      obj = obj@gtsam.GaussianFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamGaussianConditional = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(777, obj.ptr_gtsamGaussianConditional);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = augmentedInformation(this, varargin)
      % AUGMENTEDINFORMATION usage: augmentedInformation() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(778, this, varargin{:});
    end

    function varargout = augmentedJacobian(this, varargin)
      % AUGMENTEDJACOBIAN usage: augmentedJacobian() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(779, this, varargin{:});
    end

    function varargout = clone(this, varargin)
      % CLONE usage: clone() : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(780, this, varargin{:});
    end

    function varargout = empty(this, varargin)
      % EMPTY usage: empty() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(781, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(GaussianConditional cg, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.GaussianConditional') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(782, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.equals');
      end
    end

    function varargout = error(this, varargin)
      % ERROR usage: error(VectorValues c) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(783, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.error');
      end
    end

    function varargout = information(this, varargin)
      % INFORMATION usage: information() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(784, this, varargin{:});
    end

    function varargout = jacobian(this, varargin)
      % JACOBIAN usage: jacobian() : returns pair< Matrix, Vector >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = gtsam_wrapper(785, this, varargin{:});
    end

    function varargout = negate(this, varargin)
      % NEGATE usage: negate() : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(786, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(787, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.print');
      end
    end

    function varargout = scaleFrontalsBySigma(this, varargin)
      % SCALEFRONTALSBYSIGMA usage: scaleFrontalsBySigma(VectorValues gy) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        gtsam_wrapper(788, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.scaleFrontalsBySigma');
      end
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(789, this, varargin{:});
    end

    function varargout = solve(this, varargin)
      % SOLVE usage: solve(VectorValues parents) : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(790, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.solve');
      end
    end

    function varargout = solveOtherRHS(this, varargin)
      % SOLVEOTHERRHS usage: solveOtherRHS(VectorValues parents, VectorValues rhs) : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.VectorValues') && isa(varargin{2},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(791, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.solveOtherRHS');
      end
    end

    function varargout = solveTransposeInPlace(this, varargin)
      % SOLVETRANSPOSEINPLACE usage: solveTransposeInPlace(VectorValues gy) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        gtsam_wrapper(792, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianConditional.solveTransposeInPlace');
      end
    end

  end

  methods(Static = true)
  end
end
