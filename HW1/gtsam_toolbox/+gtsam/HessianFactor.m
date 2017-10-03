%class HessianFactor, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%HessianFactor()
%HessianFactor(GaussianFactor factor)
%HessianFactor(size_t j, Matrix G, Vector g, double f)
%HessianFactor(size_t j, Vector mu, Matrix Sigma)
%HessianFactor(size_t j1, size_t j2, Matrix G11, Matrix G12, Vector g1, Matrix G22, Vector g2, double f)
%HessianFactor(size_t j1, size_t j2, size_t j3, Matrix G11, Matrix G12, Matrix G13, Vector g1, Matrix G22, Matrix G23, Vector g2, Matrix G33, Vector g3, double f)
%HessianFactor(GaussianFactorGraph factors)
%
%-------Methods-------
%augmentedInformation() : returns Matrix
%augmentedJacobian() : returns Matrix
%clone() : returns gtsam::GaussianFactor
%constantTerm() : returns double
%empty() : returns bool
%equals(GaussianFactor lf, double tol) : returns bool
%error(VectorValues c) : returns double
%info() : returns Matrix
%information() : returns Matrix
%jacobian() : returns pair< Matrix, Vector >
%linearTerm() : returns Vector
%negate() : returns gtsam::GaussianFactor
%print(string s) : returns void
%printKeys(string s) : returns void
%rows() : returns size_t
%size() : returns size_t
%
classdef HessianFactor < gtsam.GaussianFactor
  properties
    ptr_gtsamHessianFactor = 0
  end
  methods
    function obj = HessianFactor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(692, varargin{2});
        end
        base_ptr = gtsam_wrapper(691, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = gtsam_wrapper(693);
      elseif nargin == 1 && isa(varargin{1},'gtsam.GaussianFactor')
        [ my_ptr, base_ptr ] = gtsam_wrapper(694, varargin{1});
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(695, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(696, varargin{1}, varargin{2}, varargin{3});
      elseif nargin == 8 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'double') && isa(varargin{5},'double') && isa(varargin{6},'double') && isa(varargin{7},'double') && isa(varargin{8},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(697, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7}, varargin{8});
      elseif nargin == 13 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric') && isa(varargin{4},'double') && isa(varargin{5},'double') && isa(varargin{6},'double') && isa(varargin{7},'double') && isa(varargin{8},'double') && isa(varargin{9},'double') && isa(varargin{10},'double') && isa(varargin{11},'double') && isa(varargin{12},'double') && isa(varargin{13},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(698, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7}, varargin{8}, varargin{9}, varargin{10}, varargin{11}, varargin{12}, varargin{13});
      elseif nargin == 1 && isa(varargin{1},'gtsam.GaussianFactorGraph')
        [ my_ptr, base_ptr ] = gtsam_wrapper(699, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.HessianFactor constructor');
      end
      obj = obj@gtsam.GaussianFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamHessianFactor = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(700, obj.ptr_gtsamHessianFactor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = augmentedInformation(this, varargin)
      % AUGMENTEDINFORMATION usage: augmentedInformation() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(701, this, varargin{:});
    end

    function varargout = augmentedJacobian(this, varargin)
      % AUGMENTEDJACOBIAN usage: augmentedJacobian() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(702, this, varargin{:});
    end

    function varargout = clone(this, varargin)
      % CLONE usage: clone() : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(703, this, varargin{:});
    end

    function varargout = constantTerm(this, varargin)
      % CONSTANTTERM usage: constantTerm() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(704, this, varargin{:});
    end

    function varargout = empty(this, varargin)
      % EMPTY usage: empty() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(705, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(GaussianFactor lf, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.GaussianFactor') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(706, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.HessianFactor.equals');
      end
    end

    function varargout = error(this, varargin)
      % ERROR usage: error(VectorValues c) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(707, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.HessianFactor.error');
      end
    end

    function varargout = info(this, varargin)
      % INFO usage: info() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(708, this, varargin{:});
    end

    function varargout = information(this, varargin)
      % INFORMATION usage: information() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(709, this, varargin{:});
    end

    function varargout = jacobian(this, varargin)
      % JACOBIAN usage: jacobian() : returns pair< Matrix, Vector >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = gtsam_wrapper(710, this, varargin{:});
    end

    function varargout = linearTerm(this, varargin)
      % LINEARTERM usage: linearTerm() : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(711, this, varargin{:});
    end

    function varargout = negate(this, varargin)
      % NEGATE usage: negate() : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(712, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(713, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.HessianFactor.print');
      end
    end

    function varargout = printKeys(this, varargin)
      % PRINTKEYS usage: printKeys(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(714, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.HessianFactor.printKeys');
      end
    end

    function varargout = rows(this, varargin)
      % ROWS usage: rows() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(715, this, varargin{:});
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(716, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
