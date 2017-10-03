%class Rot2, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Rot2()
%Rot2(double theta)
%
%-------Methods-------
%between(Rot2 p2) : returns gtsam::Rot2
%c() : returns double
%compose(Rot2 p2) : returns gtsam::Rot2
%degrees() : returns double
%dim() : returns size_t
%equals(Rot2 rot, double tol) : returns bool
%inverse() : returns gtsam::Rot2
%localCoordinates(Rot2 p) : returns Vector
%matrix() : returns Matrix
%print(string s) : returns void
%retract(Vector v) : returns gtsam::Rot2
%rotate(Point2 point) : returns gtsam::Point2
%s() : returns double
%theta() : returns double
%unrotate(Point2 point) : returns gtsam::Point2
%
%-------Static Methods-------
%Dim() : returns size_t
%Expmap(Vector v) : returns gtsam::Rot2
%Logmap(Rot2 p) : returns Vector
%atan2(double y, double x) : returns gtsam::Rot2
%fromAngle(double theta) : returns gtsam::Rot2
%fromCosSin(double c, double s) : returns gtsam::Rot2
%fromDegrees(double theta) : returns gtsam::Rot2
%identity() : returns gtsam::Rot2
%relativeBearing(Point2 d) : returns gtsam::Rot2
%
classdef Rot2 < gtsam.Value
  properties
    ptr_gtsamRot2 = 0
  end
  methods
    function obj = Rot2(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(123, varargin{2});
        end
        base_ptr = gtsam_wrapper(122, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = gtsam_wrapper(124);
      elseif nargin == 1 && isa(varargin{1},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(125, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.Rot2 constructor');
      end
      obj = obj@gtsam.Value(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamRot2 = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(126, obj.ptr_gtsamRot2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = between(this, varargin)
      % BETWEEN usage: between(Rot2 p2) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Rot2')
        varargout{1} = gtsam_wrapper(127, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.between');
      end
    end

    function varargout = c(this, varargin)
      % C usage: c() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(128, this, varargin{:});
    end

    function varargout = compose(this, varargin)
      % COMPOSE usage: compose(Rot2 p2) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Rot2')
        varargout{1} = gtsam_wrapper(129, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.compose');
      end
    end

    function varargout = degrees(this, varargin)
      % DEGREES usage: degrees() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(130, this, varargin{:});
    end

    function varargout = dim(this, varargin)
      % DIM usage: dim() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(131, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Rot2 rot, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.Rot2') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(132, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.equals');
      end
    end

    function varargout = inverse(this, varargin)
      % INVERSE usage: inverse() : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(133, this, varargin{:});
    end

    function varargout = localCoordinates(this, varargin)
      % LOCALCOORDINATES usage: localCoordinates(Rot2 p) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Rot2')
        varargout{1} = gtsam_wrapper(134, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.localCoordinates');
      end
    end

    function varargout = matrix(this, varargin)
      % MATRIX usage: matrix() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(135, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(136, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.print');
      end
    end

    function varargout = retract(this, varargin)
      % RETRACT usage: retract(Vector v) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = gtsam_wrapper(137, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.retract');
      end
    end

    function varargout = rotate(this, varargin)
      % ROTATE usage: rotate(Point2 point) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = gtsam_wrapper(138, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.rotate');
      end
    end

    function varargout = s(this, varargin)
      % S usage: s() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(139, this, varargin{:});
    end

    function varargout = theta(this, varargin)
      % THETA usage: theta() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(140, this, varargin{:});
    end

    function varargout = unrotate(this, varargin)
      % UNROTATE usage: unrotate(Point2 point) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = gtsam_wrapper(141, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.unrotate');
      end
    end

  end

  methods(Static = true)
    function varargout = Dim(varargin)
      % DIM usage: Dim() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % DIM()
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(142, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.Dim');
      end
    end

    function varargout = Expmap(varargin)
      % EXPMAP usage: Expmap(Vector v) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % EXPMAP(Vector v)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = gtsam_wrapper(143, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.Expmap');
      end
    end

    function varargout = Logmap(varargin)
      % LOGMAP usage: Logmap(Rot2 p) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % LOGMAP(Rot2 p)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Rot2')
        varargout{1} = gtsam_wrapper(144, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.Logmap');
      end
    end

    function varargout = Atan2(varargin)
      % ATAN2 usage: atan2(double y, double x) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % ATAN2(double y, double x)
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(145, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.Atan2');
      end
    end

    function varargout = FromAngle(varargin)
      % FROMANGLE usage: fromAngle(double theta) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % FROMANGLE(double theta)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = gtsam_wrapper(146, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.FromAngle');
      end
    end

    function varargout = FromCosSin(varargin)
      % FROMCOSSIN usage: fromCosSin(double c, double s) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % FROMCOSSIN(double c, double s)
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(147, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.FromCosSin');
      end
    end

    function varargout = FromDegrees(varargin)
      % FROMDEGREES usage: fromDegrees(double theta) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % FROMDEGREES(double theta)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = gtsam_wrapper(148, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.FromDegrees');
      end
    end

    function varargout = Identity(varargin)
      % IDENTITY usage: identity() : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % IDENTITY()
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(149, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.Identity');
      end
    end

    function varargout = RelativeBearing(varargin)
      % RELATIVEBEARING usage: relativeBearing(Point2 d) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % RELATIVEBEARING(Point2 d)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = gtsam_wrapper(150, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Rot2.RelativeBearing');
      end
    end

  end
end
