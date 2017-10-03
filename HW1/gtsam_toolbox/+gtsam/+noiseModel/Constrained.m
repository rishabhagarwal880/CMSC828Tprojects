%class Constrained, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Methods-------
%unit() : returns gtsam::noiseModel::Constrained
%
%-------Static Methods-------
%All(size_t dim) : returns gtsam::noiseModel::Constrained
%All(size_t dim, double mu) : returns gtsam::noiseModel::Constrained
%MixedPrecisions(Vector mu, Vector precisions) : returns gtsam::noiseModel::Constrained
%MixedPrecisions(Vector precisions) : returns gtsam::noiseModel::Constrained
%MixedSigmas(Vector mu, Vector sigmas) : returns gtsam::noiseModel::Constrained
%MixedSigmas(double m, Vector sigmas) : returns gtsam::noiseModel::Constrained
%MixedVariances(Vector mu, Vector variances) : returns gtsam::noiseModel::Constrained
%MixedVariances(Vector variances) : returns gtsam::noiseModel::Constrained
%
classdef Constrained < gtsam.noiseModel.Diagonal
  properties
    ptr_gtsamnoiseModelConstrained = 0
  end
  methods
    function obj = Constrained(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(551, varargin{2});
        end
        base_ptr = gtsam_wrapper(550, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.noiseModel.Constrained constructor');
      end
      obj = obj@gtsam.noiseModel.Diagonal(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamnoiseModelConstrained = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(552, obj.ptr_gtsamnoiseModelConstrained);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = unit(this, varargin)
      % UNIT usage: unit() : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(553, this, varargin{:});
    end

  end

  methods(Static = true)
    function varargout = All(varargin)
      % ALL usage: All(size_t dim), All(size_t dim, double mu) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % ALL(size_t dim)
      % ALL(size_t dim, double mu)
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = gtsam_wrapper(554, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(555, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.All');
      end
    end

    function varargout = MixedPrecisions(varargin)
      % MIXEDPRECISIONS usage: MixedPrecisions(Vector mu, Vector precisions), MixedPrecisions(Vector precisions) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % MIXEDPRECISIONS(Vector mu, Vector precisions)
      % MIXEDPRECISIONS(Vector precisions)
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(556, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = gtsam_wrapper(557, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.MixedPrecisions');
      end
    end

    function varargout = MixedSigmas(varargin)
      % MIXEDSIGMAS usage: MixedSigmas(Vector mu, Vector sigmas), MixedSigmas(double m, Vector sigmas) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % MIXEDSIGMAS(Vector mu, Vector sigmas)
      % MIXEDSIGMAS(double m, Vector sigmas)
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(558, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(559, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.MixedSigmas');
      end
    end

    function varargout = MixedVariances(varargin)
      % MIXEDVARIANCES usage: MixedVariances(Vector mu, Vector variances), MixedVariances(Vector variances) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Usage
      % MIXEDVARIANCES(Vector mu, Vector variances)
      % MIXEDVARIANCES(Vector variances)
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(560, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = gtsam_wrapper(561, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.MixedVariances');
      end
    end

  end
end
