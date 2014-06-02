% Based on P. Corke's tranimate.m script.
% This is a hack. The code is not guaranteed to be bug-free or reliable.
% Do not use in your project without making sure every line below is valid.
function rpyanimate(P2, varargin)

    opt.fps = 10;
    opt.nsteps = 50;
    opt.axis = [];
    opt.movie = [];

    [opt, args] = tb_optparse(opt, varargin);
    
    if ~isempty(opt.movie)
        mkdir(opt.movie);
        framenum = 1;
    end
    P1 = [];

    % convert quaternion and rotation matrix to hom transform
    if isa(P2, 'Quaternion')
        T2 = P2.T;   % convert quaternion to transform
        if ~isempty(args) && isa(args{1},'Quaternion')
            T1 = T2;
            Q2 = args{1};
            T2 = Q2.T;
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif isrot(P2)
        T2 = r2t(P2);
        if ~isempty(args) && isrot(args{1})
            T1 = T2;
            T2 = r2t(args{1});
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif ishomog(P2)
        T2 = P2;
        if ~isempty(args) && ishomog(args{1})
            T1 = T2;
            T2 = args{1};
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    end
    
    % at this point
    %   T1 is the initial pose
    %   T2 is the final pose
    %
    %  T2 may be a sequence
        
    if size(T2,3) > 1
        % tranimate(Ts)
        % we were passed a homog sequence
        if ~isempty(P1)
            error('only 1 input argument if sequence specified');
        end
        Ttraj = T2;
    else
        % tranimate(P1, P2)
        % create a path between them
        Ttraj = ctraj(T1, T2, opt.nsteps);
    end
    
    if isempty(opt.axis)
        % create axis limits automatically based on motion of frame origin
        t = transl(Ttraj);
        mn = min(t) - 1.5;  % min value + length of axis + some
        mx = max(t) + 1.5;  % max value + length of axis + some
        axlim = [mn; mx];
        axlim = axlim(:)';
        args = [args 'axis' axlim];
    end
    
    % animate it for all poses in the sequence
    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        
        rpy = tr2rpy(T);
        
        plot3(rpy(1), rpy(2), rpy(3), '*');
        hold on;
        grid on;
        axis([-2*pi, 2*pi, -2*pi, 2*pi, -2*pi, 2*pi]./2);
                
        if ~isempty(opt.movie)
            f = getframe;
            imwrite(f.cdata, sprintf('%s/%04d.png', opt.movie, framenum));
            framenum = framenum+1;
        end
        pause(1/opt.fps);
    end
