% Based on P. Corke's tranimate.m script.
% This is a hack. The code is not guaranteed to be bug-free or reliable.
% Do not use in your project without making sure every line below is valid.
function ptranimate(P2, varargin)

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
    
    hg = trplot(eye(4,4), args{:});  % create a frame at the origin

    % animate it for all poses in the sequence
    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        
        trplot(hg, T);
        x_axis = T(1:3, 1);
        y_axis = T(1:3, 2);
        z_axis = T(1:3, 3);
        
        plot3(x_axis(1), x_axis(2), x_axis(3), '*');
        plot3(y_axis(1), y_axis(2), y_axis(3), '*');
        plot3(z_axis(1), z_axis(2), z_axis(3), '*');
        
        if i > 1,
          Tprev = Ttraj(:,:,i-1);
          [theta, vec] = tr2angvec(T * inv(Tprev));
          vec = vec * 1.2;
          plot2([ vec ; -vec ], 'r');
          %plot3(vec(1), vec(2), vec(3), 'r*');
          %plot3(-vec(1), -vec(2), -vec(3), 'r*');
        end
        
        if ~isempty(opt.movie)
            f = getframe;
            imwrite(f.cdata, sprintf('%s/%04d.png', opt.movie, framenum));
            framenum = framenum+1;
        end
        
        pause(1/opt.fps);
    end
