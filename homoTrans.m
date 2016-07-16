function t = homoTrans(P, v)
    
    [dim,npts] = size(v);
    
    if ~all(size(P)==dim)
        error('Transformation matrix and point dimensions do not match');
    end

    t = P*v;            % Transform

    for r = 1:dim-1     %  Now normalise    
        t(r,:) = t(r,:)./t(end,:);
    end
    
    t(end,:) = ones(1,npts);
    