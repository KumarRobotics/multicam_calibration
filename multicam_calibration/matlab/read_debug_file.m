function frames = read_debug_file(fname)
    f=fopen(fname);
    if ~f
        error('cannot open file %s', fname);
    end
    frames = {};
    [fnum, cnt] = fscanf(f, '%d', 1);
    i = 1;
    while cnt > 0
        camidx = fscanf(f, '%d',1);
        n = fscanf(f, '%d',1);
        wp = fscanf(f, '%f', [3,n]);
        ip2  = fscanf(f, '%f', [2,n]);
        ip1p = fscanf(f, '%f', [2,n]);
        frames{fnum+1}{camidx+1} = {wp, ip2, ip1p};
        [fnum, cnt] = fscanf(f, '%d', 1);
        i = i + 1;
    end
    fclose(f);
    fprintf(1, 'read %d frames\n', i-1);
end